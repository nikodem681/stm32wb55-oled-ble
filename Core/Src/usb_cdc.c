/*
 * Минимальное USB-устройство класса CDC ACM на регистрах STM32WB.
 *
 * Как это работает в двух словах:
 *  1. Подключаем подтяжку на D+ — компьютер «видит», что вставили устройство.
 *  2. Компьютер шлёт RESET, потом запросы SETUP через конечную точку 0 (EP0):
 *     «дай дескриптор устройства», «вот тебе адрес», «дай конфигурацию»...
 *     Дескрипторы — это таблицы, где описано, кто мы такие (ниже в файле).
 *  3. После SET_CONFIGURATION включаются точки данных: EP1 (bulk, данные
 *     туда-обратно) и EP2 (interrupt, уведомления — по стандарту CDC нужна).
 *
 * Данные USB-периферия берёт/кладёт не в обычную RAM, а в свою память PMA
 * (Packet Memory Area, 1 КБ по адресу USB_PMAADDR). Доступ — 16-битными словами.
 */
#include "usb_cdc.h"
#include "stm32wbxx.h"
#include "task.h"
#include "stream_buffer.h"
#include "semphr.h"
#include <string.h>

#define EP0_SIZE   64
#define DATA_SIZE  64

/* Раскладка памяти PMA (смещения в байтах). В начале — таблица буферов. */
#define PMA_EP0_TX 0x040
#define PMA_EP0_RX 0x080
#define PMA_EP1_TX 0x0C0
#define PMA_EP1_RX 0x100
#define PMA_EP2_TX 0x140

/* COUNT_RX: размер приёмного буфера 64 байта = 2 блока по 32 (BL_SIZE=1, NUM_BLOCK=1) */
#define RX_COUNT_64 (0x8000U | (1U << 10))

static volatile uint16_t *const pma = (volatile uint16_t *)USB_PMAADDR;

/* ------------------------------------------------------------------ */
/*                           Дескрипторы                               */
/* ------------------------------------------------------------------ */

static const uint8_t dev_desc[18] = {
    18, 1,            /* длина, тип = DEVICE */
    0x00, 0x02,       /* USB 2.0 */
    0x02, 0x00, 0x00, /* класс = CDC */
    EP0_SIZE,
    0x83, 0x04,       /* VID 0x0483 (STMicroelectronics) */
    0x40, 0x57,       /* PID 0x5740 (их Virtual COM Port) */
    0x00, 0x01,       /* версия устройства 1.00 */
    1, 2, 3,          /* номера строк: производитель, продукт, серийник */
    1,                /* одна конфигурация */
};

static const uint8_t cfg_desc[67] = {
    /* Конфигурация */
    9, 2, 67, 0, 2, 1, 0, 0x80, 50,       /* 2 интерфейса, питание от шины, 100 мА */
    /* Интерфейс 0: управление CDC */
    9, 4, 0, 0, 1, 0x02, 0x02, 0x01, 0,
    5, 0x24, 0x00, 0x10, 0x01,            /* Header, CDC 1.10 */
    5, 0x24, 0x01, 0x00, 0x01,            /* Call Management */
    4, 0x24, 0x02, 0x02,                  /* ACM: поддерживаем SET_LINE_CODING и т.п. */
    5, 0x24, 0x06, 0x00, 0x01,            /* Union: интерфейс 0 управляет интерфейсом 1 */
    7, 5, 0x82, 0x03, 8, 0, 16,           /* EP2 IN, interrupt, 8 байт, раз в 16 мс */
    /* Интерфейс 1: данные */
    9, 4, 1, 0, 2, 0x0A, 0x00, 0x00, 0,
    7, 5, 0x01, 0x02, DATA_SIZE, 0, 0,    /* EP1 OUT, bulk — от компьютера к нам */
    7, 5, 0x81, 0x02, DATA_SIZE, 0, 0,    /* EP1 IN,  bulk — от нас к компьютеру */
};

static const char *const strings[] = { NULL, "STM32 Learning", "WB55 OLED Virtual COM", "WB55-0001" };

/* ------------------------------------------------------------------ */
/*                    Работа с регистрами конечных точек               */
/* ------------------------------------------------------------------ */

/* Регистры EPnR лежат через 4 байта; у них хитрые биты:
   STAT и DTOG переключаются записью 1, CTR сбрасываются записью 0. */
static volatile uint16_t *epr(int ep) { return &USB->EP0R + ep * 2; }

static void ep_set_tx(int ep, uint16_t stat)
{
    uint16_t v = *epr(ep);
    *epr(ep) = ((v & (USB_EPREG_MASK | USB_EPTX_STAT)) ^ stat) | USB_EP_CTR_RX | USB_EP_CTR_TX;
}

static void ep_set_rx(int ep, uint16_t stat)
{
    uint16_t v = *epr(ep);
    *epr(ep) = ((v & (USB_EPREG_MASK | USB_EPRX_STAT)) ^ stat) | USB_EP_CTR_RX | USB_EP_CTR_TX;
}

static void ep_clear_ctr_rx(int ep) { *epr(ep) = (*epr(ep) & USB_EPREG_MASK & ~USB_EP_CTR_RX) | USB_EP_CTR_TX; }
static void ep_clear_ctr_tx(int ep) { *epr(ep) = (*epr(ep) & USB_EPREG_MASK & ~USB_EP_CTR_TX) | USB_EP_CTR_RX; }

/* Полная настройка точки: тип, адрес, состояния RX/TX, сброс битов чётности DTOG в 0 */
static void ep_setup(int ep, uint16_t type, uint16_t rx, uint16_t tx)
{
    uint16_t cur = *epr(ep) & (USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPRX_STAT | USB_EPTX_STAT);
    *epr(ep) = type | (uint16_t)ep | USB_EP_CTR_RX | USB_EP_CTR_TX | (cur ^ (rx | tx));
}

/* Таблица буферов: для точки n 4 слова — ADDR_TX, COUNT_TX, ADDR_RX, COUNT_RX */
static void btable_set(int ep, uint16_t tx, uint16_t rx)
{
    pma[ep * 4 + 0] = tx;
    pma[ep * 4 + 1] = 0;
    pma[ep * 4 + 2] = rx;
    pma[ep * 4 + 3] = rx ? RX_COUNT_64 : 0;
}

static void pma_write(uint16_t off, const uint8_t *src, uint16_t n)
{
    for (uint16_t i = 0; i < n; i += 2) {
        uint16_t w = src[i];
        if (i + 1 < n) w |= (uint16_t)src[i + 1] << 8;
        pma[(off + i) / 2] = w;
    }
}

static void pma_read(uint16_t off, uint8_t *dst, uint16_t n)
{
    for (uint16_t i = 0; i < n; i += 2) {
        uint16_t w = pma[(off + i) / 2];
        dst[i] = (uint8_t)w;
        if (i + 1 < n) dst[i + 1] = (uint8_t)(w >> 8);
    }
}

static uint16_t rx_count(int ep) { return pma[ep * 4 + 3] & 0x3FF; }

static void ep_send(int ep, uint16_t pma_off, const uint8_t *data, uint16_t n)
{
    pma_write(pma_off, data, n);
    pma[ep * 4 + 1] = n;
    ep_set_tx(ep, USB_EP_TX_VALID);
}

/* ------------------------------------------------------------------ */
/*                        EP0: управляющие запросы                     */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t  bmRequestType, bRequest;
    uint16_t wValue, wIndex, wLength;
} setup_t;

static const uint8_t *ep0_ptr;
static uint16_t ep0_left;
static uint8_t  ep0_zlp;           /* нужен ли пакет нулевой длины в конце */
static uint8_t  pending_addr;      /* адрес применяется ПОСЛЕ ответа на SET_ADDRESS */
static uint8_t  out_line_coding;   /* ждём 7 байт SET_LINE_CODING */
static uint8_t  ep0_buf[64];
static uint8_t  line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 };   /* 115200 8N1 */

static volatile usb_state_t state;
static volatile uint8_t tx_busy, tx_last_full;
static StreamBufferHandle_t rx_sb, tx_sb;
static SemaphoreHandle_t tx_mutex;

static void ep0_chunk(void)
{
    uint16_t n = ep0_left > EP0_SIZE ? EP0_SIZE : ep0_left;
    ep_send(0, PMA_EP0_TX, ep0_ptr, n);
    ep0_ptr += n;
    ep0_left -= n;
}

/* Ответ в фазе данных (IN). len == 0 — просто подтверждение (ZLP). */
static void ep0_reply(const uint8_t *data, uint16_t len, uint16_t wLength)
{
    if (len > wLength) len = wLength;
    ep0_ptr  = data;
    ep0_left = len;
    ep0_zlp  = (len < wLength) && (len % EP0_SIZE == 0) && len;
    ep0_chunk();
}

static void ep0_stall(void)
{
    ep_set_tx(0, USB_EP_TX_STALL);
}

static uint16_t make_string(uint8_t idx)
{
    if (idx == 0) {                              /* список языков: 0x0409 = English (US) */
        ep0_buf[0] = 4; ep0_buf[1] = 3; ep0_buf[2] = 0x09; ep0_buf[3] = 0x04;
        return 4;
    }
    const char *s = strings[idx];
    uint16_t n = 2;
    for (; *s && n < sizeof ep0_buf - 1; s++) { ep0_buf[n++] = (uint8_t)*s; ep0_buf[n++] = 0; }   /* UTF-16 */
    ep0_buf[0] = (uint8_t)n; ep0_buf[1] = 3;
    return n;
}

static void configure_data_eps(void)
{
    ep_setup(1, USB_EP_BULK,      USB_EP_RX_VALID, USB_EP_TX_NAK);
    ep_setup(2, USB_EP_INTERRUPT, 0,               USB_EP_TX_NAK);
    tx_busy = 0;
    tx_last_full = 0;
}

static void handle_setup(const setup_t *s)
{
    switch ((s->bmRequestType << 8) | s->bRequest) {

    case 0x8006: {                                   /* GET_DESCRIPTOR */
        uint8_t type = s->wValue >> 8, idx = s->wValue & 0xFF;
        if (type == 1)                    ep0_reply(dev_desc, sizeof dev_desc, s->wLength);
        else if (type == 2)               ep0_reply(cfg_desc, sizeof cfg_desc, s->wLength);
        else if (type == 3 && idx < 4)    ep0_reply(ep0_buf, make_string(idx), s->wLength);
        else                              ep0_stall();   /* напр. DEVICE_QUALIFIER — мы Full Speed */
        break;
    }
    case 0x0005:                                     /* SET_ADDRESS */
        pending_addr = s->wValue & 0x7F;
        ep0_reply(NULL, 0, 0);
        break;
    case 0x0009:                                     /* SET_CONFIGURATION */
        if (s->wValue) { configure_data_eps(); state = USB_STATE_CONFIGURED; }
        else state = USB_STATE_OFF;
        ep0_reply(NULL, 0, 0);
        break;
    case 0x8008:                                     /* GET_CONFIGURATION */
        ep0_buf[0] = state != USB_STATE_OFF;
        ep0_reply(ep0_buf, 1, s->wLength);
        break;
    case 0x8000: case 0x8100: case 0x8200:           /* GET_STATUS */
        ep0_buf[0] = ep0_buf[1] = 0;
        ep0_reply(ep0_buf, 2, s->wLength);
        break;
    case 0x810A:                                     /* GET_INTERFACE */
        ep0_buf[0] = 0;
        ep0_reply(ep0_buf, 1, s->wLength);
        break;
    case 0x0001: case 0x0101: case 0x0201:           /* CLEAR_FEATURE */
    case 0x0003: case 0x0203:                        /* SET_FEATURE */
    case 0x010B:                                     /* SET_INTERFACE */
        ep0_reply(NULL, 0, 0);
        break;

    /* --- запросы класса CDC --- */
    case 0x2120:                                     /* SET_LINE_CODING: скорость, биты... */
        out_line_coding = 1;                         /* 7 байт придут следующим OUT-пакетом */
        break;
    case 0xA121:                                     /* GET_LINE_CODING */
        ep0_reply(line_coding, sizeof line_coding, s->wLength);
        break;
    case 0x2122:                                     /* SET_CONTROL_LINE_STATE: бит 0 = DTR */
        if (state != USB_STATE_OFF)
            state = (s->wValue & 1) ? USB_STATE_PORT_OPEN : USB_STATE_CONFIGURED;
        ep0_reply(NULL, 0, 0);
        break;
    case 0x2123:                                     /* SEND_BREAK */
        ep0_reply(NULL, 0, 0);
        break;

    default:
        ep0_stall();                                 /* не знаем такой запрос — «STALL» */
        break;
    }
}

static void ep0_event(uint16_t istr)
{
    if (istr & USB_ISTR_DIR) {                       /* что-то пришло от компьютера */
        uint16_t v = *epr(0);
        if (v & USB_EP_SETUP) {
            uint8_t raw[8];
            pma_read(PMA_EP0_RX, raw, 8);
            ep_clear_ctr_rx(0);
            setup_t s = {
                raw[0], raw[1],
                (uint16_t)(raw[2] | raw[3] << 8),
                (uint16_t)(raw[4] | raw[5] << 8),
                (uint16_t)(raw[6] | raw[7] << 8),
            };
            out_line_coding = 0;
            handle_setup(&s);
        } else {
            uint16_t n = rx_count(0);
            if (out_line_coding && n >= sizeof line_coding) {
                pma_read(PMA_EP0_RX, line_coding, sizeof line_coding);
                out_line_coding = 0;
                ep_clear_ctr_rx(0);
                ep0_reply(NULL, 0, 0);               /* подтверждаем приём */
            } else {
                ep_clear_ctr_rx(0);                  /* статус-фаза после нашего IN */
            }
        }
        ep_set_rx(0, USB_EP_RX_VALID);
    } else {                                         /* наш IN-пакет ушёл */
        ep_clear_ctr_tx(0);
        if (pending_addr) {
            USB->DADDR = USB_DADDR_EF | pending_addr;
            pending_addr = 0;
        }
        if (ep0_left)      ep0_chunk();
        else if (ep0_zlp) { ep0_zlp = 0; ep0_chunk(); }
    }
}

/* ------------------------------------------------------------------ */
/*                       EP1: данные COM-порта                         */
/* ------------------------------------------------------------------ */

/* Отправить следующий пакет из tx_sb. Вызывается при запрещённом прерывании USB. */
static void tx_kick(BaseType_t from_isr, BaseType_t *woken)
{
    uint8_t pkt[DATA_SIZE];
    size_t n = from_isr ? xStreamBufferReceiveFromISR(tx_sb, pkt, sizeof pkt, woken)
                        : xStreamBufferReceive(tx_sb, pkt, sizeof pkt, 0);
    if (n == 0 && !tx_last_full) { tx_busy = 0; return; }
    /* Если последний пакет был полным (64 байта), компьютер ждёт «конец» —
       пакет нулевой длины, иначе данные могут застрять в его буфере. */
    tx_last_full = (n == DATA_SIZE);
    tx_busy = 1;
    ep_send(1, PMA_EP1_TX, pkt, (uint16_t)n);
}

static void ep1_event(uint16_t istr, BaseType_t *woken)
{
    uint16_t v = *epr(1);
    if (v & USB_EP_CTR_RX) {
        uint8_t pkt[DATA_SIZE];
        uint16_t n = rx_count(1);
        pma_read(PMA_EP1_RX, pkt, n);
        ep_clear_ctr_rx(1);
        xStreamBufferSendFromISR(rx_sb, pkt, n, woken);   /* не влезло — теряем, для учёбы ок */
        ep_set_rx(1, USB_EP_RX_VALID);
    }
    if (v & USB_EP_CTR_TX) {
        ep_clear_ctr_tx(1);
        tx_kick(pdTRUE, woken);
    }
    (void)istr;
}

/* ------------------------------------------------------------------ */
/*                       Прерывание и инициализация                    */
/* ------------------------------------------------------------------ */

static void usb_reset(void)
{
    USB->BTABLE = 0;
    btable_set(0, PMA_EP0_TX, PMA_EP0_RX);
    btable_set(1, PMA_EP1_TX, PMA_EP1_RX);
    btable_set(2, PMA_EP2_TX, 0);
    ep_setup(0, USB_EP_CONTROL, USB_EP_RX_VALID, USB_EP_TX_NAK);
    USB->DADDR = USB_DADDR_EF;                       /* адрес 0, пока не назначат */
    pending_addr = 0;
    ep0_left = 0;
    ep0_zlp = 0;
    state = USB_STATE_OFF;
    tx_busy = 0;
}

void USB_LP_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    uint16_t istr;

    while ((istr = USB->ISTR) & (USB_ISTR_CTR | USB_ISTR_RESET)) {
        if (istr & USB_ISTR_RESET) {
            USB->ISTR = (uint16_t)~USB_ISTR_RESET;
            usb_reset();
            continue;
        }
        switch (istr & USB_ISTR_EP_ID) {
        case 0:  ep0_event(istr); break;
        case 1:  ep1_event(istr, &woken); break;
        default: ep_clear_ctr_tx(istr & USB_ISTR_EP_ID); break;   /* EP2 нам не нужна */
        }
    }
    portYIELD_FROM_ISR(woken);   /* если разбудили задачу — переключиться на неё сразу */
}

void usb_cdc_init(void)
{
    rx_sb    = xStreamBufferCreate(256, 1);
    tx_sb    = xStreamBufferCreate(512, 1);
    tx_mutex = xSemaphoreCreateMutex();

    /* 0. Генератором 48 МГц пользуется и радио-ядро (для генератора случайных чисел).
       Аппаратный семафор №5 говорит ему: «CLK48 нужен USB, не выключай».
       Захват: читаем регистр RLR — если вернулся наш ID с битом LOCK, семафор наш. */
    RCC->AHB3ENR |= RCC_AHB3ENR_HSEMEN;
    (void)RCC->AHB3ENR;
    while (HSEM->RLR[5] != (HSEM_RLR_LOCK | (HSEM_CPU1_COREID << HSEM_RLR_COREID_Pos))) { }

    /* 1. Генератор 48 МГц для USB: HSI48 + CRS (подстройка по SOF от компьютера) */
    RCC->CRRCR |= RCC_CRRCR_HSI48ON;
    while (!(RCC->CRRCR & RCC_CRRCR_HSI48RDY)) { }
    RCC->CCIPR &= ~RCC_CCIPR_CLK48SEL_Msk;           /* 00 = HSI48 -> USB */
    RCC->APB1ENR1 |= RCC_APB1ENR1_CRSEN | RCC_APB1ENR1_USBEN;
    (void)RCC->APB1ENR1;
    CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;       /* источник синхронизации по умолчанию = USB SOF */

    /* 2. Питание приёмопередатчика USB */
    PWR->CR2 |= PWR_CR2_USV;

    /* 3. PA11/PA12 -> альтернативная функция AF10 (USB) */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    (void)RCC->AHB2ENR;
    GPIOA->OSPEEDR |= (3U << 22) | (3U << 24);
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFFU << 12)) | (10U << 12) | (10U << 16);
    GPIOA->MODER  = (GPIOA->MODER & ~((3U << 22) | (3U << 24))) | (2U << 22) | (2U << 24);

    /* 4. Включаем периферию USB */
    USB->CNTR = USB_CNTR_FRES;                       /* снимаем PDWN, держим сброс */
    for (volatile int i = 0; i < 100; i++) { }       /* tSTARTUP ~1 мкс */
    USB->CNTR = 0;
    USB->ISTR = 0;
    USB->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM;     /* прерывания: передача и сброс шины */

    /* Приоритет 6 — «ниже» configMAX_SYSCALL_INTERRUPT_PRIORITY (5),
       поэтому из этого прерывания можно звать функции FreeRTOS ...FromISR */
    NVIC_SetPriority(USB_LP_IRQn, 6);
    NVIC_EnableIRQ(USB_LP_IRQn);

    /* 5. Подтяжка на D+ — для компьютера это как «вставили кабель» */
    USB->BCDR |= USB_BCDR_DPPU;
}

usb_state_t usb_cdc_state(void) { return state; }

size_t usb_cdc_read(uint8_t *buf, size_t max, TickType_t wait)
{
    return xStreamBufferReceive(rx_sb, buf, max, wait);
}

void usb_cdc_write(const void *data, size_t len)
{
    if (state != USB_STATE_PORT_OPEN) return;        /* никто не слушает — не копим */

    xSemaphoreTake(tx_mutex, portMAX_DELAY);         /* писать могут несколько задач */
    xStreamBufferSend(tx_sb, data, len, pdMS_TO_TICKS(50));
    NVIC_DisableIRQ(USB_LP_IRQn);                    /* чтобы не столкнуться с прерыванием */
    if (!tx_busy) tx_kick(pdFALSE, NULL);
    NVIC_EnableIRQ(USB_LP_IRQn);
    xSemaphoreGive(tx_mutex);
}

void usb_cdc_print(const char *s)
{
    usb_cdc_write(s, strlen(s));
}
