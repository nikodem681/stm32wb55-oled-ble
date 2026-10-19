/*
 * I2C1 (PB8 = SCL, PB9 = SDA), 400 кГц, передача через DMA.
 *
 * Раньше процессор сам клал каждый байт в TXDR и крутился в цикле, ожидая, пока
 * шина его передаст, — задача дисплея съедала ~90% процессора просто на ожидание.
 * Теперь:
 *   1. копируем посылку в буфер и отдаём её DMA;
 *   2. I2C сам просит у DMA следующий байт (сигнал TXIS -> запрос DMA);
 *   3. задача засыпает (ulTaskNotifyTake), процессор свободен для других;
 *   4. в конце передачи I2C даёт прерывание STOP (или NACK/ошибка) -> будим задачу.
 */
#include "i2c.h"
#include "stm32wbxx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define SCL_PIN 8
#define SDA_PIN 9
#define DMAMUX_I2C1_TX 11       /* номер запроса I2C1_TX в DMAMUX */
#define XFER_TIMEOUT_MS 20      /* 255 байт на 400 кГц — это ~6 мс, берём с запасом */

static uint8_t txbuf[256];                      /* посылка целиком: префикс + данные */
static volatile i2c_status_t xfer_status;
static TaskHandle_t waiting_task;

static void short_delay(void)
{
    for (volatile uint32_t i = 0; i < SystemCoreClock / 400000; i++) { }   /* ~5 мкс при любой частоте */
}

static void pin_mode(uint32_t pin, uint32_t mode)
{
    GPIOB->MODER = (GPIOB->MODER & ~(3U << (pin * 2))) | (mode << (pin * 2));
}

/*
 * Восстановление «зависшей» шины.
 * Если МК перезагрузился посреди передачи, ведомое устройство может
 * держать SDA в нуле, ожидая продолжения тактов. Лечится так: вручную
 * подаём до 9 импульсов на SCL, пока SDA не отпустят, затем формируем STOP.
 */
static void bus_recover(void)
{
    GPIOB->BSRR = (1U << SCL_PIN) | (1U << SDA_PIN);    /* обе в 1 (open-drain = отпущены) */
    pin_mode(SCL_PIN, 1);                               /* обычный выход */
    pin_mode(SDA_PIN, 1);
    short_delay();

    for (int i = 0; i < 9 && !(GPIOB->IDR & (1U << SDA_PIN)); i++) {
        GPIOB->BRR  = 1U << SCL_PIN; short_delay();
        GPIOB->BSRR = 1U << SCL_PIN; short_delay();
    }
    /* STOP: SDA 0 -> 1 при SCL = 1 */
    GPIOB->BRR  = 1U << SCL_PIN; short_delay();
    GPIOB->BRR  = 1U << SDA_PIN; short_delay();
    GPIOB->BSRR = 1U << SCL_PIN; short_delay();
    GPIOB->BSRR = 1U << SDA_PIN; short_delay();
}

void i2c1_init(void)
{
    RCC->AHB1ENR  |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;     /* тактирование порта B */
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;     /* тактирование I2C1 */
    (void)RCC->APB1ENR1;

    NVIC_DisableIRQ(I2C1_EV_IRQn);
    NVIC_DisableIRQ(I2C1_ER_IRQn);
    DMA1_Channel2->CCR = 0;

    /* PB8/PB9: open-drain, высокая скорость, внутренняя подтяжка вверх */
    GPIOB->OTYPER  |= (1U << SCL_PIN) | (1U << SDA_PIN);
    GPIOB->OSPEEDR |= (3U << (SCL_PIN * 2)) | (3U << (SDA_PIN * 2));
    GPIOB->PUPDR = (GPIOB->PUPDR & ~((3U << (SCL_PIN * 2)) | (3U << (SDA_PIN * 2))))
                 | (1U << (SCL_PIN * 2)) | (1U << (SDA_PIN * 2));

    bus_recover();

    /* Альтернативная функция AF4 = I2C1 */
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xFFU)) | (4U << 0) | (4U << 4);
    pin_mode(SCL_PIN, 2);
    pin_mode(SDA_PIN, 2);

    /* Сброс периферии, чтобы стереть состояние, оставшееся от старой прошивки */
    RCC->APB1RSTR1 |=  RCC_APB1RSTR1_I2C1RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C1RST;

    I2C1->CR1 = 0;
    /* TIMINGR для 400 кГц (Fast mode, по таблице из Reference Manual RM0434):
       делитель PRESC даёт такт 8 МГц (125 нс), дальше в этих тактах:
       SCLDEL=3, SDADEL=2, SCLH=3 (0.5 мкс), SCLL=9 (1.25 мкс) */
    uint32_t presc = SystemCoreClock / 8000000 - 1;     /* 64 МГц -> 7 */
    I2C1->TIMINGR = (presc << 28) | 0x00320309;

    /* Прерывания: конец передачи (STOP), «устройство не ответило» (NACK), ошибки шины.
       Приоритет 6 — можно будить задачи функциями FreeRTOS ...FromISR. */
    I2C1->CR1 = I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_ERRIE | I2C_CR1_TXDMAEN;
    NVIC_SetPriority(I2C1_EV_IRQn, 6);
    NVIC_SetPriority(I2C1_ER_IRQn, 6);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
    I2C1->CR1 |= I2C_CR1_PE;                  /* включаем */
}

/* ---- Прерывания: передача закончилась (или сломалась) -> будим задачу ---- */

static void wake(i2c_status_t st)
{
    xfer_status = st;
    BaseType_t woken = pdFALSE;
    if (waiting_task) vTaskNotifyGiveFromISR(waiting_task, &woken);
    portYIELD_FROM_ISR(woken);
}

void I2C1_EV_IRQHandler(void)
{
    uint32_t isr = I2C1->ISR;
    if (isr & I2C_ISR_NACKF) {
        I2C1->ICR = I2C_ICR_NACKCF;           /* устройство не ответило; STOP пошлётся сам (AUTOEND) */
        xfer_status = I2C_ERR_NACK;
    }
    if (isr & I2C_ISR_STOPF) {
        I2C1->ICR = I2C_ICR_STOPCF;
        wake(xfer_status);                    /* OK, если по дороге не было NACK */
    }
}

void I2C1_ER_IRQHandler(void)
{
    I2C1->ICR = I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
    wake(I2C_ERR_BUSY);
}

/* ---- Одна транзакция записи: адрес + n байт из txbuf ---- */

static i2c_status_t xfer(uint8_t addr7, uint16_t n)
{
    if (I2C1->ISR & I2C_ISR_BUSY) return I2C_ERR_BUSY;

    /* DMA1 канал 2: из txbuf в регистр I2C1->TXDR, по байту, адрес в памяти растёт */
    DMA1_Channel2->CCR = 0;
    DMA1->IFCR = DMA_IFCR_CGIF2;
    if (n) {
        DMAMUX1_Channel1->CCR = DMAMUX_I2C1_TX;   /* канал DMAMUX 1 обслуживает канал DMA 2 */
        DMA1_Channel2->CPAR  = (uint32_t)&I2C1->TXDR;
        DMA1_Channel2->CMAR  = (uint32_t)txbuf;
        DMA1_Channel2->CNDTR = n;
        DMA1_Channel2->CCR   = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_EN;   /* DIR: память -> периферия */
    }

    xfer_status   = I2C_OK;
    waiting_task  = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTake(pdTRUE, 0);              /* сбросить старое уведомление, если было */

    I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
    I2C1->CR2 = ((uint32_t)addr7 << 1)
              | ((uint32_t)n << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_AUTOEND               /* STOP автоматически после n байт */
              | I2C_CR2_START;                /* RD_WRN = 0 -> запись */

    /* Спим, пока прерывание не разбудит. Процессор в это время работает на другие задачи. */
    i2c_status_t st;
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(XFER_TIMEOUT_MS)))
        st = xfer_status;
    else
        st = I2C_ERR_TIMEOUT;                 /* так и не дождались STOP — шина повисла */

    waiting_task = NULL;
    DMA1_Channel2->CCR = 0;
    return st;
}

i2c_status_t i2c1_probe(uint8_t addr7)
{
    return xfer(addr7, 0);                    /* только адрес, без данных: ответит или нет */
}

i2c_status_t i2c1_write2(uint8_t addr7, uint8_t prefix, const uint8_t *data, uint16_t len)
{
    if (len > 254) return I2C_ERR_TIMEOUT;    /* NBYTES — 8 бит (до 255 с префиксом); нам больше и не надо */
    txbuf[0] = prefix;                        /* DMA шлёт подряд — собираем посылку в один буфер */
    memcpy(txbuf + 1, data, len);
    return xfer(addr7, len + 1);
}

i2c_status_t i2c1_write(uint8_t addr7, const uint8_t *data, uint16_t len)
{
    if (len == 0) return i2c1_probe(addr7);
    return i2c1_write2(addr7, data[0], data + 1, len - 1);
}
