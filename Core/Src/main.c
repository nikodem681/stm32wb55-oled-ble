/*
 * NUCLEO-WB55RG + FreeRTOS + OLED 0.96" 128x64 (SSD1306, I2C) + USB COM-порт
 *
 * Подключение дисплея (разъём Arduino на Nucleo):
 *   GND -> GND,  VCC -> 3V3,  SCL -> D15 (PB8),  SDA -> D14 (PB9)
 * USB: кабель в разъём USB_USER (не ST-LINK!) -> в системе появится COM-порт.
 *
 * Светодиоды:
 *   синий  (PB5) — тускло мигает, значит FreeRTOS работает (яркость: команда "led N");
 *   зелёный(PB0) — дисплей найден и отвечает;
 *   красный(PB1) — дисплей не отвечает по I2C (проверь провода).
 */
#include "stm32wbxx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "i2c.h"
#include "ssd1306.h"
#include "usb_cdc.h"
#include "temp.h"
#include "ble_app.h"
#include "scope.h"
#include <string.h>

#define LED_BLUE  5
#define LED_GREEN 0
#define LED_RED   1

#define LOG_COLS  21   /* столько символов шрифта 5x7 влезает в строку (21 * 6 = 126 пикселей) */
#define LOG_ROWS  6    /* строки текста на экране: страницы 1..6 */

/* Отладочные переменные — их можно прочитать через ST-Link не останавливая МК */
volatile uint32_t g_disp_status = 0xFF;   /* последний i2c_status_t от дисплея */
volatile uint32_t g_disp_addr   = 0;
volatile uint32_t g_frames      = 0;
volatile uint32_t g_presses     = 0;
static volatile int scope_screen;        /* 1 — на дисплее осциллограф, 0 — обычный текст */
volatile int32_t  g_temp_x10    = 0;      /* температура кристалла, десятые °C */

/* Яркость синего светодиода: длительность импульса в мкс из периода 1000 мкс */
static volatile uint32_t blue_duty = 20;     /* 2% — тускло */
static volatile uint8_t  blue_on   = 0;      /* мигание: heartbeat-задача включает/выключает */

/* Текст с компьютера, как в терминале: новая строка снизу, старые уезжают вверх.
   Пишет usb_task, читает display_task — поэтому доступ под мьютексом,
   чтобы не нарисовать строку «наполовину». */
static char log_lines[LOG_ROWS][LOG_COLS + 1] = { "Type text in terminal" };
static int  log_count = 1;
static SemaphoreHandle_t log_mutex;

static void log_push_row(const char *s, size_t n)
{
    if (log_count == LOG_ROWS) {                         /* экран полон — прокрутка вверх */
        memmove(log_lines[0], log_lines[1], sizeof log_lines - sizeof log_lines[0]);
        log_count--;
    }
    memcpy(log_lines[log_count], s, n);
    log_lines[log_count][n] = '\0';
    log_count++;
}

/* Длинная строка переносится кусками по LOG_COLS символов */
static void log_add(const char *s)
{
    xSemaphoreTake(log_mutex, portMAX_DELAY);
    size_t len = strlen(s);
    do {
        size_t n = len > LOG_COLS ? LOG_COLS : len;
        log_push_row(s, n);
        s += n;
        len -= n;
    } while (len);
    xSemaphoreGive(log_mutex);
}

static void log_clear(void)
{
    xSemaphoreTake(log_mutex, portMAX_DELAY);
    log_count = 0;
    xSemaphoreGive(log_mutex);
}

static void led(int pin, int on)
{
    if (on) GPIOB->BSRR = 1U << pin;
    else    GPIOB->BRR  = 1U << pin;
}

/* Заводская подстройка кварца HSE хранится в OTP-памяти, в записи с id = 0
   (ищем последнюю такую запись — их могут дописывать). */
static int otp_hse_tuning(void)
{
    for (uint32_t a = OTP_AREA_END_ADDR + 1 - 8; a >= OTP_AREA_BASE; a -= 8) {
        const uint8_t *e = (const uint8_t *)a;
        if (e[7] == 0) return e[6];           /* [0..5] адрес BT, [6] подстройка HSE, [7] id */
    }
    return -1;
}

static void clock_init(void)
{
    /*
     * Радио требует кварц HSE 32 МГц, им же тактируем и процессор.
     * Плюс кварц LSE 32.768 кГц — по нему радио-ядро отмеряет интервалы в BLE.
     */
    int tune = otp_hse_tuning();
    if (tune >= 0) {
        RCC->HSECR = 0xCAFECAFE;              /* ключ разблокировки регистра HSECR */
        RCC->HSECR = (RCC->HSECR & ~RCC_HSECR_HSETUNE_Msk) | ((uint32_t)tune << RCC_HSECR_HSETUNE_Pos);
    }
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) { }

    /* HSI16 процессору не нужен, но на STM32WB без него не работает запись/стирание Flash.
       А BLE-стек во втором ядре при старте пишет свои данные во Flash. */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }

    /* 32 МГц уже требуют 1 такт ожидания у Flash — ставим ДО переключения */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) | FLASH_ACR_LATENCY_1WS;
    while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_1WS) { }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_1;          /* SW = 10: HSE */
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_1) { }
    SystemCoreClockUpdate();   /* SystemCoreClock = 32000000 */

    /* LSE живёт в «домене резервного питания» — сначала снимаем защиту записи */
    PWR->CR1 |= PWR_CR1_DBP;
    RCC->BDCR &= ~RCC_BDCR_LSEDRV_Msk;        /* слабый драйв кварца — как в примерах ST */
    RCC->BDCR |= RCC_BDCR_LSEON;
    while (!(RCC->BDCR & RCC_BDCR_LSERDY)) { }   /* LSE может запускаться до ~1 с */
    RCC->CSR = (RCC->CSR & ~RCC_CSR_RFWKPSEL_Msk) | (1U << RCC_CSR_RFWKPSEL_Pos);   /* радио от LSE */
}

static void gpio_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIODEN;
    (void)RCC->AHB2ENR;

    /* Светодиоды PB0, PB1, PB5 — выходы */
    GPIOB->MODER = (GPIOB->MODER & ~((3U << 0) | (3U << 2) | (3U << 10)))
                 | (1U << 0) | (1U << 2) | (1U << 10);

    /* Кнопка SW1 = PC4: вход с подтяжкой вверх, нажатие даёт 0 */
    GPIOC->MODER &= ~(3U << 8);
    GPIOC->PUPDR = (GPIOC->PUPDR & ~(3U << 8)) | (1U << 8);

    /* Кнопки SW2 = PD0, SW3 = PD1: так же, вход с подтяжкой вверх */
    GPIOD->MODER &= ~((3U << 0) | (3U << 2));
    GPIOD->PUPDR = (GPIOD->PUPDR & ~((3U << 0) | (3U << 2))) | (1U << 0) | (1U << 2);
}

/*
 * «Программный ШИМ» для синего светодиода на TIM17
 * (PB5 не подключён ни к одному каналу таймера, поэтому переключаем пин в прерывании).
 * Таймер считает микросекунды от 0 до 999 (период 1 мс = 1 кГц — глаз не видит мерцания).
 *   - переполнение (update) -> включаем светодиод;
 *   - совпадение с CCR1      -> выключаем.
 * Чем меньше CCR1, тем короче вспышка и тем тусклее светодиод.
 * (TIM2 занят осциллографом — он запускает измерения АЦП.)
 */
static void led_pwm_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
    (void)RCC->APB2ENR;
    TIM17->PSC  = SystemCoreClock / 1000000 - 1;  /* делим до 1 МГц -> 1 тик = 1 мкс */
    TIM17->ARR  = 1000 - 1;       /* период 1000 мкс */
    TIM17->CCR1 = blue_duty;
    TIM17->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;
    NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 7);
    NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
    TIM17->CR1  = TIM_CR1_CEN;
}

/* Вектор общий с TIM1, но прерывания TIM1 мы не включаем — здесь только TIM17 */
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
    uint32_t sr = TIM17->SR;
    TIM17->SR = ~sr;                              /* сбрасываем обработанные флаги */
    if ((sr & TIM_SR_UIF) && blue_on && blue_duty) led(LED_BLUE, 1);
    if ((sr & TIM_SR_CC1IF) && blue_duty < 1000)   led(LED_BLUE, 0);
}

/* Простое преобразование числа в строку (без printf) */
static char *utoa_dec(uint32_t v, char *end)
{
    *--end = '\0';
    do { *--end = (char)('0' + v % 10); v /= 10; } while (v);
    return end;
}

/* 314 -> "31.4" */
static char *fmt_x10(int32_t v, char *end)
{
    int neg = v < 0;
    uint32_t a = neg ? (uint32_t)-v : (uint32_t)v;
    *--end = '\0';
    *--end = (char)('0' + a % 10);
    *--end = '.';
    a /= 10;
    do { *--end = (char)('0' + a % 10); a /= 10; } while (a);
    if (neg) *--end = '-';
    return end;
}

static void temp_task(void *arg)
{
    (void)arg;
    temp_init();
    for (;;) {
        g_temp_x10 = temp_read_x10();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void heartbeat_task(void *arg)
{
    (void)arg;
    for (;;) {
        blue_on = !blue_on;
        if (!blue_on) led(LED_BLUE, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* Сообщение и в USB, и в Bluetooth */
static void say(const char *a, const char *b, const char *c)
{
    usb_cdc_print(a); usb_cdc_print(b); usb_cdc_print(c);
    ble_print(a);     ble_print(b);     ble_print(c);
}

/*
 * Кнопки опрашиваем раз в 20 мс (заодно это антидребезг). Нажатие = переход 1 -> 0.
 *   SW1: на осциллографе — скважность ШИМ +10%, на текстовом экране — счётчик нажатий;
 *   SW2: развёртка (время на клетку);
 *   SW3: коротко — RUN/HOLD, держать 1 с — сменить экран.
 */
static void button_task(void *arg)
{
    (void)arg;
    char buf[12];
    int prev1 = 1, prev2 = 1, prev3 = 1;
    int held3 = 0;                                /* сколько опросов SW3 удерживается */

    for (;;) {
        int sw1 = (GPIOC->IDR >> 4) & 1;
        int sw2 = (GPIOD->IDR >> 0) & 1;
        int sw3 = (GPIOD->IDR >> 1) & 1;

        if (prev1 && !sw1) {
            if (scope_screen) {
                uint32_t d = scope_duty() + 10;
                if (d > 90) d = 10;
                scope_set_duty(d);
                say("PWM duty ", utoa_dec(d, buf + sizeof buf), "%\r\n");
            } else {
                g_presses++;
                say("SW1 pressed, total: ", utoa_dec(g_presses, buf + sizeof buf), "\r\n");
            }
        }

        if (prev2 && !sw2 && scope_screen) {
            scope_next_timebase();
            say("Timebase ", scope_timebase_name(), "/div\r\n");
        }

        if (!sw3) {
            if (++held3 == 50) {                  /* 50 * 20 мс = 1 с */
                scope_screen = !scope_screen;
                say("Screen: ", scope_screen ? "oscilloscope" : "text", "\r\n");
            }
        } else {
            if (!prev3 && held3 < 50 && scope_screen) {   /* отпустили быстро */
                scope_toggle_hold();
                say("Scope: ", scope_is_hold() ? "HOLD" : "RUN", "\r\n");
            }
            held3 = 0;
        }

        prev1 = sw1; prev2 = sw2; prev3 = sw3;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static char *hex(uint32_t v, char *end)
{
    *--end = '\0';
    do { *--end = "0123456789ABCDEF"[v & 0xF]; v >>= 4; } while (v);
    *--end = 'x';
    *--end = '0';
    return end;
}

static void print_ble_status(void (*out)(const char *))
{
    static const char *const names[] = { "starting", "advertising", "connected", "ERROR" };
    char buf[12];
    uint32_t v = g_ble_fw_version;
    out("BLE: ");
    out(names[ble_app_state()]);
    out(", stack ");
    out(utoa_dec(v >> 16, buf + sizeof buf));        out(".");
    out(utoa_dec((v >> 8) & 0xFF, buf + sizeof buf)); out(".");
    out(utoa_dec(v & 0xFF, buf + sizeof buf));
    if (ble_app_state() == BLE_STATE_ERROR) { out(", error "); out(hex(g_ble_error, buf + sizeof buf)); }
    out("\r\n");
}

/* Выполнить команду. Одна и та же функция работает и для USB, и для Bluetooth —
   отличается только out(): куда печатать ответ. */
void app_handle_line(const char *line, void (*out)(const char *))
{
    char buf[12];

    if (strcmp(line, "help") == 0) {
        out("Commands:\r\n"
            "  led N   - blue LED brightness 0..100 %\r\n"
            "  temp    - chip temperature\r\n"
            "  clear   - clear the OLED text\r\n"
            "  ble     - Bluetooth status\r\n"
            "  scope   - switch OLED: text / oscilloscope\r\n"
            "  pwm N   - test PWM duty 0..100 % (pin D3)\r\n"
            "  pwmf N  - test PWM frequency, Hz\r\n"
            "  help    - this text\r\n"
            "  other   - shown on the OLED\r\n");
    } else if (strncmp(line, "led ", 4) == 0) {
        uint32_t p = 0;
        for (const char *c = line + 4; *c >= '0' && *c <= '9'; c++) p = p * 10 + (uint32_t)(*c - '0');
        if (p > 100) p = 100;
        blue_duty = p * 10;
        TIM17->CCR1 = blue_duty;
        out("LED brightness = ");
        out(utoa_dec(p, buf + sizeof buf));
        out("%\r\n");
    } else if (strcmp(line, "temp") == 0) {
        out("Chip temperature: ");
        out(fmt_x10(g_temp_x10, buf + sizeof buf));
        out(" C\r\n");
    } else if (strcmp(line, "clear") == 0) {
        log_clear();
        out("OLED cleared\r\n");
    } else if (strcmp(line, "scope") == 0) {
        scope_screen = !scope_screen;
        out(scope_screen ? "Screen: oscilloscope\r\n" : "Screen: text\r\n");
    } else if (strncmp(line, "pwmf ", 5) == 0) {
        uint32_t f = 0;
        for (const char *c = line + 5; *c >= '0' && *c <= '9'; c++) f = f * 10 + (uint32_t)(*c - '0');
        scope_set_freq(f);
        out("PWM frequency = ");
        out(utoa_dec(scope_freq(), buf + sizeof buf));
        out(" Hz\r\n");
    } else if (strncmp(line, "pwm ", 4) == 0) {
        uint32_t d = 0;
        for (const char *c = line + 4; *c >= '0' && *c <= '9'; c++) d = d * 10 + (uint32_t)(*c - '0');
        scope_set_duty(d);
        out("PWM duty = ");
        out(utoa_dec(scope_duty(), buf + sizeof buf));
        out("%\r\n");
    } else if (strcmp(line, "ble") == 0) {
        print_ble_status(out);
    } else {
        log_add(line);
        out("Shown on OLED\r\n");
    }
}

/* Принимает символы с компьютера, собирает строку до Enter и выполняет */
static void usb_task(void *arg)
{
    (void)arg;
    char line[64];
    size_t len = 0;
    usb_state_t prev = USB_STATE_OFF;

    usb_cdc_init();

    for (;;) {
        usb_state_t st = usb_cdc_state();
        if (st == USB_STATE_PORT_OPEN && prev != USB_STATE_PORT_OPEN) {
            vTaskDelay(pdMS_TO_TICKS(50));        /* дать терминалу «проснуться» */
            usb_cdc_print("\r\nHello from NUCLEO-WB55 + FreeRTOS! Type 'help'.\r\n> ");
        }
        prev = st;

        uint8_t rx[64];
        size_t n = usb_cdc_read(rx, sizeof rx, pdMS_TO_TICKS(100));
        for (size_t i = 0; i < n; i++) {
            char c = (char)rx[i];
            if (c == '\r' || c == '\n') {
                if (len == 0) continue;
                line[len] = '\0';
                usb_cdc_print("\r\n");
                app_handle_line(line, usb_cdc_print);
                usb_cdc_print("> ");
                len = 0;
            } else if ((c == '\b' || c == 0x7F) && len > 0) {   /* Backspace */
                len--;
                usb_cdc_print("\b \b");
            } else if (c >= 0x20 && c < 0x7F && len < sizeof line - 1) {
                line[len++] = c;
                usb_cdc_write(&c, 1);                           /* эхо — видно, что печатаешь */
            }
        }
    }
}

/* Значок USB в заголовке: пусто — нет связи, кружок — опознаны, точка — порт открыт */
static void draw_usb_icon(int x, usb_state_t st)
{
    static const uint8_t ring[5] = { 0x0E, 0x11, 0x11, 0x11, 0x0E };
    static const uint8_t dot[5]  = { 0x0E, 0x1F, 0x1F, 0x1F, 0x0E };
    const uint8_t *g = st == USB_STATE_PORT_OPEN ? dot : st == USB_STATE_CONFIGURED ? ring : NULL;
    for (int c = 0; g && c < 5; c++)
        for (int r = 0; r < 5; r++)
            ssd1306_pixel(x + c, 1 + r, (g[c] >> r) & 1);
}

/* Значок Bluetooth: мигает — ждём телефон, горит — подключён, крестик — ошибка */
static void draw_ble_icon(int x, ble_state_t st, uint32_t frame)
{
    static const uint8_t rune[5]  = { 0x22, 0x14, 0x7F, 0x2A, 0x14 };
    static const uint8_t cross[5] = { 0x41, 0x22, 0x1C, 0x22, 0x41 };
    const uint8_t *g = NULL;
    if (st == BLE_STATE_CONNECTED) g = rune;
    else if (st == BLE_STATE_ADVERTISING && (frame / 5) % 2) g = rune;   /* 0.5 с вкл / 0.5 с выкл */
    else if (st == BLE_STATE_ERROR) g = cross;
    for (int c = 0; g && c < 5; c++)
        for (int r = 0; r < 7; r++)
            ssd1306_pixel(x + c, r, (g[c] >> r) & 1);
}

/* Заголовок справа: "31.4°C". В шрифте нет значка градуса — рисуем кружок 3x3 сами. */
static void draw_temp(int32_t t10)
{
    char buf[12];
    const char *s = fmt_x10(t10, buf + sizeof buf);
    int x = SSD1306_WIDTH - (int)strlen(s) * 6 - 4 - 6;    /* число + градус + "C" */
    ssd1306_text(x, 0, s);
    x += (int)strlen(s) * 6;
    ssd1306_pixel(x, 0, 1); ssd1306_pixel(x + 1, 0, 1); ssd1306_pixel(x + 2, 0, 1);
    ssd1306_pixel(x, 1, 1);                         ssd1306_pixel(x + 2, 1, 1);
    ssd1306_pixel(x, 2, 1); ssd1306_pixel(x + 1, 2, 1); ssd1306_pixel(x + 2, 2, 1);
    ssd1306_text(x + 4, 0, "C");
}

static void display_task(void *arg)
{
    (void)arg;
    char text[LOG_ROWS][LOG_COLS + 1];
    int rows;

    i2c1_init();

    for (;;) {
        /* Ждём, пока дисплей ответит. Если нет — горит красный, пробуем раз в секунду. */
        i2c_status_t st;
        while ((st = ssd1306_init()) != I2C_OK) {
            g_disp_status = st;
            led(LED_GREEN, 0);
            led(LED_RED, 1);
            i2c1_init();                   /* на всякий случай перезапускаем шину */
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        g_disp_status = I2C_OK;
        g_disp_addr = ssd1306_addr;
        led(LED_RED, 0);
        led(LED_GREEN, 1);

        TickType_t last = xTaskGetTickCount();
        while (st == I2C_OK) {
            /* Копируем текст под мьютексом и сразу отпускаем — рисуем уже копию */
            xSemaphoreTake(log_mutex, portMAX_DELAY);
            memcpy(text, log_lines, sizeof text);
            rows = log_count;
            xSemaphoreGive(log_mutex);

            ssd1306_clear();

            if (scope_screen) {
                scope_frame();                    /* снять сигнал и нарисовать осциллограмму */
            } else {
                /* Заголовок + линия под ним */
                ssd1306_text(0, 0, "Hello, STM32!");
                draw_usb_icon(80, usb_cdc_state());
                draw_ble_icon(87, ble_app_state(), g_frames);
                draw_temp(g_temp_x10);
                for (int x = 0; x < SSD1306_WIDTH; x++) ssd1306_pixel(x, 7, 1);

                /* Текст с компьютера: страницы 1..6 */
                for (int i = 0; i < rows; i++) ssd1306_text(0, 1 + i, text[i]);

                /* Бегающая точка внизу — видно, что кадры обновляются */
                ssd1306_pixel(2 + (int)(g_frames % 124), 61, 1);
            }

            st = ssd1306_update();
            g_disp_status = st;
            g_frames++;
            vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
        }
        /* Ошибка связи посреди работы (например, отошёл провод) — начинаем заново */
    }
}

int main(void)
{
    /* После включения/прошивки у STM32WB часто стоит флаг OPTVERR («ошибка проверки
       option bytes»). Пока он стоит, Flash нельзя записывать — а BLE-стеку во втором ядре
       при старте нужно писать в свою область Flash, и без сброса он зависает навсегда. */
    FLASH->SR = FLASH_SR_OPTVERR;

    clock_init();
    gpio_init();
    led_pwm_init();
    adc_shared_init();
    scope_init();

    log_mutex = xSemaphoreCreateMutex();

    xTaskCreate(heartbeat_task, "heartbeat", 128, NULL, 1, NULL);
    xTaskCreate(button_task,    "button",    192, NULL, 2, NULL);
    xTaskCreate(usb_task,       "usb",       384, NULL, 2, NULL);
    xTaskCreate(display_task,   "display",   384, NULL, 1, NULL);
    xTaskCreate(temp_task,      "temp",      192, NULL, 1, NULL);

    ble_app_start();

    vTaskStartScheduler();   /* дальше управление у FreeRTOS, сюда не вернёмся */
    for (;;) { }
}

/* Вызываются FreeRTOS при серьёзных ошибках: зажигаем все светодиоды и стоим. */
void vApplicationStackOverflowHook(TaskHandle_t t, char *name)
{
    (void)t; (void)name;
    __disable_irq();
    GPIOB->BSRR = (1U << LED_RED) | (1U << LED_GREEN) | (1U << LED_BLUE);
    for (;;) { }
}

void vApplicationMallocFailedHook(void)
{
    __disable_irq();
    GPIOB->BSRR = (1U << LED_RED) | (1U << LED_GREEN) | (1U << LED_BLUE);
    for (;;) { }
}
