/*
 * Осциллограф.
 *
 * Как снимается сигнал:
 *   TIM2 тикает с периодом «одна точка» и на каждом тике (событие TRGO) запускает
 *   одно преобразование АЦП. Результат DMA сам кладёт в массив buf[] — процессор в
 *   это время свободен. Когда массив заполнен (512 точек), DMA ставит флаг TCIF.
 *
 * Как картинка «стоит на месте»:
 *   в записи ищем триггер — момент, когда сигнал пересёк середину снизу вверх.
 *   Рисуем 128 точек, начиная чуть раньше триггера. Так каждый кадр начинается
 *   в одной и той же фазе сигнала.
 *
 * Тестовый сигнал: TIM1, канал 3 -> D3 (PA10), обычный аппаратный ШИМ.
 */
#include "scope.h"
#include "temp.h"
#include "ssd1306.h"
#include "stm32wbxx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define N_SAMPLES   512
#define PRE_TRIG    16          /* сколько точек показать ДО триггера (одна клетка) */
#define ADC_CH      6           /* A2 = PA1 = ADC1_IN6 */
#define DMAMUX_ADC1 5           /* номер запроса АЦП в DMAMUX */

/* Плот: y = 9 (3.3 В) ... 55 (0 В), 128 точек по x, клетка = 16 точек */
#define Y_TOP       9
#define Y_BOT       55

static uint16_t buf[N_SAMPLES];
static int      have_data;

static uint32_t pwm_duty = 30;
static uint32_t pwm_hz   = 1000;

static const uint16_t tb_us[]   = { 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000 };
static const char    *tb_name[] = { "20us", "50us", "100us", "200us", "500us", "1ms", "2ms", "5ms", "10ms" };
static volatile uint32_t tb = 4;            /* 500 мкс на клетку */
static volatile int hold;

/* ------------------------------------------------------------------ */
/*                     Тестовый ШИМ на TIM1 / D3                       */
/* ------------------------------------------------------------------ */

static void pwm_apply(void)
{
    /* Частота таймера / (PSC+1) / (ARR+1) = частота ШИМ.
       TIM1 16-битный (ARR <= 65535), поэтому для низких частот растёт делитель PSC. */
    uint32_t cycles = SystemCoreClock / pwm_hz;
    uint32_t psc = cycles / 65536;
    uint32_t arr = cycles / (psc + 1) - 1;
    TIM1->PSC  = psc;
    TIM1->ARR  = arr;
    TIM1->CCR3 = (arr + 1) * pwm_duty / 100;  /* сколько тиков из периода выход в «1» */
    TIM1->EGR  = TIM_EGR_UG;                  /* применить сразу, не дожидаясь конца периода */
}

void scope_set_duty(uint32_t p) { pwm_duty = p > 100 ? 100 : p; pwm_apply(); }
uint32_t scope_duty(void)       { return pwm_duty; }
void scope_set_freq(uint32_t hz)
{
    if (hz < 10) hz = 10;
    if (hz > 100000) hz = 100000;
    pwm_hz = hz;
    pwm_apply();
}
uint32_t scope_freq(void)       { return pwm_hz; }

void        scope_next_timebase(void) { tb = (tb + 1) % (sizeof tb_us / sizeof tb_us[0]); }
const char *scope_timebase_name(void) { return tb_name[tb]; }
void        scope_toggle_hold(void)   { hold = !hold; }
int         scope_is_hold(void)       { return hold; }

void scope_init(void)
{
    RCC->AHB1ENR  |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB2ENR  |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    (void)RCC->APB1ENR1;

    /* PA1 — аналоговый вход (MODER = 11) */
    GPIOA->MODER |= 3U << (1 * 2);

    /* PA10 — альтернативная функция AF1 = TIM1_CH3 */
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFU << 8)) | (1U << 8);
    GPIOA->MODER  = (GPIOA->MODER & ~(3U << 20)) | (2U << 20);

    /* TIM1 CH3: режим PWM1 (выход «1», пока счётчик < CCR3) */
    TIM1->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM1->CCER  = TIM_CCER_CC3E;
    TIM1->BDTR  = TIM_BDTR_MOE;      /* у «продвинутого» TIM1 выходы надо отдельно разрешить */
    pwm_apply();
    TIM1->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;

    /* TIM2: событие обновления (переполнение) выдаётся наружу как TRGO -> запуск АЦП */
    TIM2->CR2 = TIM_CR2_MMS_1;       /* MMS = 010: TRGO = update */
}

/* ------------------------------------------------------------------ */
/*                            Захват                                   */
/* ------------------------------------------------------------------ */

static int capture(void)
{
    if (!adc_is_ready()) return 0;
    adc_lock();

    /* АЦП: канал 6, запуск по TIM2_TRGO (EXTSEL = 1011) по фронту, результаты — в DMA */
    ADC1->CFGR = (ADC1->CFGR & ~(ADC_CFGR_EXTSEL_Msk | ADC_CFGR_EXTEN_Msk | ADC_CFGR_CONT |
                                 ADC_CFGR_DMACFG | ADC_CFGR_RES_Msk | ADC_CFGR_ALIGN))
               | (0xBU << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_EXTEN_0 | ADC_CFGR_DMAEN | ADC_CFGR_OVRMOD;
    /* Время выборки 12.5 такта: 12.5 + 12.5 на преобразование = 25 тактов = 0.8 мкс при 32 МГц */
    ADC1->SMPR1 = (ADC1->SMPR1 & ~(7U << ADC_SMPR1_SMP6_Pos)) | (2U << ADC_SMPR1_SMP6_Pos);
    ADC1->SQR1  = ADC_CH << ADC_SQR1_SQ1_Pos;

    /* DMA1 канал 1: из регистра ADC1->DR в buf[], по 16 бит, адрес в памяти растёт */
    DMA1_Channel1->CCR   = 0;
    DMA1->IFCR           = DMA_IFCR_CGIF1;
    DMAMUX1_Channel0->CCR = DMAMUX_ADC1;   /* канал DMAMUX 0 обслуживает канал DMA 1 */
    DMA1_Channel1->CPAR  = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR  = (uint32_t)buf;
    DMA1_Channel1->CNDTR = N_SAMPLES;
    DMA1_Channel1->CCR   = DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_EN;

    /* TIM2: период = время одной точки = (время клетки) / 16 */
    uint32_t ticks = SystemCoreClock / 1000000 * tb_us[tb] / 16;
    TIM2->CR1 = 0;
    TIM2->PSC = 0;
    TIM2->ARR = ticks - 1;
    TIM2->CNT = 0;

    ADC1->ISR = ADC_ISR_OVR | ADC_ISR_EOC | ADC_ISR_EOS;
    ADC1->CR |= ADC_CR_ADSTART;            /* АЦП готов и ждёт импульсов от TIM2 */
    TIM2->CR1 = TIM_CR1_CEN;

    /* Ждём заполнения буфера (максимум 512 * 625 мкс = 320 мс) */
    TickType_t limit = pdMS_TO_TICKS((uint32_t)N_SAMPLES * tb_us[tb] / 16 / 1000 + 50);
    TickType_t t0 = xTaskGetTickCount();
    int ok = 1;
    while (!(DMA1->ISR & DMA_ISR_TCIF1)) {
        if (xTaskGetTickCount() - t0 > limit) { ok = 0; break; }
        vTaskDelay(1);
    }

    /* Всё останавливаем и отдаём АЦП датчику температуры */
    TIM2->CR1 = 0;
    ADC1->CR |= ADC_CR_ADSTP;
    while (ADC1->CR & ADC_CR_ADSTART) { }
    DMA1_Channel1->CCR = 0;
    DMA1->IFCR = DMA_IFCR_CGIF1;
    ADC1->CFGR &= ~(ADC_CFGR_EXTEN_Msk | ADC_CFGR_DMAEN);

    adc_unlock();
    return ok;
}

/* ------------------------------------------------------------------ */
/*                    Анализ: триггер, частота, скважность             */
/* ------------------------------------------------------------------ */

typedef struct {
    int      start;        /* с какой точки рисовать */
    int      triggered;
    float    freq_hz;      /* 0 — не удалось измерить */
    uint32_t duty_x10;     /* скважность в десятых процента */
} meas_t;

static meas_t analyze(void)
{
    meas_t m = { 0, 0, 0, 0 };
    uint16_t lo_v = 4095, hi_v = 0;
    for (int i = 0; i < N_SAMPLES; i++) {
        if (buf[i] < lo_v) lo_v = buf[i];
        if (buf[i] > hi_v) hi_v = buf[i];
    }
    if (hi_v - lo_v < 80) return m;        /* почти прямая линия (< 65 мВ) — триггерить нечем */

    /* Пороги с гистерезисом: шум возле середины не даёт ложных срабатываний */
    int mid = (lo_v + hi_v) / 2, hyst = (hi_v - lo_v) / 10;
    int below = 0, first = -1, last = -1, n = 0;
    for (int i = 0; i < N_SAMPLES; i++) {
        if (buf[i] < mid - hyst) below = 1;
        else if (below && buf[i] > mid + hyst) {   /* пересечение снизу вверх */
            below = 0;
            if (first < 0) first = i;
            last = i;
            n++;
            if (!m.triggered && i >= PRE_TRIG && i - PRE_TRIG + 128 <= N_SAMPLES) {
                m.start = i - PRE_TRIG;
                m.triggered = 1;
            }
        }
    }

    if (n >= 2) {
        /* Период = среднее расстояние между фронтами; скважность — доля точек «выше середины» */
        float period_samples = (float)(last - first) / (float)(n - 1);
        float ts_us = (float)tb_us[tb] / 16.0f;
        m.freq_hz = 1e6f / (period_samples * ts_us);
        uint32_t high = 0;
        for (int i = first; i < last; i++) if (buf[i] >= mid) high++;
        m.duty_x10 = high * 1000 / (uint32_t)(last - first);
    }
    return m;
}

/* ------------------------------------------------------------------ */
/*                              Рисование                              */
/* ------------------------------------------------------------------ */

static char *put_u(char *p, uint32_t v)
{
    char tmp[11];
    int n = 0;
    do { tmp[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (n) *p++ = tmp[--n];
    return p;
}

static void put_freq(char *p, float f)
{
    if (f <= 0) { strcpy(p, "--"); return; }
    if (f >= 1000.0f) {                    /* 1.234kHz */
        uint32_t v = (uint32_t)(f + 0.5f);
        p = put_u(p, v / 1000);
        *p++ = '.';
        uint32_t frac = v % 1000;
        *p++ = (char)('0' + frac / 100); *p++ = (char)('0' + frac / 10 % 10); *p++ = (char)('0' + frac % 10);
        strcpy(p, "kHz");
    } else {                               /* 123.4Hz */
        uint32_t v = (uint32_t)(f * 10.0f + 0.5f);
        p = put_u(p, v / 10);
        *p++ = '.';
        *p++ = (char)('0' + v % 10);
        strcpy(p, "Hz");
    }
}

static int v2y(uint16_t v) { return Y_BOT - (int)v * (Y_BOT - Y_TOP) / 4095; }

void scope_frame(void)
{
    if (!hold || !have_data) {
        if (capture()) have_data = 1;
    }
    meas_t m = analyze();

    /* Заголовок: развёртка, скважность ШИМ, режим */
    char line[24], *p = line;
    strcpy(p, tb_name[tb]); p += strlen(p);
    strcpy(p, "/d PWM"); p += strlen(p);
    p = put_u(p, pwm_duty);
    strcpy(p, hold ? "% HOLD" : "% RUN");
    ssd1306_text(0, 0, line);

    /* Сетка пунктиром: 8 клеток по горизонтали, уровни 0 / 1.65 / 3.3 В */
    for (int x = 0; x < 128; x += 4) {
        ssd1306_pixel(x, Y_TOP, 1);
        ssd1306_pixel(x, (Y_TOP + Y_BOT) / 2, 1);
        ssd1306_pixel(x, Y_BOT, 1);
    }
    for (int x = 0; x < 128; x += 16)
        for (int y = Y_TOP; y <= Y_BOT; y += 3) ssd1306_pixel(x, y, 1);

    /* Сам сигнал: соседние точки соединяем вертикальной линией — видны фронты */
    if (have_data) {
        int prev = v2y(buf[m.start]);
        for (int x = 0; x < 128; x++) {
            int y = v2y(buf[m.start + x]);
            int a = prev < y ? prev : y, b = prev < y ? y : prev;
            for (int yy = a; yy <= b; yy++) ssd1306_pixel(x, yy, 1);
            prev = y;
        }
        if (m.triggered)                    /* метка триггера сверху */
            for (int i = 0; i < 3; i++) ssd1306_pixel(PRE_TRIG, Y_TOP - 1 + i, 1);
    }

    /* Низ: измеренные частота и скважность */
    p = line;
    strcpy(p, "f="); p += 2;
    put_freq(p, m.freq_hz); p += strlen(p);
    if (m.freq_hz > 0) {
        strcpy(p, " d="); p += 3;
        p = put_u(p, m.duty_x10 / 10);
        *p++ = '.';
        *p++ = (char)('0' + m.duty_x10 % 10);
        strcpy(p, "%");
    }
    ssd1306_text(0, 7, line);
}
