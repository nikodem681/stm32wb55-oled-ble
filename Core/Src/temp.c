#include "temp.h"
#include "stm32wbxx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Каналы АЦП внутренних источников на STM32WB55 */
#define CH_VREFINT  0
#define CH_TEMP     17

/* Заводская калибровка — записана в системную память каждого чипа отдельно */
#define TS_CAL1      (*(const uint16_t *)0x1FFF75A8)   /* сырое значение при 30 °C,  VDDA = 3.0 В */
#define TS_CAL2      (*(const uint16_t *)0x1FFF75CA)   /* сырое значение при 130 °C, VDDA = 3.0 В */
#define VREFINT_CAL  (*(const uint16_t *)0x1FFF75AA)   /* VREFINT при VDDA = 3.6 В */

/* АЦП один на двоих: датчик температуры и осциллограф. Кто взял мьютекс — тот и
   хозяин АЦП, а перед работой каждый выставляет свои настройки заново. */
static SemaphoreHandle_t adc_mtx;
static volatile int adc_ready;

void adc_shared_init(void) { adc_mtx = xSemaphoreCreateMutex(); }
int  adc_is_ready(void)    { return adc_ready; }
void adc_lock(void)        { xSemaphoreTake(adc_mtx, portMAX_DELAY); }
void adc_unlock(void)      { xSemaphoreGive(adc_mtx); }

void temp_init(void)
{
    adc_lock();
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
    (void)RCC->AHB2ENR;

    /* Тактирование АЦП от шины HCLK (32 МГц) без делителя, плюс включаем
       внутренние каналы: датчик температуры (TSEN) и опорное напряжение (VREFEN) */
    ADC1_COMMON->CCR = ADC_CCR_CKMODE_0 | ADC_CCR_TSEN | ADC_CCR_VREFEN;

    /* После сброса АЦП в «глубоком сне». Будим и включаем его стабилизатор. */
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;
    vTaskDelay(pdMS_TO_TICKS(1));          /* нужно ≥ 20 мкс; заодно ≥ 120 мкс для датчика */

    /* Самокалибровка АЦП (компенсирует его собственную погрешность) */
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL) { }

    ADC1->ISR = ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) { }

    /* Время выборки 640.5 тактов (= 40 мкс). Датчику нужно не меньше 5 мкс. */
    ADC1->SMPR1 |= 7U << ADC_SMPR1_SMP0_Pos;
    ADC1->SMPR2 |= 7U << ADC_SMPR2_SMP17_Pos;
    adc_ready = 1;
    adc_unlock();
}

static uint32_t convert(uint32_t ch)
{
    ADC1->SQR1 = ch << ADC_SQR1_SQ1_Pos;    /* одно преобразование, канал ch */
    ADC1->ISR  = ADC_ISR_EOC;
    ADC1->CR  |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC)) { }
    return ADC1->DR;                        /* 12 бит: 0..4095 */
}

int32_t temp_read_x10(void)
{
    /* Усредняем 16 измерений — показания не будут «дрожать» */
    uint32_t vref = 0, ts = 0;
    adc_lock();
    /* Осциллограф мог оставить запуск от таймера и DMA — возвращаем «запуск программой» */
    ADC1->CFGR &= ~(ADC_CFGR_EXTEN_Msk | ADC_CFGR_DMAEN | ADC_CFGR_CONT);
    for (int i = 0; i < 16; i++) {
        vref += convert(CH_VREFINT);
        ts   += convert(CH_TEMP);
    }
    adc_unlock();

    /* 1. Реальное напряжение питания АЦП (мВ): VREFINT стабильна, по ней и считаем */
    int64_t vdda_mv = (int64_t)3600 * VREFINT_CAL * 16 / vref;

    /* 2. Приводим показание датчика к 3.0 В (при таком калибровали на заводе)
       и интерполируем по прямой между точками 30 °C и 130 °C.
       Сумму 16 измерений не делим на 16 сразу — так не теряем дробную часть. */
    int64_t ts16_3v = (int64_t)ts * vdda_mv / 3000;
    return (int32_t)(300 + (ts16_3v - 16 * TS_CAL1) * 1000 / (16 * (TS_CAL2 - TS_CAL1)));
}
