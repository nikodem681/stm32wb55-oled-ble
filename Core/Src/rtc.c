/*
 * RTC — часы-календарь внутри чипа.
 *
 * Время хранится в регистрах TR (часы:минуты:секунды) и DR (год-месяц-день, день недели)
 * в формате BCD: каждая десятичная цифра — отдельные 4 бита. 14:30 -> 0x14, 0x30.
 * Это удобно для дисплеев часов и неудобно для арифметики — поэтому ниже есть
 * перевод bcd()/from_bcd().
 *
 * RTC питается от «резервного домена»: при перезагрузке чипа он продолжает тикать.
 * Регистры RTC защищены от случайной записи — перед записью нужен «ключ» в WPR.
 */
#include "rtc.h"
#include "stm32wbxx.h"

#define MAGIC 0x32F2C10Cu        /* в BKP0R: «время выставлено» */

static uint32_t bcd(uint32_t v)      { return ((v / 10) << 4) | (v % 10); }
static uint32_t from_bcd(uint32_t v) { return (v >> 4) * 10 + (v & 0xF); }

int rtc_days_in_month(int y, int m)
{
    static const uint8_t d[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    int leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
    return d[m - 1] + (m == 2 && leap);
}

/* День недели по алгоритму Сакамото: 0 = вс ... 6 = сб -> переводим в 1 = пн .. 7 = вс */
int rtc_weekday(int y, int m, int d)
{
    static const int t[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
    if (m < 3) y -= 1;
    int w = (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
    return w == 0 ? 7 : w;
}

void rtc_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_RTCAPBEN;      /* доступ к регистрам RTC */
    (void)RCC->APB1ENR1;
    PWR->CR1 |= PWR_CR1_DBP;                      /* резервный домен: разрешить запись */

    /* Источник RTC — LSE (RTCSEL = 01). Выбор делается один раз: после перезагрузки
       RTC уже включён и тикает — тогда ничего не трогаем, чтобы не сбить время. */
    if (!(RCC->BDCR & RCC_BDCR_RTCEN)) {
        RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL_Msk) | (1U << RCC_BDCR_RTCSEL_Pos);
        RCC->BDCR |= RCC_BDCR_RTCEN;
    }
}

int rtc_is_set(void)
{
    return (RTC->ISR & RTC_ISR_INITS) && RTC->BKP0R == MAGIC;
}

void rtc_get(rtc_time_t *t)
{
    /* Чтение TR «замораживает» теневую копию DR до её чтения — дата и время согласованы.
       Читаем дважды и сравниваем, чтобы не попасть ровно на смену секунды. */
    uint32_t tr, dr;
    do {
        tr = RTC->TR;
        dr = RTC->DR;
    } while (tr != RTC->TR);
    (void)RTC->DR;

    t->hour  = (uint8_t)from_bcd((tr >> RTC_TR_HU_Pos) & 0x3F);
    t->min   = (uint8_t)from_bcd((tr >> RTC_TR_MNU_Pos) & 0x7F);
    t->sec   = (uint8_t)from_bcd((tr >> RTC_TR_SU_Pos) & 0x7F);
    t->year  = (uint16_t)(2000 + from_bcd((dr >> RTC_DR_YU_Pos) & 0xFF));
    t->month = (uint8_t)from_bcd((dr >> RTC_DR_MU_Pos) & 0x1F);
    t->day   = (uint8_t)from_bcd((dr >> RTC_DR_DU_Pos) & 0x3F);
    t->wday  = (uint8_t)((dr >> RTC_DR_WDU_Pos) & 0x7);
}

uint32_t rtc_seconds(const rtc_time_t *t)
{
    uint32_t days = 0;
    for (int y = 2000; y < t->year; y++) days += 365 + (rtc_days_in_month(y, 2) == 29);
    for (int m = 1; m < t->month; m++) days += (uint32_t)rtc_days_in_month(t->year, m);
    days += t->day - 1u;
    return ((days * 24 + t->hour) * 60 + t->min) * 60 + t->sec;
}

uint32_t rtc_last_sync(void) { return RTC->BKP1R; }

int rtc_set(rtc_time_t *t)
{
    if (t->year < 2000 || t->year > 2099 || t->month < 1 || t->month > 12 ||
        t->day < 1 || t->day > rtc_days_in_month(t->year, t->month) ||
        t->hour > 23 || t->min > 59 || t->sec > 59)
        return -1;
    t->wday = (uint8_t)rtc_weekday(t->year, t->month, t->day);

    RTC->WPR = 0xCA;                              /* ключ: снять защиту записи */
    RTC->WPR = 0x53;
    RTC->ISR |= RTC_ISR_INIT;                     /* режим настройки: счёт остановлен */
    while (!(RTC->ISR & RTC_ISR_INITF)) { }

    /* 32768 Гц / (127+1) / (255+1) = 1 Гц — ровно один тик в секунду */
    RTC->PRER = 255;
    RTC->PRER |= 127U << RTC_PRER_PREDIV_A_Pos;

    RTC->TR = (bcd(t->hour) << RTC_TR_HU_Pos) | (bcd(t->min) << RTC_TR_MNU_Pos) | (bcd(t->sec) << RTC_TR_SU_Pos);
    RTC->DR = (bcd(t->year - 2000u) << RTC_DR_YU_Pos) | ((uint32_t)t->wday << RTC_DR_WDU_Pos)
            | (bcd(t->month) << RTC_DR_MU_Pos) | (bcd(t->day) << RTC_DR_DU_Pos);
    RTC->CR &= ~RTC_CR_FMT;                       /* 24-часовой формат */

    RTC->ISR &= ~RTC_ISR_INIT;                    /* поехали! */

    /* Читаем мы не сами счётчики, а их «теневые» копии — они обновляются от кварца
       32 кГц, то есть не мгновенно. Сбрасываем флаг RSF и ждём, пока копии догонят. */
    RTC->ISR &= ~RTC_ISR_RSF;
    while (!(RTC->ISR & RTC_ISR_RSF)) { }
    RTC->WPR = 0xFF;                              /* защиту обратно */

    RTC->BKP0R = MAGIC;
    RTC->BKP1R = rtc_seconds(t);                  /* запомнить момент синхронизации */
    return 0;
}
