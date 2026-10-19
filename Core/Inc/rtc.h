#ifndef RTC_H
#define RTC_H

#include <stdint.h>

/* Часы-календарь RTC, тикают от кварца LSE 32.768 кГц.
   Переживают перезагрузку и перепрошивку, но не выключение питания платы. */

typedef struct {
    uint16_t year;      /* 2000..2099 */
    uint8_t  month;     /* 1..12 */
    uint8_t  day;       /* 1..31 */
    uint8_t  wday;      /* 1 = понедельник .. 7 = воскресенье */
    uint8_t  hour, min, sec;
} rtc_time_t;

void     rtc_init(void);                      /* вызывать после запуска LSE */
int      rtc_is_set(void);                    /* время уже выставляли? */
void     rtc_get(rtc_time_t *t);
int      rtc_set(rtc_time_t *t);              /* 0 — ок; день недели посчитает сама */
uint32_t rtc_seconds(const rtc_time_t *t);    /* секунд с 2000-01-01 00:00:00 */
uint32_t rtc_last_sync(void);                 /* rtc_seconds() момента последней установки */

int      rtc_weekday(int y, int m, int d);    /* 1 = пн .. 7 = вс */
int      rtc_days_in_month(int y, int m);

#endif
