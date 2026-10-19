#ifndef SCOPE_H
#define SCOPE_H

#include <stdint.h>

/*
 * Простой осциллограф на АЦП.
 *   Вход:  A2 (PA1), 0..3.3 В — НЕ подавать больше 3.3 В и минус!
 *   Тест:  D3 (PA10) — ШИМ от TIM1. Соедини проводком D3 -> A2.
 */

void        scope_init(void);           /* ШИМ, таймер запуска АЦП, DMA */

void        scope_set_duty(uint32_t percent);
uint32_t    scope_duty(void);
void        scope_set_freq(uint32_t hz);
uint32_t    scope_freq(void);

void        scope_next_timebase(void);
const char *scope_timebase_name(void);  /* "500us" и т.п. — время на одну клетку */
void        scope_toggle_hold(void);
int         scope_is_hold(void);

/* Снять сигнал (если не HOLD) и нарисовать весь экран осциллографа в буфер дисплея */
void        scope_frame(void);

#endif
