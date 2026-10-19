#ifndef TEMP_H
#define TEMP_H

#include <stdint.h>

/* Встроенный в STM32WB55 датчик температуры (через АЦП).
   Меряет температуру кристалла: обычно на 2..5 °C выше комнатной. */

void    temp_init(void);           /* включает и калибрует АЦП — вызывать из задачи */

/* Общий доступ к АЦП (им пользуется и осциллограф) */
void    adc_shared_init(void);     /* создать мьютекс — до запуска планировщика */
int     adc_is_ready(void);        /* АЦП уже включён и откалиброван */
void    adc_lock(void);
void    adc_unlock(void);
int32_t temp_read_x10(void);   /* температура в десятых долях °C: 314 = 31.4 °C */

#endif
