#ifndef TEMP_H
#define TEMP_H

#include <stdint.h>

/* Встроенный в STM32WB55 датчик температуры (через АЦП).
   Меряет температуру кристалла: обычно на 2..5 °C выше комнатной. */

void    temp_init(void);
int32_t temp_read_x10(void);   /* температура в десятых долях °C: 314 = 31.4 °C */

#endif
