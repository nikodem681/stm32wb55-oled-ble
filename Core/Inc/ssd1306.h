#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include "i2c.h"

#define SSD1306_WIDTH   128
#define SSD1306_HEIGHT  64

/* Некоторые «0.96 OLED» на самом деле на контроллере SH1106 (132 столбца).
   Если картинка сдвинута на 2 пикселя и справа мусор — поставь 2. */
#define SSD1306_COL_OFFSET 0

extern uint8_t ssd1306_addr;   /* найденный I2C-адрес (0x3C или 0x3D) */

i2c_status_t ssd1306_init(void);
void         ssd1306_clear(void);
void         ssd1306_pixel(int x, int y, int on);
void         ssd1306_text(int x, int page, const char *s);       /* шрифт 5x7, строка = page 0..7 */
void         ssd1306_text_big(int x, int page, const char *s);   /* шрифт 10x14, 2 строки */
void         ssd1306_rect(int x, int y, int w, int h);
i2c_status_t ssd1306_update(void);                               /* кадровый буфер -> экран */

#endif
