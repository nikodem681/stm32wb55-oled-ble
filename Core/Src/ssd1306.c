#include "ssd1306.h"
#include "font5x7.h"
#include <string.h>

/*
 * Как устроен SSD1306:
 *  - экран 128x64 разбит на 8 «страниц» (page) по 8 строк пикселей;
 *  - один байт = вертикальный столбик из 8 пикселей внутри страницы;
 *  - по I2C каждая посылка начинается с управляющего байта:
 *      0x00 — дальше идут команды, 0x40 — дальше идут данные (пиксели).
 */

#define CTRL_CMD  0x00
#define CTRL_DATA 0x40

uint8_t ssd1306_addr = 0x3C;
static uint8_t fb[SSD1306_HEIGHT / 8][SSD1306_WIDTH];   /* кадровый буфер, 1 КБ */

static i2c_status_t cmds(const uint8_t *c, uint16_t n)
{
    return i2c1_write2(ssd1306_addr, CTRL_CMD, c, n);
}

i2c_status_t ssd1306_init(void)
{
    /* Ищем дисплей на двух стандартных адресах */
    if (i2c1_probe(0x3C) == I2C_OK)      ssd1306_addr = 0x3C;
    else if (i2c1_probe(0x3D) == I2C_OK) ssd1306_addr = 0x3D;
    else return I2C_ERR_NACK;

    static const uint8_t init_seq[] = {
        0xAE,             /* дисплей выключен на время настройки */
        0xD5, 0x80,       /* частота внутреннего генератора */
        0xA8, 0x3F,       /* мультиплекс: 64 строки */
        0xD3, 0x00,       /* вертикальный сдвиг 0 */
        0x40,             /* начальная строка 0 */
        0x8D, 0x14,       /* ВКЛ встроенный преобразователь (charge pump) — без него экран тёмный! */
        0x20, 0x02,       /* режим адресации по страницам (работает и на SH1106) */
        0xA1,             /* отражение по горизонтали (столбец 127 -> SEG0) */
        0xC8,             /* сканирование строк сверху вниз */
        0xDA, 0x12,       /* конфигурация выводов COM для 128x64 */
        0x81, 0xCF,       /* контраст */
        0xD9, 0xF1,       /* pre-charge период */
        0xDB, 0x40,       /* уровень VCOMH */
        0xA4,             /* показывать содержимое RAM */
        0xA6,             /* нормальный (не инверсный) режим */
        0x2E,             /* выключить прокрутку */
        0xAF,             /* дисплей включен */
    };
    i2c_status_t st = cmds(init_seq, sizeof init_seq);
    if (st != I2C_OK) return st;

    ssd1306_clear();
    return ssd1306_update();
}

void ssd1306_clear(void)
{
    memset(fb, 0, sizeof fb);
}

void ssd1306_pixel(int x, int y, int on)
{
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) return;
    if (on) fb[y / 8][x] |=  (uint8_t)(1U << (y % 8));
    else    fb[y / 8][x] &= (uint8_t)~(1U << (y % 8));
}

void ssd1306_text(int x, int page, const char *s)
{
    if (page < 0 || page >= SSD1306_HEIGHT / 8) return;
    for (; *s && x <= SSD1306_WIDTH - 6; s++, x += 6) {
        char c = (*s < 0x20 || *s > 0x7E) ? '?' : *s;
        memcpy(&fb[page][x], font5x7[c - 0x20], 5);
        fb[page][x + 5] = 0;   /* промежуток между буквами */
    }
}

void ssd1306_text_big(int x, int page, const char *s)
{
    int y0 = page * 8;
    for (; *s && x <= SSD1306_WIDTH - 12; s++, x += 12) {
        char c = (*s < 0x20 || *s > 0x7E) ? '?' : *s;
        const uint8_t *g = font5x7[c - 0x20];
        for (int col = 0; col < 5; col++)
            for (int row = 0; row < 8; row++) {
                int on = (g[col] >> row) & 1;   /* каждый пиксель шрифта -> квадрат 2x2 */
                ssd1306_pixel(x + col * 2,     y0 + row * 2,     on);
                ssd1306_pixel(x + col * 2 + 1, y0 + row * 2,     on);
                ssd1306_pixel(x + col * 2,     y0 + row * 2 + 1, on);
                ssd1306_pixel(x + col * 2 + 1, y0 + row * 2 + 1, on);
            }
    }
}

void ssd1306_rect(int x, int y, int w, int h)
{
    for (int i = x; i < x + w; i++) { ssd1306_pixel(i, y, 1); ssd1306_pixel(i, y + h - 1, 1); }
    for (int j = y; j < y + h; j++) { ssd1306_pixel(x, j, 1); ssd1306_pixel(x + w - 1, j, 1); }
}

i2c_status_t ssd1306_update(void)
{
    for (uint8_t p = 0; p < SSD1306_HEIGHT / 8; p++) {
        const uint8_t pos[] = {
            (uint8_t)(0xB0 | p),                              /* страница p */
            (uint8_t)(0x00 | (SSD1306_COL_OFFSET & 0x0F)),    /* столбец, младшие 4 бита */
            (uint8_t)(0x10 | (SSD1306_COL_OFFSET >> 4)),      /* столбец, старшие 4 бита */
        };
        i2c_status_t st = cmds(pos, sizeof pos);
        if (st == I2C_OK) st = i2c1_write2(ssd1306_addr, CTRL_DATA, fb[p], SSD1306_WIDTH);
        if (st != I2C_OK) return st;
    }
    return I2C_OK;
}
