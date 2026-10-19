/*
 * Экран «Часы».
 *
 *  ┌────────────┬─────────────────┐
 *  │  циферблат │ 14:30:05        │
 *  │  со        │ Mon 19 Oct      │
 *  │  стрелками │ M T W T F S S   │
 *  │  (64x64)   │ календарик      │
 *  └────────────┴─────────────────┘
 *
 * Стрелки считаются через sin/cos: угол стрелки -> координаты её конца.
 * В Cortex-M4F есть аппаратный блок плавающей точки (FPU), так что это быстро.
 */
#include "clock.h"
#include "rtc.h"
#include "ssd1306.h"
#include <math.h>
#include <string.h>

#define CX 31                 /* центр циферблата */
#define CY 31
#define R  30                 /* радиус */
#define PANEL_X 66            /* левый край правой половины */

/* ---- Мини-шрифт 3x5: каждая строка — 3 бита (старший бит — левый пиксель) ---- */
static const uint8_t tiny_digits[10][5] = {
    {7,5,5,5,7}, {2,6,2,2,7}, {7,1,7,4,7}, {7,1,7,1,7}, {5,5,7,1,1},
    {7,4,7,1,7}, {7,4,7,5,7}, {7,1,1,1,1}, {7,5,7,5,7}, {7,5,7,1,7},
};
static const uint8_t tiny_letters[][5] = {   /* M T W F S — для шапки календаря */
    {5,7,7,5,5}, {7,2,2,2,2}, {5,5,7,7,5}, {7,4,6,4,4}, {7,4,7,1,7},
};

static void tiny_glyph(int x, int y, const uint8_t g[5], int on)
{
    for (int r = 0; r < 5; r++)
        for (int c = 0; c < 3; c++)
            if (g[r] & (4 >> c)) ssd1306_pixel(x + c, y + r, on);
}

/* Отрезок по алгоритму Брезенхэма: только целые числа, без деления */
static void line(int x0, int y0, int x1, int y1)
{
    int dx = x1 > x0 ? x1 - x0 : x0 - x1, sx = x0 < x1 ? 1 : -1;
    int dy = y1 > y0 ? y0 - y1 : y1 - y0, sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        ssd1306_pixel(x0, y0, 1);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/* Точка на окружности: угол в «минутах» (0..60), 0 — вверх, по часовой стрелке */
static void polar(float minutes, float len, int *x, int *y)
{
    float a = minutes * (2.0f * 3.14159265f / 60.0f);
    *x = CX + (int)lroundf(len * sinf(a));
    *y = CY - (int)lroundf(len * cosf(a));
}

static void hand(float minutes, float len)
{
    int x, y;
    polar(minutes, len, &x, &y);
    line(CX, CY, x, y);
}

static void draw_dial(const rtc_time_t *t)
{
    /* Окружность: 120 точек — проще, чем алгоритм окружности, и выглядит ровно */
    for (int i = 0; i < 120; i++) {
        int x, y;
        polar(i * 0.5f, R, &x, &y);
        ssd1306_pixel(x, y, 1);
    }
    /* Риски: длинные на 12/3/6/9, короткие на остальных часах */
    for (int h = 0; h < 12; h++) {
        int x0, y0, x1, y1;
        polar(h * 5.0f, R - (h % 3 ? 3 : 6), &x0, &y0);
        polar(h * 5.0f, R - 1, &x1, &y1);
        line(x0, y0, x1, y1);
    }

    /* Стрелки двигаются плавно: минутная учитывает секунды, часовая — минуты */
    float m = t->min + t->sec / 60.0f;
    float h = (t->hour % 12) * 5.0f + m / 12.0f;
    hand(h, 14);
    hand(m, 22);
    hand(t->sec, 26);
    /* Утолщаем часовую и минутную: рисуем ещё раз со сдвигом на пиксель */
    int x, y;
    polar(h, 14, &x, &y);  line(CX + 1, CY, x + 1, y);
    polar(m, 22, &x, &y);  line(CX, CY + 1, x, y + 1);
    for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++) ssd1306_pixel(CX + dx, CY + dy, 1);   /* ось */
}

static void draw_calendar(const rtc_time_t *t)
{
    /* Шапка: Пн Вт Ср Чт Пт Сб Вс латиницей: M T W T F S S */
    static const uint8_t head[7] = { 0, 1, 2, 1, 3, 4, 4 };
    for (int c = 0; c < 7; c++) tiny_glyph(PANEL_X + c * 9 + 2, 17, tiny_letters[head[c]], 1);

    /* Числа месяца: столбец = день недели первого числа + сдвиг */
    int first = rtc_weekday(t->year, t->month, 1);           /* 1 = пн */
    int days  = rtc_days_in_month(t->year, t->month);
    for (int d = 1; d <= days; d++) {
        int cell = first - 1 + d - 1;
        int x = PANEL_X + (cell % 7) * 9;
        int y = 24 + (cell / 7) * 6;
        int on = 1;
        if (d == t->day) {                                    /* сегодня — белый квадрат, цифры чёрные */
            for (int yy = y - 1; yy <= y + 5; yy++)
                for (int xx = x - 1; xx <= x + 7; xx++) ssd1306_pixel(xx, yy, 1);
            on = 0;
        }
        if (d >= 10) tiny_glyph(x, y, tiny_digits[d / 10], on);
        tiny_glyph(x + 4, y, tiny_digits[d % 10], on);
    }
}

static char *two(char *p, int v) { *p++ = (char)('0' + v / 10); *p++ = (char)('0' + v % 10); return p; }

void clock_draw(void)
{
    if (!rtc_is_set()) {
        ssd1306_text(0, 1, "Clock is not set.");
        ssd1306_text(0, 3, "Run tools/sync_time");
        ssd1306_text(0, 4, "on the PC, or type:");
        ssd1306_text(0, 6, "time 2026-10-19");
        ssd1306_text(0, 7, "     14:30:00");
        return;
    }

    rtc_time_t t;
    rtc_get(&t);
    draw_dial(&t);

    static const char *const wd[] = { "", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun" };
    static const char *const mo[] = { "", "Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                      "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
    char s[16], *p = s;
    p = two(p, t.hour); *p++ = ':'; p = two(p, t.min); *p++ = ':'; p = two(p, t.sec); *p = '\0';
    ssd1306_text(PANEL_X + 4, 0, s);

    p = s;
    strcpy(p, wd[t.wday]); p += 3; *p++ = ' ';
    p = two(p, t.day); *p++ = ' ';
    strcpy(p, mo[t.month]);
    ssd1306_text(PANEL_X + 1, 1, s);

    draw_calendar(&t);
}
