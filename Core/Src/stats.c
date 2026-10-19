/*
 * Статистика задач FreeRTOS.
 *
 * При каждом переключении задач FreeRTOS читает «часы статистики» и прибавляет
 * прошедшее время к счётчику той задачи, которая только что работала.
 * Раз в секунду мы снимаем все счётчики (uxTaskGetSystemState) и вычитаем
 * прошлые значения — получаем, сколько микросекунд из этой секунды досталось
 * каждой задаче. IDLE — это задача FreeRTOS, которая работает, когда все
 * остальные ждут: её процент и есть «свободное время» процессора.
 */
#include "stats.h"
#include "ssd1306.h"
#include "stm32wbxx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define MAX_TASKS 12

typedef struct {
    char     name[configMAX_TASK_NAME_LEN];
    uint32_t cpu_x10;        /* загрузка за последнюю секунду, десятые процента */
    uint32_t stack_free;     /* минимальный запас стека за всё время, в словах (по 4 байта) */
} row_t;

static row_t rows[MAX_TASKS];
static int   n_rows;

/* ---- Часы статистики: счётчик тактов ядра Cortex-M4 ---- */
void stats_timer_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* включить блок отладки/трассировки */
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;             /* считать такты процессора */
}

/* 64 МГц / 64 = 1 МГц: 1 единица = 1 мкс; 32-битный счётчик переполнится через ~71 мин,
   но мы всегда берём разность за 1 секунду — переполнение ей не мешает */
uint32_t stats_timer_value(void) { return DWT->CYCCNT / (SystemCoreClock / 1000000); }

static void stats_task(void *arg)
{
    (void)arg;
    static TaskStatus_t st[MAX_TASKS];
    static struct { TaskHandle_t h; uint32_t t; } prev[MAX_TASKS];
    static int n_prev;
    row_t tmp[MAX_TASKS];

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint32_t total;
        int n = (int)uxTaskGetSystemState(st, MAX_TASKS, &total);

        /* Разности за секунду: ищем каждую задачу в прошлом снимке */
        uint32_t sum = 0, delta[MAX_TASKS];
        for (int i = 0; i < n; i++) {
            uint32_t before = st[i].ulRunTimeCounter;
            for (int j = 0; j < n_prev; j++)
                if (prev[j].h == st[i].xHandle) { before = prev[j].t; break; }
            delta[i] = st[i].ulRunTimeCounter - before;
            sum += delta[i];
        }
        for (int i = 0; i < n; i++) {
            prev[i].h = st[i].xHandle;
            prev[i].t = st[i].ulRunTimeCounter;
        }
        n_prev = n;
        if (sum == 0) continue;

        for (int i = 0; i < n; i++) {
            strncpy(tmp[i].name, st[i].pcTaskName, sizeof tmp[i].name - 1);
            tmp[i].name[sizeof tmp[i].name - 1] = '\0';
            tmp[i].cpu_x10 = (uint32_t)((uint64_t)delta[i] * 1000 / sum);
            tmp[i].stack_free = st[i].usStackHighWaterMark;
        }
        /* Сортировка по загрузке, самые «прожорливые» сверху (простая вставка) */
        for (int i = 1; i < n; i++)
            for (int j = i; j > 0 && tmp[j].cpu_x10 > tmp[j - 1].cpu_x10; j--) {
                row_t t = tmp[j]; tmp[j] = tmp[j - 1]; tmp[j - 1] = t;
            }

        taskENTER_CRITICAL();                /* копируем целиком, чтобы экран не увидел половину */
        memcpy(rows, tmp, sizeof(row_t) * (size_t)n);
        n_rows = n;
        taskEXIT_CRITICAL();
    }
}

void stats_start(void)
{
    xTaskCreate(stats_task, "stats", 256, NULL, 1, NULL);
}

/* ---- Форматирование ---- */

static char *put_u(char *p, uint32_t v, int width)
{
    char tmp[11];
    int n = 0;
    do { tmp[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (width-- > n) *p++ = ' ';
    while (n) *p++ = tmp[--n];
    return p;
}

/* "display    31.2%  210" */
static void format_row(const row_t *r, char *line)
{
    char *p = line;
    int len = (int)strlen(r->name);
    if (len > 9) len = 9;
    memcpy(p, r->name, (size_t)len); p += len;
    while (p < line + 9) *p++ = ' ';
    p = put_u(p, r->cpu_x10 / 10, 4);
    *p++ = '.';
    *p++ = (char)('0' + r->cpu_x10 % 10);
    *p++ = '%';
    p = put_u(p, r->stack_free, 5);
    *p = '\0';
}

static int snapshot(row_t *out)
{
    taskENTER_CRITICAL();
    int n = n_rows;
    memcpy(out, rows, sizeof(row_t) * (size_t)n);
    taskEXIT_CRITICAL();
    return n;
}

void stats_draw(void)
{
    row_t r[MAX_TASKS];
    char line[24];
    int n = snapshot(r);

    ssd1306_text(0, 0, "TASK       CPU  STACK");
    for (int x = 0; x < SSD1306_WIDTH; x++) ssd1306_pixel(x, 7, 1);
    if (n == 0) { ssd1306_text(0, 2, "measuring..."); return; }

    /* На экране 7 строк — показываем 7 самых загруженных задач */
    for (int i = 0; i < n && i < 7; i++) {
        format_row(&r[i], line);
        ssd1306_text(0, 1 + i, line);
    }
}

void stats_print(void (*out)(const char *))
{
    row_t r[MAX_TASKS];
    char line[24];
    int n = snapshot(r);

    if (n == 0) { out("Statistics not ready yet, try again in a second\r\n"); return; }
    out("TASK       CPU  STACK (free words, minimum ever)\r\n");
    for (int i = 0; i < n; i++) {
        format_row(&r[i], line);
        out(line);
        out("\r\n");
    }
}
