#ifndef BLE_APP_H
#define BLE_APP_H

#include <stdint.h>

/*
 * Bluetooth Low Energy: плата видна телефону как «WB55-OLED».
 *   - Nordic UART Service — «COM-порт по Bluetooth»: текст туда-обратно;
 *   - Environmental Sensing — температура кристалла, обновляется раз в секунду.
 */

typedef enum {
    BLE_STATE_STARTING = 0,   /* запускаем радио-ядро */
    BLE_STATE_ADVERTISING,    /* рекламируемся — ждём, пока телефон подключится */
    BLE_STATE_CONNECTED,      /* телефон подключён */
    BLE_STATE_ERROR,          /* что-то пошло не так — см. g_ble_error */
} ble_state_t;

extern volatile uint32_t g_ble_fw_version;   /* версия стека: 0x00MMmmss (1.24.0 = 0x011800) */
extern volatile uint32_t g_ble_error;        /* код ошибки, если BLE_STATE_ERROR */

void        ble_app_start(void);             /* вызвать из main до запуска планировщика */
ble_state_t ble_app_state(void);
void        ble_print(const char *s);        /* отправить текст на телефон (если подписан) */

/* Реализуется в main.c: выполнить строку-команду, ответы печатать через out() */
void app_handle_line(const char *line, void (*out)(const char *));

#endif
