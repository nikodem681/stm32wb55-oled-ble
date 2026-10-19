#ifndef USB_CDC_H
#define USB_CDC_H

#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"

/*
 * USB CDC ACM — «виртуальный COM-порт» через разъём USB_USER (PA11 = D-, PA12 = D+).
 * В Windows появится как «Устройство с последовательным интерфейсом USB (COMx)».
 */

typedef enum {
    USB_STATE_OFF = 0,     /* кабель не подключён / ещё не опознаны */
    USB_STATE_CONFIGURED,  /* Windows опознала устройство */
    USB_STATE_PORT_OPEN,   /* программа-терминал открыла COM-порт (сигнал DTR) */
} usb_state_t;

void        usb_cdc_init(void);
usb_state_t usb_cdc_state(void);

/* Ждёт данные от компьютера до wait тиков. Возвращает число прочитанных байт. */
size_t usb_cdc_read(uint8_t *buf, size_t max, TickType_t wait);

/* Отправка на компьютер. Можно вызывать из любой задачи. */
void usb_cdc_write(const void *data, size_t len);
void usb_cdc_print(const char *s);

#endif
