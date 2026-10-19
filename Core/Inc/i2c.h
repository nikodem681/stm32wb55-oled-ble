#ifndef I2C_H
#define I2C_H

#include <stdint.h>

/* I2C1: PB8 = SCL, PB9 = SDA (разъём Arduino D15/D14 на Nucleo). */

typedef enum {
    I2C_OK = 0,
    I2C_ERR_BUSY,     /* шина занята (кто-то держит линию в 0) */
    I2C_ERR_NACK,     /* устройство не ответило — нет по такому адресу / не подключено */
    I2C_ERR_TIMEOUT,
} i2c_status_t;

void         i2c1_init(void);
i2c_status_t i2c1_probe(uint8_t addr7);
i2c_status_t i2c1_write(uint8_t addr7, const uint8_t *data, uint16_t len);
/* Пишет префикс (1 байт) + данные одной транзакцией — удобно для SSD1306. */
i2c_status_t i2c1_write2(uint8_t addr7, uint8_t prefix, const uint8_t *data, uint16_t len);

#endif
