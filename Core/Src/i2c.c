#include "i2c.h"
#include "stm32wbxx.h"

#define SCL_PIN 8
#define SDA_PIN 9
#define TIMEOUT 100000U   /* итераций опроса; на 16 МГц это несколько миллисекунд */

static void short_delay(void)
{
    for (volatile uint32_t i = 0; i < SystemCoreClock / 400000; i++) { }   /* ~5 мкс при любой частоте */
}

static void pin_mode(uint32_t pin, uint32_t mode)
{
    GPIOB->MODER = (GPIOB->MODER & ~(3U << (pin * 2))) | (mode << (pin * 2));
}

/*
 * Восстановление «зависшей» шины.
 * Если МК перезагрузился посреди передачи, ведомое устройство может
 * держать SDA в нуле, ожидая продолжения тактов. Лечится так: вручную
 * подаём до 9 импульсов на SCL, пока SDA не отпустят, затем формируем STOP.
 */
static void bus_recover(void)
{
    GPIOB->BSRR = (1U << SCL_PIN) | (1U << SDA_PIN);    /* обе в 1 (open-drain = отпущены) */
    pin_mode(SCL_PIN, 1);                               /* обычный выход */
    pin_mode(SDA_PIN, 1);
    short_delay();

    for (int i = 0; i < 9 && !(GPIOB->IDR & (1U << SDA_PIN)); i++) {
        GPIOB->BRR  = 1U << SCL_PIN; short_delay();
        GPIOB->BSRR = 1U << SCL_PIN; short_delay();
    }
    /* STOP: SDA 0 -> 1 при SCL = 1 */
    GPIOB->BRR  = 1U << SCL_PIN; short_delay();
    GPIOB->BRR  = 1U << SDA_PIN; short_delay();
    GPIOB->BSRR = 1U << SCL_PIN; short_delay();
    GPIOB->BSRR = 1U << SDA_PIN; short_delay();
}

void i2c1_init(void)
{
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;     /* тактирование порта B */
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;     /* тактирование I2C1 */
    (void)RCC->APB1ENR1;

    /* PB8/PB9: open-drain, высокая скорость, внутренняя подтяжка вверх */
    GPIOB->OTYPER  |= (1U << SCL_PIN) | (1U << SDA_PIN);
    GPIOB->OSPEEDR |= (3U << (SCL_PIN * 2)) | (3U << (SDA_PIN * 2));
    GPIOB->PUPDR = (GPIOB->PUPDR & ~((3U << (SCL_PIN * 2)) | (3U << (SDA_PIN * 2))))
                 | (1U << (SCL_PIN * 2)) | (1U << (SDA_PIN * 2));

    bus_recover();

    /* Альтернативная функция AF4 = I2C1 */
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xFFU)) | (4U << 0) | (4U << 4);
    pin_mode(SCL_PIN, 2);
    pin_mode(SDA_PIN, 2);

    /* Сброс периферии, чтобы стереть состояние, оставшееся от старой прошивки */
    RCC->APB1RSTR1 |=  RCC_APB1RSTR1_I2C1RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C1RST;

    I2C1->CR1 = 0;
    /* TIMINGR для 100 кГц (по таблице из Reference Manual RM0434):
       делитель PRESC даёт такт 4 МГц (250 нс), дальше в этих тактах:
       SCLDEL=4, SDADEL=2, SCLH=0x0F (4 мкс), SCLL=0x13 (5 мкс) */
    uint32_t presc = SystemCoreClock / 4000000 - 1;     /* 32 МГц -> 7, 64 МГц -> 15 (больше 15 нельзя) */
    I2C1->TIMINGR = (presc << 28) | 0x00420F13;
    I2C1->CR1 = I2C_CR1_PE;                   /* включаем */
}

static i2c_status_t wait_flag(uint32_t flag)
{
    for (uint32_t t = 0; t < TIMEOUT; t++) {
        uint32_t isr = I2C1->ISR;
        if (isr & I2C_ISR_NACKF) {
            /* Устройство не ответило. С AUTOEND аппаратура сама пошлёт STOP. */
            for (uint32_t s = 0; s < TIMEOUT && !(I2C1->ISR & I2C_ISR_STOPF); s++) { }
            I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
            return I2C_ERR_NACK;
        }
        if (isr & flag) return I2C_OK;
    }
    return I2C_ERR_TIMEOUT;
}

static i2c_status_t start(uint8_t addr7, uint16_t nbytes)
{
    for (uint32_t t = 0; I2C1->ISR & I2C_ISR_BUSY; t++) {
        if (t > TIMEOUT) return I2C_ERR_BUSY;
    }
    I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
    I2C1->CR2 = ((uint32_t)addr7 << 1)
              | ((uint32_t)nbytes << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_AUTOEND               /* STOP автоматически после nbytes */
              | I2C_CR2_START;                /* RD_WRN = 0 -> запись */
    return I2C_OK;
}

static i2c_status_t finish(void)
{
    i2c_status_t st = wait_flag(I2C_ISR_STOPF);
    I2C1->ICR = I2C_ICR_STOPCF;
    return st;
}

static i2c_status_t send_byte(uint8_t b)
{
    i2c_status_t st = wait_flag(I2C_ISR_TXIS);
    if (st == I2C_OK) I2C1->TXDR = b;
    return st;
}

i2c_status_t i2c1_probe(uint8_t addr7)
{
    i2c_status_t st = start(addr7, 0);   /* только адрес, без данных */
    if (st != I2C_OK) return st;
    return finish();
}

i2c_status_t i2c1_write2(uint8_t addr7, uint8_t prefix, const uint8_t *data, uint16_t len)
{
    if (len > 254) return I2C_ERR_TIMEOUT;   /* NBYTES — 8 бит; нам больше и не надо */

    i2c_status_t st = start(addr7, len + 1);
    if (st != I2C_OK) return st;

    if ((st = send_byte(prefix)) != I2C_OK) return st;
    for (uint16_t i = 0; i < len; i++) {
        if ((st = send_byte(data[i])) != I2C_OK) return st;
    }
    return finish();
}

i2c_status_t i2c1_write(uint8_t addr7, const uint8_t *data, uint16_t len)
{
    if (len == 0) return i2c1_probe(addr7);
    return i2c1_write2(addr7, data[0], data + 1, len - 1);
}
