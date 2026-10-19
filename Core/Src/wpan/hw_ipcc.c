/*
 * IPCC — «почтовый ящик» между нашим ядром (CPU1, Cortex-M4) и радио-ядром (CPU2, M0+).
 *
 * У IPCC 6 каналов в каждую сторону. Канал — это просто флаг:
 *   - CPU1 положил команду в общую память и ставит флаг «занято» (C1SCR.CHnS);
 *   - CPU2 забирает команду и снимает флаг -> у нас прерывание «канал свободен» (TX);
 *   - в обратную сторону CPU2 ставит свой флаг -> у нас прерывание «пришло» (RX),
 *     мы обрабатываем и снимаем флаг (C1SCR.CHnC).
 * Сами данные лежат в общей памяти SRAM2A — ими управляет транспортный слой ST (tl_mbox.c),
 * а этот файл реализует для него низкоуровневые функции HW_IPCC_* (см. hw.h).
 */
#include "stm32wbxx.h"

#define LL_IPCC_CHANNEL_1 (1U << 0)
#define LL_IPCC_CHANNEL_2 (1U << 1)
#define LL_IPCC_CHANNEL_3 (1U << 2)
#define LL_IPCC_CHANNEL_4 (1U << 3)
#define LL_IPCC_CHANNEL_5 (1U << 4)
#define LL_IPCC_CHANNEL_6 (1U << 5)

#include "hw.h"
#include "mbox_def.h"

/* Флаги и маски. Маска TX — биты 16..21 регистра C1MR, маска RX — биты 0..5. */
#define TX_FLAG_SET(ch)      (IPCC->C1SCR = (ch) << 16)
#define RX_FLAG_CLEAR(ch)    (IPCC->C1SCR = (ch))
#define TX_BUSY(ch)          (IPCC->C1TOC2SR & (ch))
#define RX_ARRIVED(ch)       (IPCC->C2TOC1SR & (ch))
#define TX_IRQ_ON(ch)        (IPCC->C1MR &= ~((ch) << 16))
#define TX_IRQ_OFF(ch)       (IPCC->C1MR |=  ((ch) << 16))
#define RX_IRQ_ON(ch)        (IPCC->C1MR &= ~(ch))
#define TX_IRQ_ENABLED(ch)   (!(IPCC->C1MR & ((ch) << 16)))
#define RX_IRQ_ENABLED(ch)   (!(IPCC->C1MR & (ch)))

static void (*free_buf_cb)(void);

static uint32_t crit_enter(void) { uint32_t p = __get_PRIMASK(); __disable_irq(); return p; }
static void     crit_exit(uint32_t p) { __set_PRIMASK(p); }

/* ---- Запуск CPU2 ---- */
void HW_IPCC_Enable(void)
{
    RCC->C2AHB3ENR |= RCC_C2AHB3ENR_IPCCEN;   /* CPU2 тоже нужна тактирование IPCC */
    /* Линия EXTI 41 будит CPU2 событием */
    EXTI->RTSR2  |= EXTI_RTSR2_RT41;
    EXTI->C2EMR2 |= EXTI_C2EMR2_EM41;
    __SEV();
    __WFE();
    PWR->CR4 |= PWR_CR4_C2BOOT;               /* <-- вот здесь CPU2 начинает работать */
}

void HW_IPCC_Init(void)
{
    RCC->AHB3ENR |= RCC_AHB3ENR_IPCCEN;
    (void)RCC->AHB3ENR;
    IPCC->C1CR |= IPCC_C1CR_RXOIE | IPCC_C1CR_TXFIE;
    /* Приоритет 6: ниже configMAX_SYSCALL_INTERRUPT_PRIORITY, можно звать FreeRTOS ...FromISR */
    NVIC_SetPriority(IPCC_C1_RX_IRQn, 6);
    NVIC_SetPriority(IPCC_C1_TX_IRQn, 6);
    NVIC_EnableIRQ(IPCC_C1_RX_IRQn);
    NVIC_EnableIRQ(IPCC_C1_TX_IRQn);
}

/* ---- BLE ---- */
void HW_IPCC_BLE_Init(void)
{
    uint32_t p = crit_enter();
    RX_IRQ_ON(HW_IPCC_BLE_EVENT_CHANNEL);
    crit_exit(p);
}

void HW_IPCC_BLE_SendCmd(void)
{
    TX_FLAG_SET(HW_IPCC_BLE_CMD_CHANNEL);
}

void HW_IPCC_BLE_SendAclData(void)
{
    TX_FLAG_SET(HW_IPCC_HCI_ACL_DATA_CHANNEL);
    uint32_t p = crit_enter();
    TX_IRQ_ON(HW_IPCC_HCI_ACL_DATA_CHANNEL);
    crit_exit(p);
}

/* ---- Системный канал (SHCI): команды «включи BLE», «какая версия» ---- */
void HW_IPCC_SYS_Init(void)
{
    uint32_t p = crit_enter();
    RX_IRQ_ON(HW_IPCC_SYSTEM_EVENT_CHANNEL);
    crit_exit(p);
}

void HW_IPCC_SYS_SendCmd(void)
{
    TX_FLAG_SET(HW_IPCC_SYSTEM_CMD_RSP_CHANNEL);
    uint32_t p = crit_enter();
    TX_IRQ_ON(HW_IPCC_SYSTEM_CMD_RSP_CHANNEL);
    crit_exit(p);
}

/* ---- Возврат буферов событий обратно CPU2 ---- */
void HW_IPCC_MM_SendFreeBuf(void (*cb)(void))
{
    if (TX_BUSY(HW_IPCC_MM_RELEASE_BUFFER_CHANNEL)) {
        free_buf_cb = cb;                     /* канал занят — сделаем, когда освободится */
        uint32_t p = crit_enter();
        TX_IRQ_ON(HW_IPCC_MM_RELEASE_BUFFER_CHANNEL);
        crit_exit(p);
    } else {
        cb();
        TX_FLAG_SET(HW_IPCC_MM_RELEASE_BUFFER_CHANNEL);
    }
}

void HW_IPCC_TRACES_Init(void)
{
    uint32_t p = crit_enter();
    RX_IRQ_ON(HW_IPCC_TRACES_CHANNEL);
    crit_exit(p);
}

/* Слабые заглушки: настоящие реализации — в tl_mbox.c */
__attribute__((weak)) void HW_IPCC_BLE_RxEvtNot(void) {}
__attribute__((weak)) void HW_IPCC_BLE_AclDataAckNot(void) {}
__attribute__((weak)) void HW_IPCC_SYS_CmdEvtNot(void) {}
__attribute__((weak)) void HW_IPCC_SYS_EvtNot(void) {}
__attribute__((weak)) void HW_IPCC_TRACES_EvtNot(void) {}

/* ---- Прерывания ---- */
void IPCC_C1_RX_IRQHandler(void)
{
    if (RX_ARRIVED(HW_IPCC_SYSTEM_EVENT_CHANNEL) && RX_IRQ_ENABLED(HW_IPCC_SYSTEM_EVENT_CHANNEL)) {
        HW_IPCC_SYS_EvtNot();
        RX_FLAG_CLEAR(HW_IPCC_SYSTEM_EVENT_CHANNEL);
    } else if (RX_ARRIVED(HW_IPCC_BLE_EVENT_CHANNEL) && RX_IRQ_ENABLED(HW_IPCC_BLE_EVENT_CHANNEL)) {
        HW_IPCC_BLE_RxEvtNot();
        RX_FLAG_CLEAR(HW_IPCC_BLE_EVENT_CHANNEL);
    } else if (RX_ARRIVED(HW_IPCC_TRACES_CHANNEL) && RX_IRQ_ENABLED(HW_IPCC_TRACES_CHANNEL)) {
        HW_IPCC_TRACES_EvtNot();
        RX_FLAG_CLEAR(HW_IPCC_TRACES_CHANNEL);
    }
}

void IPCC_C1_TX_IRQHandler(void)
{
    if (!TX_BUSY(HW_IPCC_SYSTEM_CMD_RSP_CHANNEL) && TX_IRQ_ENABLED(HW_IPCC_SYSTEM_CMD_RSP_CHANNEL)) {
        TX_IRQ_OFF(HW_IPCC_SYSTEM_CMD_RSP_CHANNEL);
        HW_IPCC_SYS_CmdEvtNot();              /* CPU2 ответил на системную команду */
    } else if (!TX_BUSY(HW_IPCC_MM_RELEASE_BUFFER_CHANNEL) && TX_IRQ_ENABLED(HW_IPCC_MM_RELEASE_BUFFER_CHANNEL)) {
        TX_IRQ_OFF(HW_IPCC_MM_RELEASE_BUFFER_CHANNEL);
        free_buf_cb();
        TX_FLAG_SET(HW_IPCC_MM_RELEASE_BUFFER_CHANNEL);
    } else if (!TX_BUSY(HW_IPCC_HCI_ACL_DATA_CHANNEL) && TX_IRQ_ENABLED(HW_IPCC_HCI_ACL_DATA_CHANNEL)) {
        TX_IRQ_OFF(HW_IPCC_HCI_ACL_DATA_CHANNEL);
        HW_IPCC_BLE_AclDataAckNot();
    }
}

/* Сброс IPCC: после перепрошивки там могли остаться флаги от прошлого запуска */
void HW_IPCC_Reset(void)
{
    RCC->AHB3ENR |= RCC_AHB3ENR_IPCCEN;
    (void)RCC->AHB3ENR;
    IPCC->C1SCR = 0x3F;                       /* снять все флаги CPU1 */
    IPCC->C2SCR = 0x3F;
    IPCC->C1MR  = 0x3F | (0x3FU << 16);       /* все каналы замаскированы */
    IPCC->C2MR  = 0x3F | (0x3FU << 16);
}
