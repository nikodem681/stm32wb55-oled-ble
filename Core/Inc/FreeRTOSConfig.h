#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Этот файл настраивает ядро FreeRTOS под наш чип и наши нужды. */

#include <stdint.h>
extern uint32_t SystemCoreClock;

#define configUSE_PREEMPTION                    1
#define configCPU_CLOCK_HZ                      (SystemCoreClock)
#define configTICK_RATE_HZ                      ((TickType_t)1000)   /* 1 тик = 1 мс */
#define configMAX_PRIORITIES                    5
#define configMINIMAL_STACK_SIZE                ((uint16_t)128)     /* в словах (по 4 байта) */
#define configTOTAL_HEAP_SIZE                   ((size_t)(32 * 1024))
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               0
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_MALLOC_FAILED_HOOK            1
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_TRACE_FACILITY                1   /* нужно для uxTaskGetSystemState() */

/* Статистика: сколько времени проработала каждая задача.
   Часы — счётчик тактов ядра DWT CYCCNT, поделённый до 1 МГц (см. stats.c). */
#define configGENERATE_RUN_TIME_STATS           1
void     stats_timer_init(void);
uint32_t stats_timer_value(void);
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  stats_timer_init()
#define portGET_RUN_TIME_COUNTER_VALUE()          stats_timer_value()
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configSUPPORT_STATIC_ALLOCATION         0

#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               2
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            256

#define INCLUDE_vTaskDelay                      1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_xTaskGetTickCount               1
#define INCLUDE_uxTaskGetStackHighWaterMark     1

/* Приоритеты прерываний Cortex-M4: у STM32WB 4 бита приоритета (0..15). */
#define configPRIO_BITS                                 4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5
#define configKERNEL_INTERRUPT_PRIORITY      (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* Если assert сработал — останавливаемся, в отладчике будет видно где. */
#define configASSERT(x) if ((x) == 0) { __asm volatile("cpsid i"); for (;;); }

/* Связываем обработчики FreeRTOS с именами векторов из startup-файла. */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#endif
