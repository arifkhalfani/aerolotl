#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* ------------------------------------------------------------------------
 * This is a minimal config for the FreeRTOS POSIX simulator port.
 * It is NOT the config you'll use on the STM32H755 later - that one
 * will set configCPU_CLOCK_HZ to the real core clock, use the Cortex-M
 * port instead of the POSIX port, and enable an MPU/FPU if needed.
 * Keep this file small; only add options here when you actually need them.
 * ------------------------------------------------------------------------ */

#include <stdint.h>
extern uint32_t SystemCoreClock; /* unused on POSIX, some headers expect it */

/* --- Scheduling --- */
#define configUSE_PREEMPTION                   1
#define configUSE_TIME_SLICING                 1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      ( ( unsigned long ) 1000000 )
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                    ( 8 )
#define configMINIMAL_STACK_SIZE                ( ( unsigned short ) 4096 )
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1

/* --- Memory allocation --- */
/* heap_3 wraps malloc/free - simplest option for a laptop sim.
   On the STM32 you'll switch to heap_4 (real allocator, no OS backing it). */
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configTOTAL_HEAP_SIZE                    ( ( size_t ) ( 1024 * 1024 ) )
#define configAPPLICATION_ALLOCATED_HEAP         0

/* --- Synchronization primitives --- */
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_COUNTING_SEMAPHORES            1
#define configQUEUE_REGISTRY_SIZE                8
#define configUSE_QUEUE_SETS                     0

/* --- Hooks --- */
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configCHECK_FOR_STACK_OVERFLOW            2
#define configUSE_MALLOC_FAILED_HOOK              1

/* --- Runtime stats (off for now, turn on later if you want CPU-usage numbers) --- */
#define configGENERATE_RUN_TIME_STATS             0
#define configUSE_TRACE_FACILITY                   1
#define configUSE_STATS_FORMATTING_FUNCTIONS       0

/* --- Software timers --- */
#define configUSE_TIMERS                           1
#define configTIMER_TASK_PRIORITY                  ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                   10
#define configTIMER_TASK_STACK_DEPTH                configMINIMAL_STACK_SIZE

/* --- Optional API inclusions --- */
#define INCLUDE_vTaskPrioritySet                    1
#define INCLUDE_uxTaskPriorityGet                   1
#define INCLUDE_vTaskDelete                         1
#define INCLUDE_vTaskSuspend                        1
#define INCLUDE_vTaskDelayUntil                     1
#define INCLUDE_vTaskDelay                          1
#define INCLUDE_xTaskGetSchedulerState               1
#define INCLUDE_xTaskGetCurrentTaskHandle            1
#define INCLUDE_uxTaskGetStackHighWaterMark          1
#define INCLUDE_eTaskGetState                        1

/* --- POSIX port specifics --- */
/* Needed by the simulator port to know how many "interrupt priorities" exist. */
#define configKERNEL_INTERRUPT_PRIORITY             255
#define configMAX_SYSCALL_INTERRUPT_PRIORITY        191

#endif /* FREERTOS_CONFIG_H */