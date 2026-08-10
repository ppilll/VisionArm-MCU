#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>
#include <stddef.h>
#include "system_stm32f1xx.h"

void AppFatal_Assert(const char *file, int line);

#define configUSE_PREEMPTION                            1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION         1
#define configUSE_TICKLESS_IDLE                         0
#define configCPU_CLOCK_HZ    SystemCoreClock
// #define configSYSTICK_CLOCK_HZ                          (configCPU_CLOCK_HZ / 8)
#define configTICK_RATE_HZ                              1000

/* Application priorities are 0, 2, 4 and 5. */
#define configMAX_PRIORITIES                            6
#define configMINIMAL_STACK_SIZE                        128
#define configMAX_TASK_NAME_LEN                         12
#define configUSE_16_BIT_TICKS                          0
#define configIDLE_SHOULD_YIELD                         1
#define configUSE_TIME_SLICING                          1

#define configUSE_TASK_NOTIFICATIONS                    1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES           1
#define configUSE_MUTEXES                               0
#define configUSE_RECURSIVE_MUTEXES                     0
#define configUSE_COUNTING_SEMAPHORES                   1
#define configUSE_ALTERNATIVE_API                       0
#define configQUEUE_REGISTRY_SIZE                       0
#define configUSE_QUEUE_SETS                            0
#define configUSE_NEWLIB_REENTRANT                      0
#define configENABLE_BACKWARD_COMPATIBILITY             0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS         0
#define configSTACK_DEPTH_TYPE                          uint16_t
#define configMESSAGE_BUFFER_LENGTH_TYPE                size_t

/* All application RTOS objects are statically allocated. */
#define configSUPPORT_STATIC_ALLOCATION                 1
#define configSUPPORT_DYNAMIC_ALLOCATION                0
#define configAPPLICATION_ALLOCATED_HEAP                0
#define configSTACK_ALLOCATION_FROM_SEPARATE_HEAP       0

#define configUSE_IDLE_HOOK                             0
#define configUSE_TICK_HOOK                             0
#define configCHECK_FOR_STACK_OVERFLOW                  2
#define configUSE_MALLOC_FAILED_HOOK                    0
#define configUSE_DAEMON_TASK_STARTUP_HOOK              0

#define configGENERATE_RUN_TIME_STATS                   0
#define configUSE_TRACE_FACILITY                        0
#define configUSE_STATS_FORMATTING_FUNCTIONS            0

#define configUSE_CO_ROUTINES                           0
#define configMAX_CO_ROUTINE_PRIORITIES                 1

/* No software timers are used by the application. */
#define configUSE_TIMERS                                0

/* Only APIs used by this project are enabled. */
#define INCLUDE_vTaskPrioritySet                        0
#define INCLUDE_uxTaskPriorityGet                       0
#define INCLUDE_vTaskDelete                             0
#define INCLUDE_vTaskSuspend                            0
#define INCLUDE_xResumeFromISR                          0
#define INCLUDE_vTaskDelayUntil                        1
#define INCLUDE_vTaskDelay                              0
#define INCLUDE_xTaskGetSchedulerState                  1
#define INCLUDE_xTaskGetCurrentTaskHandle               1
#define INCLUDE_uxTaskGetStackHighWaterMark             0
#define INCLUDE_xTaskGetIdleTaskHandle                  0
#define INCLUDE_eTaskGetState                           0
#define INCLUDE_xEventGroupSetBitFromISR                0
#define INCLUDE_xTimerPendFunctionCall                  0
#define INCLUDE_xTaskAbortDelay                         0
#define INCLUDE_xTaskGetHandle                          0
#define INCLUDE_xTaskResumeFromISR                      0

#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS __NVIC_PRIO_BITS
#else
#define configPRIO_BITS 4
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5

#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_API_CALL_INTERRUPT_PRIORITY \
    configMAX_SYSCALL_INTERRUPT_PRIORITY

#define xPortPendSVHandler                              PendSV_Handler
#define vPortSVCHandler                                 SVC_Handler

#define configASSERT(x) \
    do \
    { \
        if ((x) == 0) \
        { \
            AppFatal_Assert(__FILE__, __LINE__); \
        } \
    } while (0)

#endif /* FREERTOS_CONFIG_H */
