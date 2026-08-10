#ifndef VISIONARM_APP_CONFIG_H
#define VISIONARM_APP_CONFIG_H

#include <stdint.h>


/* USART2 / RS-485 */
#define APP_UART_BAUDRATE                    115200U
#define APP_UART_IRQ_PREEMPT_PRIORITY        6U
#define APP_UART_IRQ_SUBPRIORITY             0U
#define APP_RS485_PRE_TX_GUARD_US            0U
#define APP_RS485_POST_TX_GUARD_US           0U

/* RX transport and parser */
#define APP_PROTOCOL_MAX_ENCODED_FRAME      294U
#define APP_RX_RING_CAPACITY                 1024U
#define APP_RX_TASK_CHUNK_SIZE               64U
#define APP_PARSER_ASSEMBLY_TIMEOUT_MS       100U
#define APP_RX_POLL_MS                       10U

/* RTOS task layout */
#define APP_RX_TASK_PRIORITY                 5U
#define APP_TX_TASK_PRIORITY                 4U
#define APP_GIMBAL_TASK_PRIORITY             2U

#define APP_RX_TASK_STACK_WORDS              384U
#define APP_TX_TASK_STACK_WORDS              320U
#define APP_GIMBAL_TASK_STACK_WORDS          256U

/* TX scheduling */
#define APP_TX_REQUEST_SLOTS                 6U
#define APP_TX_MAX_RESPONSE_PAYLOAD          52U
#define APP_TX_COMPLETE_TIMEOUT_MS           100U
#define APP_RESPONSE_ENQUEUE_WAIT_MS         20U

/* Watchdogs / control consumer */
#define APP_LINK_WATCHDOG_TIMEOUT_MS         1000U
#define APP_CONTROL_WATCHDOG_TIMEOUT_MS      200U
#define APP_GIMBAL_TASK_PERIOD_MS            20U

#if (APP_UART_IRQ_PREEMPT_PRIORITY < 5U)
#error "USART2 IRQ priority is too urgent for FreeRTOS FromISR use"
#endif

#if (APP_RX_RING_CAPACITY < (2U * APP_PROTOCOL_MAX_ENCODED_FRAME))
#error "RX ring must hold at least two maximum encoded frames"
#endif

#if ((APP_RX_RING_CAPACITY & (APP_RX_RING_CAPACITY - 1U)) != 0U)
#error "RX ring capacity must be a power of two"
#endif

#if (APP_TX_REQUEST_SLOTS == 0U)
#error "At least one TX response slot is required"
#endif

#if (APP_TX_MAX_RESPONSE_PAYLOAD < 52U)
#error "TX response storage must hold STATUS payload"
#endif

#if (APP_CONTROL_WATCHDOG_TIMEOUT_MS >= APP_LINK_WATCHDOG_TIMEOUT_MS)
#error "Control watchdog must expire before link watchdog"
#endif

#endif /* VISIONARM_APP_CONFIG_H */
