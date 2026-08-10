#include "rs485_uart.h"

#include "FreeRTOS.h"
#include "task.h"

#include "protocol_rx_task.h"
#include "protocol_tx_task.h"
#include "uart_rx_ring.h"

void Rs485Uart_OnRxByteFromISR(uint8_t byte)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    bool should_wake_task = false;

    if (!UartRxRing_PushFromISR(byte, &should_wake_task))
    {
        should_wake_task = true;
    }

    if (should_wake_task)
    {
        ProtocolRxTask_NotifyFromISR(&higher_priority_task_woken);
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void Rs485Uart_OnTxCompleteFromISR(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    ProtocolTxTask_NotifyTxCompleteFromISR(&higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void Rs485Uart_OnErrorFromISR(uint32_t hal_error_flags)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    (void)hal_error_flags;

    /* A receive error can invalidate the byte stream boundary. */
    UartRxRing_MarkResyncFromISR();
    ProtocolRxTask_NotifyFromISR(&higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}
