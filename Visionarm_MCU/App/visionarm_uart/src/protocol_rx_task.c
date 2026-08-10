#include "protocol_rx_task.h"

#include <stddef.h>
#include <stdint.h>

#include "app_config.h"
#include "protocol_engine.h"
#include "protocol_watchdog.h"
#include "rs485_uart.h"
#include "stm32f1xx_hal.h"
#include "uart_rx_ring.h"

static TaskHandle_t s_rx_task;
static StaticTask_t s_rx_task_tcb;
static StackType_t s_rx_task_stack[APP_RX_TASK_STACK_WORDS];

static void ProtocolRxTask_Entry(void *argument);
static void HandleResync(uint32_t *last_resync_epoch);

bool ProtocolRxTask_Create(void)
{
    if (s_rx_task != NULL)
    {
        return false;
    }

    s_rx_task = xTaskCreateStatic(ProtocolRxTask_Entry,
                                  "ProtocolRx",
                                  APP_RX_TASK_STACK_WORDS,
                                  NULL,
                                  APP_RX_TASK_PRIORITY,
                                  s_rx_task_stack,
                                  &s_rx_task_tcb);

    return (s_rx_task != NULL);
}

void ProtocolRxTask_NotifyFromISR(BaseType_t *higher_priority_task_woken)
{
    if (s_rx_task != NULL)
    {
        vTaskNotifyGiveFromISR(s_rx_task, higher_priority_task_woken);
    }
}

static void ProtocolRxTask_Entry(void *argument)
{
    uint8_t chunk[APP_RX_TASK_CHUNK_SIZE];
    size_t count;
    uint32_t last_resync_epoch;

    (void)argument;

    last_resync_epoch = UartRxRing_GetResyncEpoch();
    Rs485Uart_StartReceive();

    for (;;)
    {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(APP_RX_POLL_MS));

        HandleResync(&last_resync_epoch);

        do
        {
            count = UartRxRing_Read(chunk, sizeof(chunk));
            if (count > 0U)
            {
                HandleResync(&last_resync_epoch);
                ProtocolEngine_Feed(chunk, count, HAL_GetTick());
            }
        }
        while (count > 0U);

        ProtocolEngine_Tick(HAL_GetTick());
        HandleResync(&last_resync_epoch);
        ProtocolWatchdog_Check();
    }
}

static void HandleResync(uint32_t *last_resync_epoch)
{
    uint32_t current_epoch;

    if (last_resync_epoch == NULL)
    {
        return;
    }

    current_epoch = UartRxRing_GetResyncEpoch();
    if (current_epoch != *last_resync_epoch)
    {
        *last_resync_epoch = current_epoch;
        ProtocolEngine_ForceResync();
    }
}
