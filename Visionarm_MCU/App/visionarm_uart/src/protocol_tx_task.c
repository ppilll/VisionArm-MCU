#include "protocol_tx_task.h"

#include <string.h>

#include "semphr.h"
#include "stm32f1xx_hal.h"

#include "app_config.h"
#include "protocol_engine.h"
#include "protocol_state.h"
#include "rs485_uart.h"
#include "visionarm_uart_c/protocol.h"

#define APP_TX_MAX_ENCODED_RESPONSE 142U

typedef enum
{
    TX_PRIORITY_ACK_NACK = 0,
    TX_PRIORITY_HELLO_ACK = 1,
    TX_PRIORITY_PONG = 2,
    TX_PRIORITY_STATUS = 3,
    TX_PRIORITY_DIAGNOSTIC = 4 /* Reserved: no wire diagnostic type exists yet. */
} TxPriority;

typedef struct
{
    uint8_t message_type;
    uint8_t payload_size;
    uint8_t payload[APP_TX_MAX_RESPONSE_PAYLOAD];
} TxRequest;

typedef struct
{
    bool used;
    uint8_t priority;
    uint32_t enqueue_order;
    TxRequest request;
} TxSlot;

static TaskHandle_t s_tx_task;
static StaticTask_t s_tx_task_tcb;
static StackType_t s_tx_task_stack[APP_TX_TASK_STACK_WORDS];

static StaticSemaphore_t s_free_slot_sem_storage;
static StaticSemaphore_t s_pending_sem_storage;
static SemaphoreHandle_t s_free_slot_sem;
static SemaphoreHandle_t s_pending_sem;

static TxSlot s_slots[APP_TX_REQUEST_SLOTS];
static uint32_t s_next_enqueue_order = 1U;
static uint32_t s_next_wire_sequence = 1U;

static void ProtocolTxTask_Entry(void *argument);
static bool ClassifyPriority(uint8_t message_type, uint8_t *priority);
static int32_t FindFreeSlot(void);
static bool TakeHighestPriorityRequest(TxRequest *request);

bool ProtocolTxTask_Create(void)
{
    if ((s_tx_task != NULL) ||
        (s_free_slot_sem != NULL) ||
        (s_pending_sem != NULL))
    {
        return false;
    }

    (void)memset(s_slots, 0, sizeof(s_slots));

    s_free_slot_sem = xSemaphoreCreateCountingStatic(
        APP_TX_REQUEST_SLOTS,
        APP_TX_REQUEST_SLOTS,
        &s_free_slot_sem_storage);

    s_pending_sem = xSemaphoreCreateCountingStatic(
        APP_TX_REQUEST_SLOTS,
        0U,
        &s_pending_sem_storage);

    if ((s_free_slot_sem == NULL) || (s_pending_sem == NULL))
    {
        return false;
    }

    s_tx_task = xTaskCreateStatic(ProtocolTxTask_Entry,
                                  "ProtocolTx",
                                  APP_TX_TASK_STACK_WORDS,
                                  NULL,
                                  APP_TX_TASK_PRIORITY,
                                  s_tx_task_stack,
                                  &s_tx_task_tcb);

    return (s_tx_task != NULL);
}

bool ProtocolTxTask_EnqueueResponse(uint8_t message_type,
                                    const uint8_t *payload,
                                    size_t payload_size,
                                    TickType_t ticks_to_wait)
{
    uint8_t priority;
    int32_t slot_index;
    TxSlot *slot;

    if ((s_free_slot_sem == NULL) ||
        (s_pending_sem == NULL) ||
        (payload_size > APP_TX_MAX_RESPONSE_PAYLOAD) ||
        ((payload_size > 0U) && (payload == NULL)) ||
        !ClassifyPriority(message_type, &priority))
    {
        return false;
    }

    if (xSemaphoreTake(s_free_slot_sem, ticks_to_wait) != pdTRUE)
    {
        return false;
    }

    taskENTER_CRITICAL();
    slot_index = FindFreeSlot();

    if (slot_index >= 0)
    {
        slot = &s_slots[(uint32_t)slot_index];
        (void)memset(slot, 0, sizeof(*slot));
        slot->priority = priority;
        slot->enqueue_order = s_next_enqueue_order++;
        slot->request.message_type = message_type;
        slot->request.payload_size = (uint8_t)payload_size;
        if (payload_size > 0U)
        {
            (void)memcpy(slot->request.payload, payload, payload_size);
        }
        slot->used = true;
    }

    taskEXIT_CRITICAL();

    if (slot_index < 0)
    {
        (void)xSemaphoreGive(s_free_slot_sem);
        return false;
    }

    if (xSemaphoreGive(s_pending_sem) != pdTRUE)
    {
        taskENTER_CRITICAL();
        s_slots[(uint32_t)slot_index].used = false;
        taskEXIT_CRITICAL();
        (void)xSemaphoreGive(s_free_slot_sem);
        return false;
    }

    return true;
}

void ProtocolTxTask_NotifyTxCompleteFromISR(BaseType_t *higher_priority_task_woken)
{
    if (s_tx_task != NULL)
    {
        vTaskNotifyGiveFromISR(s_tx_task, higher_priority_task_woken);
    }
}

static void ProtocolTxTask_Entry(void *argument)
{
    TxRequest request;
    va_uart_header_t header;
    uint8_t encoded[APP_TX_MAX_ENCODED_RESPONSE];
    size_t encoded_size;

    (void)argument;

    for (;;)
    {
        if (xSemaphoreTake(s_pending_sem, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        if (!TakeHighestPriorityRequest(&request))
        {
            continue;
        }

        (void)memset(&header, 0, sizeof(header));
        header.protocol_version = VA_UART_PROTOCOL_VERSION;
        header.message_type = request.message_type;
        header.payload_length = request.payload_size;
        header.wire_sequence = s_next_wire_sequence;
        header.sender_boot_id = ProtocolState_GetMcuBootId();
        header.sender_uptime_ms = HAL_GetTick();

        if (!ProtocolEngine_EncodeFrame(&header,
                                        request.payload,
                                        request.payload_size,
                                        encoded,
                                        sizeof(encoded),
                                        &encoded_size))
        {
            continue;
        }

        s_next_wire_sequence++;
        (void)ulTaskNotifyTake(pdTRUE, 0U);

        if (!Rs485Uart_StartTransmit(encoded, encoded_size))
        {
            continue;
        }

        if (ulTaskNotifyTake(pdTRUE,
                             pdMS_TO_TICKS(APP_TX_COMPLETE_TIMEOUT_MS)) == 0U)
        {
            Rs485Uart_AbortTransmit();
        }
    }
}

static bool ClassifyPriority(uint8_t message_type, uint8_t *priority)
{
    if (priority == NULL)
    {
        return false;
    }

    switch ((va_uart_message_type_t)message_type)
    {
        case VA_UART_MSG_ACK:
        case VA_UART_MSG_NACK:
            *priority = TX_PRIORITY_ACK_NACK;
            return true;

        case VA_UART_MSG_HELLO_ACK:
            *priority = TX_PRIORITY_HELLO_ACK;
            return true;

        case VA_UART_MSG_PONG:
            *priority = TX_PRIORITY_PONG;
            return true;

        case VA_UART_MSG_STATUS:
            *priority = TX_PRIORITY_STATUS;
            return true;

        default:
            /* Enforces "MCU responses only" at the TX boundary. */
            return false;
    }
}

static int32_t FindFreeSlot(void)
{
    uint32_t i;

    for (i = 0U; i < APP_TX_REQUEST_SLOTS; ++i)
    {
        if (!s_slots[i].used)
        {
            return (int32_t)i;
        }
    }

    return -1;
}

static bool TakeHighestPriorityRequest(TxRequest *request)
{
    int32_t best_index = -1;
    uint8_t best_priority = 0xFFU;
    uint32_t best_order = 0U;
    uint32_t i;

    if (request == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();

    for (i = 0U; i < APP_TX_REQUEST_SLOTS; ++i)
    {
        if (!s_slots[i].used)
        {
            continue;
        }

        if ((best_index < 0) ||
            (s_slots[i].priority < best_priority) ||
            ((s_slots[i].priority == best_priority) &&
             ((int32_t)(s_slots[i].enqueue_order - best_order) < 0)))
        {
            best_index = (int32_t)i;
            best_priority = s_slots[i].priority;
            best_order = s_slots[i].enqueue_order;
        }
    }

    if (best_index >= 0)
    {
        *request = s_slots[(uint32_t)best_index].request;
        s_slots[(uint32_t)best_index].used = false;
    }

    taskEXIT_CRITICAL();

    if (best_index < 0)
    {
        return false;
    }

    (void)xSemaphoreGive(s_free_slot_sem);
    return true;
}
