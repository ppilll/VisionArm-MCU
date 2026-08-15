#include "protocol_transport.h"

#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "stm32f1xx_hal.h"

#include "protocol_core.h"
#include "visionarm_app.h"
#include "visionarm_uart_c/frame_parser.h"
#include "visionarm_uart_c/protocol.h"

#define RX_RING_MASK                         (APP_RX_RING_CAPACITY - 1U)
#define TX_MAX_ENCODED_RESPONSE              142U
#define RS485_DIRECTION_GPIO_PORT            GPIOD
#define RS485_DIRECTION_GPIO_PIN             GPIO_PIN_7
#define RS485_UART_RX_ERROR_SR_MASK           \
    (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)

typedef enum
{
    TX_PRIORITY_ACK_NACK = 0,
    TX_PRIORITY_HELLO_ACK = 1,
    TX_PRIORITY_PONG = 2,
    TX_PRIORITY_STATUS = 3
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

static UART_HandleTypeDef s_huart2;
static bool s_uart_initialized;
static volatile bool s_tx_busy;

static uint8_t s_rx_ring[APP_RX_RING_CAPACITY];
static volatile uint32_t s_rx_write_sequence;
static volatile uint32_t s_rx_read_sequence;
static volatile uint32_t s_rx_resync_epoch;
static volatile uint32_t s_rx_overflow_count;

static va_uart_frame_parser_t s_parser;
static ProtocolRxStats s_accumulated_stats;

static TaskHandle_t s_rx_task;
static StaticTask_t s_rx_task_tcb;
static StackType_t s_rx_task_stack[APP_RX_TASK_STACK_WORDS];

static TaskHandle_t s_tx_task;
static StaticTask_t s_tx_task_tcb;
static StackType_t s_tx_task_stack[APP_TX_TASK_STACK_WORDS];

static StaticSemaphore_t s_free_slot_sem_storage;
static StaticSemaphore_t s_pending_sem_storage;
static SemaphoreHandle_t s_free_slot_sem;
static SemaphoreHandle_t s_pending_sem;
static TxSlot s_tx_slots[APP_TX_REQUEST_SLOTS];
static uint32_t s_next_enqueue_order = 1U;
static uint32_t s_next_wire_sequence = 1U;

static bool UartInit(void);
static void UartStartReceive(void);
static bool UartStartTransmit(const uint8_t *data, size_t length);
static void UartAbortTransmit(void);
static void UartEnterReceive(void);
static void UartEnterTransmit(void);
static uint32_t UartSrErrorsToHalErrors(uint32_t status_register);
static void DwtInit(void);
static void DelayUs(uint32_t delay_us);

static void RingInit(void);
static bool RingPushFromISR(uint8_t byte, bool *should_wake_task);
static void RingMarkResyncFromISR(void);
static size_t RingRead(uint8_t *output, size_t output_capacity);
static uint32_t RingGetResyncEpoch(void);

static void ParserInit(void);
static void ParserOnFrame(const va_uart_frame_t *frame, void *context);
static void ParserFeed(const uint8_t *data, size_t length, uint32_t now_ms);
static void ParserTick(uint32_t now_ms);
static void ParserForceResync(void);
static void ParserAccumulateCurrentStats(void);

static bool CreateTxTask(void);
static bool CreateRxTask(void);
static void RxTaskEntry(void *argument);
static void TxTaskEntry(void *argument);
static void HandleResync(uint32_t *last_resync_epoch);
static void NotifyRxTaskFromISR(BaseType_t *higher_priority_task_woken);
static void NotifyTxTaskFromISR(BaseType_t *higher_priority_task_woken);

static bool ClassifyTxPriority(uint8_t message_type, uint8_t *priority);
static int32_t FindFreeTxSlot(void);
static bool TakeHighestPriorityRequest(TxRequest *request);

bool ProtocolTransport_Init(void)
{
    RingInit();
    ParserInit();
    return UartInit();
}

bool ProtocolTransport_CreateTasks(void)
{
    /* TX must exist before RX can dispatch responses. */
    return CreateTxTask() && CreateRxTask();
}

bool ProtocolTransport_QueueResponse(uint8_t message_type,
                                     const uint8_t *payload,
                                     size_t payload_size)
{
    uint8_t priority;
    int32_t slot_index;
    TxSlot *slot;

    if ((s_free_slot_sem == NULL) ||
        (s_pending_sem == NULL) ||
        (payload_size > APP_TX_MAX_RESPONSE_PAYLOAD) ||
        ((payload_size > 0U) && (payload == NULL)) ||
        !ClassifyTxPriority(message_type, &priority))
    {
        return false;
    }

    if (xSemaphoreTake(
            s_free_slot_sem,
            pdMS_TO_TICKS(APP_RESPONSE_ENQUEUE_WAIT_MS)) != pdTRUE)
    {
        return false;
    }

    taskENTER_CRITICAL();
    slot_index = FindFreeTxSlot();

    if (slot_index >= 0)
    {
        slot = &s_tx_slots[(uint32_t)slot_index];
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
        s_tx_slots[(uint32_t)slot_index].used = false;
        taskEXIT_CRITICAL();
        (void)xSemaphoreGive(s_free_slot_sem);
        return false;
    }

    return true;
}

void ProtocolTransport_GetRxStats(ProtocolRxStats *stats)
{
    if (stats == NULL)
    {
        return;
    }

    stats->valid_frame_count =
        s_accumulated_stats.valid_frame_count + s_parser.stats.valid_frames;
    stats->crc_error_count =
        s_accumulated_stats.crc_error_count + s_parser.stats.crc_errors;
    stats->length_error_count =
        s_accumulated_stats.length_error_count + s_parser.stats.length_errors;
    stats->version_error_count =
        s_accumulated_stats.version_error_count + s_parser.stats.version_errors;
    stats->unknown_type_count =
        s_accumulated_stats.unknown_type_count +
        s_parser.stats.unknown_type_errors;
}

uint32_t ProtocolTransport_GetRxOverflowCount(void)
{
    return s_rx_overflow_count;
}

void ProtocolTransport_UartIrqHandler(void)
{
    uint32_t status_register;
    uint32_t control_register_1;
    uint32_t control_register_3;
    uint32_t received_data;
    uint32_t hal_errors;
    bool rx_interrupt_pending;
    bool error_interrupt_pending;
    BaseType_t higher_priority_task_woken = pdFALSE;
    bool should_wake_task = false;

    status_register = READ_REG(s_huart2.Instance->SR);
    control_register_1 = READ_REG(s_huart2.Instance->CR1);
    control_register_3 = READ_REG(s_huart2.Instance->CR3);

    rx_interrupt_pending =
        (((status_register & USART_SR_RXNE) != 0U) &&
         ((control_register_1 & USART_CR1_RXNEIE) != 0U));

    error_interrupt_pending =
        (((status_register & RS485_UART_RX_ERROR_SR_MASK) != 0U) &&
         (((control_register_3 & USART_CR3_EIE) != 0U) ||
          ((control_register_1 & USART_CR1_RXNEIE) != 0U)));

    if (rx_interrupt_pending || error_interrupt_pending)
    {
        /* STM32F1 clears RXNE/PE/FE/NE/ORE by SR read followed by DR read. */
        received_data = READ_REG(s_huart2.Instance->DR);
        hal_errors = UartSrErrorsToHalErrors(status_register);

        if (hal_errors != HAL_UART_ERROR_NONE)
        {
            RingMarkResyncFromISR();
            NotifyRxTaskFromISR(&higher_priority_task_woken);
        }
        else if (rx_interrupt_pending)
        {
            if (!RingPushFromISR((uint8_t)(received_data & 0xFFU),
                                 &should_wake_task))
            {
                should_wake_task = true;
            }

            if (should_wake_task)
            {
                NotifyRxTaskFromISR(&higher_priority_task_woken);
            }
        }
    }

    /* HAL remains owner of the TXE -> TC interrupt transmit state machine. */
    HAL_UART_IRQHandler(&s_huart2);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio = {0};

    if ((huart == NULL) || (huart->Instance != USART2))
    {
        return;
    }

    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_AFIO_REMAP_USART2_DISABLE();

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

    gpio.Pin = GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &gpio);

    gpio.Pin = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_NVIC_SetPriority(USART2_IRQn,
                         APP_UART_IRQ_PREEMPT_PRIORITY,
                         APP_UART_IRQ_SUBPRIORITY);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART2))
    {
        return;
    }

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if ((huart == NULL) || (huart->Instance != USART2))
    {
        return;
    }

    DelayUs(APP_RS485_POST_TX_GUARD_US);
    UartEnterReceive();
    s_tx_busy = false;
    NotifyTxTaskFromISR(&higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if ((huart == NULL) || (huart->Instance != USART2))
    {
        return;
    }

    if (huart->gState != HAL_UART_STATE_BUSY_TX)
    {
        UartEnterReceive();
    }

    RingMarkResyncFromISR();
    NotifyRxTaskFromISR(&higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static bool UartInit(void)
{
    HAL_StatusTypeDef status;

    if (s_uart_initialized)
    {
        UartEnterReceive();
        return true;
    }

    s_huart2 = (UART_HandleTypeDef){0};
    s_huart2.Instance = USART2;
    s_huart2.Init.BaudRate = APP_UART_BAUDRATE;
    s_huart2.Init.WordLength = UART_WORDLENGTH_8B;
    s_huart2.Init.StopBits = UART_STOPBITS_1;
    s_huart2.Init.Parity = UART_PARITY_NONE;
    s_huart2.Init.Mode = UART_MODE_TX_RX;
    s_huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
#if defined(USART_CR1_OVER8)
    s_huart2.Init.OverSampling = UART_OVERSAMPLING_16;
#endif

    status = HAL_UART_Init(&s_huart2);
    if (status != HAL_OK)
    {
        UartEnterReceive();
        return false;
    }

    DwtInit();
    s_tx_busy = false;
    s_uart_initialized = true;
    UartEnterReceive();
    return true;
}

static void UartStartReceive(void)
{
    if (!s_uart_initialized)
    {
        return;
    }

    UartEnterReceive();
    __HAL_UART_CLEAR_OREFLAG(&s_huart2);
    __HAL_UART_ENABLE_IT(&s_huart2, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&s_huart2, UART_IT_ERR);
}

static bool UartStartTransmit(const uint8_t *data, size_t length)
{
    HAL_StatusTypeDef status;

    if (!s_uart_initialized ||
        (data == NULL) ||
        (length == 0U) ||
        (length > (size_t)UINT16_MAX) ||
        s_tx_busy)
    {
        return false;
    }

    s_tx_busy = true;
    UartEnterTransmit();
    DelayUs(APP_RS485_PRE_TX_GUARD_US);

    status = HAL_UART_Transmit_IT(&s_huart2,
                                  (uint8_t *)(uintptr_t)data,
                                  (uint16_t)length);
    if (status != HAL_OK)
    {
        s_tx_busy = false;
        UartEnterReceive();
        return false;
    }

    return true;
}

static void UartAbortTransmit(void)
{
    if (!s_uart_initialized)
    {
        return;
    }

    (void)HAL_UART_AbortTransmit(&s_huart2);
    s_tx_busy = false;
    UartEnterReceive();
}

static void UartEnterReceive(void)
{
    HAL_GPIO_WritePin(RS485_DIRECTION_GPIO_PORT,
                      RS485_DIRECTION_GPIO_PIN,
                      GPIO_PIN_RESET);
}

static void UartEnterTransmit(void)
{
    HAL_GPIO_WritePin(RS485_DIRECTION_GPIO_PORT,
                      RS485_DIRECTION_GPIO_PIN,
                      GPIO_PIN_SET);
}

static uint32_t UartSrErrorsToHalErrors(uint32_t status_register)
{
    uint32_t hal_errors = HAL_UART_ERROR_NONE;

    if ((status_register & USART_SR_PE) != 0U)
    {
        hal_errors |= HAL_UART_ERROR_PE;
    }
    if ((status_register & USART_SR_NE) != 0U)
    {
        hal_errors |= HAL_UART_ERROR_NE;
    }
    if ((status_register & USART_SR_FE) != 0U)
    {
        hal_errors |= HAL_UART_ERROR_FE;
    }
    if ((status_register & USART_SR_ORE) != 0U)
    {
        hal_errors |= HAL_UART_ERROR_ORE;
    }

    return hal_errors;
}

static void DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void DelayUs(uint32_t delay_us)
{
    uint32_t cycles_per_us;
    uint32_t target_cycles;
    uint32_t start_cycles;

    if (delay_us == 0U)
    {
        return;
    }

    cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U)
    {
        return;
    }

    if (delay_us > (UINT32_MAX / cycles_per_us))
    {
        delay_us = UINT32_MAX / cycles_per_us;
    }

    target_cycles = delay_us * cycles_per_us;
    start_cycles = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start_cycles) < target_cycles)
    {
        __NOP();
    }
}

static void RingInit(void)
{
    s_rx_write_sequence = 0U;
    s_rx_read_sequence = 0U;
    s_rx_resync_epoch = 0U;
    s_rx_overflow_count = 0U;
}

static bool RingPushFromISR(uint8_t byte, bool *should_wake_task)
{
    uint32_t write_sequence = s_rx_write_sequence;
    uint32_t read_sequence = s_rx_read_sequence;

    if (should_wake_task != NULL)
    {
        *should_wake_task = (write_sequence == read_sequence);
    }

    if ((uint32_t)(write_sequence - read_sequence) >= APP_RX_RING_CAPACITY)
    {
        s_rx_overflow_count++;
        s_rx_resync_epoch++;
        return false;
    }

    s_rx_ring[write_sequence & RX_RING_MASK] = byte;
    __DMB();
    s_rx_write_sequence = write_sequence + 1U;
    return true;
}

static void RingMarkResyncFromISR(void)
{
    s_rx_resync_epoch++;
}

static size_t RingRead(uint8_t *output, size_t output_capacity)
{
    uint32_t read_sequence;
    uint32_t available;
    size_t count;
    size_t i;

    if ((output == NULL) || (output_capacity == 0U))
    {
        return 0U;
    }

    read_sequence = s_rx_read_sequence;
    available = (uint32_t)(s_rx_write_sequence - read_sequence);
    count = (available > output_capacity) ? output_capacity : (size_t)available;

    for (i = 0U; i < count; ++i)
    {
        output[i] = s_rx_ring[(read_sequence + (uint32_t)i) & RX_RING_MASK];
    }

    __DMB();
    s_rx_read_sequence = read_sequence + (uint32_t)count;
    return count;
}

static uint32_t RingGetResyncEpoch(void)
{
    return s_rx_resync_epoch;
}

static void ParserInit(void)
{
    (void)memset(&s_accumulated_stats, 0, sizeof(s_accumulated_stats));
    va_uart_frame_parser_init(&s_parser,
                              APP_PARSER_ASSEMBLY_TIMEOUT_MS,
                              ParserOnFrame,
                              NULL);
}

static void ParserOnFrame(const va_uart_frame_t *frame, void *context)
{
    (void)context;
    ProtocolCore_OnFrame(frame);
}

static void ParserFeed(const uint8_t *data, size_t length, uint32_t now_ms)
{
    if ((data != NULL) && (length > 0U))
    {
        va_uart_frame_parser_feed(&s_parser, data, length, now_ms);
    }
}

static void ParserTick(uint32_t now_ms)
{
    va_uart_frame_parser_tick(&s_parser, now_ms);
}

static void ParserForceResync(void)
{
    ParserAccumulateCurrentStats();
    va_uart_frame_parser_reset(&s_parser);
}

static void ParserAccumulateCurrentStats(void)
{
    s_accumulated_stats.valid_frame_count += s_parser.stats.valid_frames;
    s_accumulated_stats.crc_error_count += s_parser.stats.crc_errors;
    s_accumulated_stats.length_error_count += s_parser.stats.length_errors;
    s_accumulated_stats.version_error_count += s_parser.stats.version_errors;
    s_accumulated_stats.unknown_type_count += s_parser.stats.unknown_type_errors;
}

static bool CreateRxTask(void)
{
    if (s_rx_task != NULL)
    {
        return false;
    }

    s_rx_task = xTaskCreateStatic(RxTaskEntry,
                                  "ProtocolRx",
                                  APP_RX_TASK_STACK_WORDS,
                                  NULL,
                                  APP_RX_TASK_PRIORITY,
                                  s_rx_task_stack,
                                  &s_rx_task_tcb);
    return (s_rx_task != NULL);
}

static bool CreateTxTask(void)
{
    if ((s_tx_task != NULL) ||
        (s_free_slot_sem != NULL) ||
        (s_pending_sem != NULL))
    {
        return false;
    }

    (void)memset(s_tx_slots, 0, sizeof(s_tx_slots));

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

    s_tx_task = xTaskCreateStatic(TxTaskEntry,
                                  "ProtocolTx",
                                  APP_TX_TASK_STACK_WORDS,
                                  NULL,
                                  APP_TX_TASK_PRIORITY,
                                  s_tx_task_stack,
                                  &s_tx_task_tcb);
    return (s_tx_task != NULL);
}

static void RxTaskEntry(void *argument)
{
    uint8_t chunk[APP_RX_TASK_CHUNK_SIZE];
    size_t count;
    uint32_t last_resync_epoch;

    (void)argument;
    last_resync_epoch = RingGetResyncEpoch();
    UartStartReceive();

    for (;;)
    {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(APP_RX_POLL_MS));
        HandleResync(&last_resync_epoch);

        do
        {
            count = RingRead(chunk, sizeof(chunk));
            if (count > 0U)
            {
                HandleResync(&last_resync_epoch);
                ParserFeed(chunk, count, HAL_GetTick());
            }
        }
        while (count > 0U);

        ParserTick(HAL_GetTick());
        HandleResync(&last_resync_epoch);
        ProtocolCore_Tick();
    }
}

static void TxTaskEntry(void *argument)
{
    TxRequest request;
    va_uart_header_t header;
    uint8_t encoded[TX_MAX_ENCODED_RESPONSE];
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
        header.sender_boot_id = ProtocolCore_GetMcuBootId();
        header.sender_uptime_ms = HAL_GetTick();

        if (!va_uart_encode_frame(&header,
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

        if (!UartStartTransmit(encoded, encoded_size))
        {
            continue;
        }

        if (ulTaskNotifyTake(
                pdTRUE,
                pdMS_TO_TICKS(APP_TX_COMPLETE_TIMEOUT_MS)) == 0U)
        {
            UartAbortTransmit();
        }
    }
}

static void HandleResync(uint32_t *last_resync_epoch)
{
    uint32_t current_epoch;

    if (last_resync_epoch == NULL)
    {
        return;
    }

    current_epoch = RingGetResyncEpoch();
    if (current_epoch != *last_resync_epoch)
    {
        *last_resync_epoch = current_epoch;
        ParserForceResync();
    }
}

static void NotifyRxTaskFromISR(BaseType_t *higher_priority_task_woken)
{
    if (s_rx_task != NULL)
    {
        vTaskNotifyGiveFromISR(s_rx_task, higher_priority_task_woken);
    }
}

static void NotifyTxTaskFromISR(BaseType_t *higher_priority_task_woken)
{
    if (s_tx_task != NULL)
    {
        vTaskNotifyGiveFromISR(s_tx_task, higher_priority_task_woken);
    }
}

static bool ClassifyTxPriority(uint8_t message_type, uint8_t *priority)
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
            return false;
    }
}

static int32_t FindFreeTxSlot(void)
{
    uint32_t i;

    for (i = 0U; i < APP_TX_REQUEST_SLOTS; ++i)
    {
        if (!s_tx_slots[i].used)
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
        if (!s_tx_slots[i].used)
        {
            continue;
        }

        if ((best_index < 0) ||
            (s_tx_slots[i].priority < best_priority) ||
            ((s_tx_slots[i].priority == best_priority) &&
             ((int32_t)(s_tx_slots[i].enqueue_order - best_order) < 0)))
        {
            best_index = (int32_t)i;
            best_priority = s_tx_slots[i].priority;
            best_order = s_tx_slots[i].enqueue_order;
        }
    }

    if (best_index >= 0)
    {
        *request = s_tx_slots[(uint32_t)best_index].request;
        s_tx_slots[(uint32_t)best_index].used = false;
    }

    taskEXIT_CRITICAL();

    if (best_index < 0)
    {
        return false;
    }

    (void)xSemaphoreGive(s_free_slot_sem);
    return true;
}
