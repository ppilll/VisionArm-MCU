#include "uart_rx_ring.h"

#include "app_config.h"
#include "stm32f1xx.h"

#define RX_RING_MASK (APP_RX_RING_CAPACITY - 1U)

static uint8_t s_ring[APP_RX_RING_CAPACITY];
static volatile uint32_t s_write_sequence;
static volatile uint32_t s_read_sequence;
static volatile uint32_t s_resync_epoch;
static volatile uint32_t s_overflow_count;

void UartRxRing_Init(void)
{
    s_write_sequence = 0U;
    s_read_sequence = 0U;
    s_resync_epoch = 0U;
    s_overflow_count = 0U;
}

bool UartRxRing_PushFromISR(uint8_t byte, bool *should_wake_task)
{
    uint32_t write_sequence = s_write_sequence;
    uint32_t read_sequence = s_read_sequence;

    if (should_wake_task != NULL)
    {
        *should_wake_task = (write_sequence == read_sequence);
    }

    if ((uint32_t)(write_sequence - read_sequence) >= APP_RX_RING_CAPACITY)
    {
        s_overflow_count++;
        s_resync_epoch++;
        return false;
    }

    s_ring[write_sequence & RX_RING_MASK] = byte;
    __DMB();
    s_write_sequence = write_sequence + 1U;
    return true;
}

void UartRxRing_MarkResyncFromISR(void)
{
    s_resync_epoch++;
}

size_t UartRxRing_Read(uint8_t *output, size_t output_capacity)
{
    uint32_t read_sequence;
    uint32_t available;
    size_t count;
    size_t i;

    if ((output == NULL) || (output_capacity == 0U))
    {
        return 0U;
    }

    read_sequence = s_read_sequence;
    available = (uint32_t)(s_write_sequence - read_sequence);
    count = (available > output_capacity) ? output_capacity : (size_t)available;

    for (i = 0U; i < count; ++i)
    {
        output[i] = s_ring[(read_sequence + (uint32_t)i) & RX_RING_MASK];
    }

    __DMB();
    s_read_sequence = read_sequence + (uint32_t)count;
    return count;
}

uint32_t UartRxRing_GetResyncEpoch(void)
{
    return s_resync_epoch;
}

uint32_t UartRxRing_GetOverflowCount(void)
{
    return s_overflow_count;
}
