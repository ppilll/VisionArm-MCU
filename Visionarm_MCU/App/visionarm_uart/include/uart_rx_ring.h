#ifndef VISIONARM_UART_RX_RING_H
#define VISIONARM_UART_RX_RING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void UartRxRing_Init(void);
bool UartRxRing_PushFromISR(uint8_t byte, bool *should_wake_task);
void UartRxRing_MarkResyncFromISR(void);
size_t UartRxRing_Read(uint8_t *output, size_t output_capacity);
uint32_t UartRxRing_GetResyncEpoch(void);
uint32_t UartRxRing_GetOverflowCount(void);

#endif /* VISIONARM_UART_RX_RING_H */
