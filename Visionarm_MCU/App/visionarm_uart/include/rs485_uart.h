#ifndef VISIONARM_RS485_UART_H
#define VISIONARM_RS485_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool Rs485Uart_Init(void);
void Rs485Uart_StartReceive(void);
bool Rs485Uart_StartTransmit(const uint8_t *data, size_t length);
void Rs485Uart_AbortTransmit(void);
void Rs485Uart_EnterReceive(void);
void Rs485Uart_EnterTransmit(void);
void Rs485Uart_IrqHandler(void);

/* Application hooks; strong definitions live in uart_transport_hooks.c. */
void Rs485Uart_OnRxByteFromISR(uint8_t byte);
void Rs485Uart_OnTxCompleteFromISR(void);
void Rs485Uart_OnErrorFromISR(uint32_t hal_error_flags);

#endif /* VISIONARM_RS485_UART_H */
