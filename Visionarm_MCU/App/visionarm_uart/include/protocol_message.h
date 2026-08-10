#ifndef VISIONARM_PROTOCOL_MESSAGE_H
#define VISIONARM_PROTOCOL_MESSAGE_H

#include <stdbool.h>
#include <stdint.h>

#include "visionarm_uart_c/messages.h"
#include "visionarm_uart_c/protocol.h"

typedef union
{
    va_uart_hello_t hello;
    va_uart_control_update_t control_update;
    va_uart_remote_stop_request_t remote_stop_request;
    va_uart_clear_remote_stop_t clear_remote_stop;
    va_uart_ping_t ping;
} ProtocolInboundPayload;

typedef struct
{
    va_uart_header_t header;
    uint16_t payload_crc;
    ProtocolInboundPayload payload;
} ProtocolMessage;

/* Decode only Linux->MCU message types accepted by this application. */
bool ProtocolMessage_Decode(const va_uart_frame_t *frame,
                            ProtocolMessage *message);

#endif /* VISIONARM_PROTOCOL_MESSAGE_H */
