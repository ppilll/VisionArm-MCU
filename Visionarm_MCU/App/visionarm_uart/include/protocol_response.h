#ifndef VISIONARM_PROTOCOL_RESPONSE_H
#define VISIONARM_PROTOCOL_RESPONSE_H

#include <stdbool.h>
#include <stdint.h>

#include "visionarm_uart_c/messages.h"
#include "visionarm_uart_c/protocol.h"

bool ProtocolResponse_SendHelloAck(const va_uart_header_t *request_header,
                                   bool accepted,
                                   uint8_t result_code);
bool ProtocolResponse_SendStatus(void);
bool ProtocolResponse_SendPong(const va_uart_header_t *request_header,
                               const va_uart_ping_t *ping);
bool ProtocolResponse_SendAckOrNack(uint8_t request_message_type,
                                    uint32_t transaction_id,
                                    bool acknowledged,
                                    uint16_t result_code,
                                    uint16_t detail_code);

#endif /* VISIONARM_PROTOCOL_RESPONSE_H */
