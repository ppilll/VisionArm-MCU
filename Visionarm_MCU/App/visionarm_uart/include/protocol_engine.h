#ifndef VISIONARM_PROTOCOL_ENGINE_H
#define VISIONARM_PROTOCOL_ENGINE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "visionarm_uart_c/protocol.h"

typedef struct
{
    uint32_t valid_frame_count;
    uint32_t crc_error_count;
    uint32_t length_error_count;
    uint32_t version_error_count;
    uint32_t unknown_type_count;
} ProtocolRxStats;

void ProtocolEngine_Init(void);
void ProtocolEngine_Feed(const uint8_t *data, size_t length, uint32_t now_ms);
void ProtocolEngine_Tick(uint32_t now_ms);
void ProtocolEngine_ForceResync(void);
void ProtocolEngine_GetRxStats(ProtocolRxStats *stats);

bool ProtocolEngine_EncodeFrame(const va_uart_header_t *header,
                                const uint8_t *payload,
                                size_t payload_size,
                                uint8_t *encoded,
                                size_t encoded_capacity,
                                size_t *encoded_size);

#endif /* VISIONARM_PROTOCOL_ENGINE_H */
