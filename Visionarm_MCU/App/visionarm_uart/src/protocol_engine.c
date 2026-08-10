#include "protocol_engine.h"

#include <string.h>

#include "app_config.h"
#include "protocol_dispatcher.h"
#include "protocol_message.h"
#include "visionarm_uart_c/frame_parser.h"

static va_uart_frame_parser_t s_parser;
static ProtocolRxStats s_accumulated_stats;

static void OnFrame(const va_uart_frame_t *frame, void *context);
static void AccumulateCurrentStats(void);

void ProtocolEngine_Init(void)
{
    (void)memset(&s_accumulated_stats, 0, sizeof(s_accumulated_stats));
    va_uart_frame_parser_init(&s_parser,
                              APP_PARSER_ASSEMBLY_TIMEOUT_MS,
                              OnFrame,
                              NULL);
}

void ProtocolEngine_Feed(const uint8_t *data, size_t length, uint32_t now_ms)
{
    if ((data == NULL) || (length == 0U))
    {
        return;
    }

    va_uart_frame_parser_feed(&s_parser, data, length, now_ms);
}

void ProtocolEngine_Tick(uint32_t now_ms)
{
    va_uart_frame_parser_tick(&s_parser, now_ms);
}

void ProtocolEngine_ForceResync(void)
{
    AccumulateCurrentStats();
    va_uart_frame_parser_reset(&s_parser);
}

void ProtocolEngine_GetRxStats(ProtocolRxStats *stats)
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
        s_accumulated_stats.unknown_type_count + s_parser.stats.unknown_type_errors;
}

bool ProtocolEngine_EncodeFrame(const va_uart_header_t *header,
                                const uint8_t *payload,
                                size_t payload_size,
                                uint8_t *encoded,
                                size_t encoded_capacity,
                                size_t *encoded_size)
{
    return va_uart_encode_frame(header,
                                payload,
                                payload_size,
                                encoded,
                                encoded_capacity,
                                encoded_size);
}

static void OnFrame(const va_uart_frame_t *frame, void *context)
{
    ProtocolMessage message;

    (void)context;

    if (ProtocolMessage_Decode(frame, &message))
    {
        ProtocolDispatcher_OnMessage(&message);
    }
}

static void AccumulateCurrentStats(void)
{
    s_accumulated_stats.valid_frame_count += s_parser.stats.valid_frames;
    s_accumulated_stats.crc_error_count += s_parser.stats.crc_errors;
    s_accumulated_stats.length_error_count += s_parser.stats.length_errors;
    s_accumulated_stats.version_error_count += s_parser.stats.version_errors;
    s_accumulated_stats.unknown_type_count += s_parser.stats.unknown_type_errors;
}
