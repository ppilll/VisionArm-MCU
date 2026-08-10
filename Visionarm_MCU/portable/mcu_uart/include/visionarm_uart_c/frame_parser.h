#ifndef VISIONARM_UART_C_FRAME_PARSER_H
#define VISIONARM_UART_C_FRAME_PARSER_H

#include "visionarm_uart_c/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct va_uart_parser_stats {
    uint32_t input_bytes;
    uint32_t noise_bytes;
    uint32_t valid_frames;
    uint32_t empty_frames;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t version_errors;
    uint32_t unknown_type_errors;
    uint32_t oversize_errors;
    uint32_t escape_errors;
    uint32_t timeout_errors;
} va_uart_parser_stats_t;

typedef void (*va_uart_frame_callback_t)(const va_uart_frame_t* frame,
                                         void* context);

typedef struct va_uart_frame_parser {
    uint32_t assembly_timeout_ms;
    bool collecting;
    bool escape_pending;
    bool discard_until_flag;
    bool has_activity_time;
    uint32_t last_activity_ms;
    uint8_t raw[VA_UART_MAX_RAW_FRAME_SIZE];
    size_t raw_size;
    va_uart_parser_stats_t stats;
    va_uart_frame_callback_t callback;
    void* callback_context;
} va_uart_frame_parser_t;

void va_uart_frame_parser_init(va_uart_frame_parser_t* parser,
                               uint32_t assembly_timeout_ms,
                               va_uart_frame_callback_t callback,
                               void* callback_context);
void va_uart_frame_parser_reset(va_uart_frame_parser_t* parser);
void va_uart_frame_parser_feed(va_uart_frame_parser_t* parser,
                               const uint8_t* data,
                               size_t size,
                               uint32_t now_ms);
void va_uart_frame_parser_tick(va_uart_frame_parser_t* parser,
                               uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
