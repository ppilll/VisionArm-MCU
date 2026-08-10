#include "visionarm_uart_c/frame_parser.h"

#include <string.h>

static void reset_candidate(va_uart_frame_parser_t* parser,
                            bool collecting) {
    parser->collecting = collecting;
    parser->escape_pending = false;
    parser->discard_until_flag = false;
    parser->has_activity_time = false;
    parser->last_activity_ms = 0U;
    parser->raw_size = 0U;
}

static void account_error(va_uart_frame_parser_t* parser,
                          va_uart_frame_error_t error) {
    switch (error) {
        case VA_UART_FRAME_CRC_ERROR:
            ++parser->stats.crc_errors;
            break;
        case VA_UART_FRAME_VERSION_ERROR:
            ++parser->stats.version_errors;
            break;
        case VA_UART_FRAME_UNKNOWN_TYPE_ERROR:
            ++parser->stats.unknown_type_errors;
            break;
        case VA_UART_FRAME_OVERSIZE_ERROR:
            ++parser->stats.oversize_errors;
            break;
        case VA_UART_FRAME_TOO_SHORT:
        case VA_UART_FRAME_LENGTH_ERROR:
            ++parser->stats.length_errors;
            break;
        case VA_UART_FRAME_OK:
        default:
            break;
    }
}

static void append_raw(va_uart_frame_parser_t* parser, uint8_t value) {
    if (parser->raw_size >= VA_UART_MAX_RAW_FRAME_SIZE) {
        ++parser->stats.oversize_errors;
        parser->discard_until_flag = true;
        parser->escape_pending = false;
        parser->raw_size = 0U;
        return;
    }
    parser->raw[parser->raw_size++] = value;
}

static void handle_flag(va_uart_frame_parser_t* parser,
                        uint32_t now_ms) {
    va_uart_frame_t frame;
    va_uart_frame_error_t error = VA_UART_FRAME_OK;

    if (!parser->collecting) {
        reset_candidate(parser, true);
        parser->has_activity_time = true;
        parser->last_activity_ms = now_ms;
        return;
    }
    if (parser->escape_pending) {
        ++parser->stats.escape_errors;
        reset_candidate(parser, true);
        parser->has_activity_time = true;
        parser->last_activity_ms = now_ms;
        return;
    }
    if (parser->discard_until_flag) {
        reset_candidate(parser, true);
        parser->has_activity_time = true;
        parser->last_activity_ms = now_ms;
        return;
    }
    if (parser->raw_size == 0U) {
        ++parser->stats.empty_frames;
        parser->has_activity_time = true;
        parser->last_activity_ms = now_ms;
        return;
    }

    if (va_uart_decode_raw_frame(parser->raw, parser->raw_size,
                                 &frame, &error)) {
        ++parser->stats.valid_frames;
        if (parser->callback != NULL) {
            parser->callback(&frame, parser->callback_context);
        }
    } else {
        account_error(parser, error);
    }
    reset_candidate(parser, true);
    parser->has_activity_time = true;
    parser->last_activity_ms = now_ms;
}

void va_uart_frame_parser_init(va_uart_frame_parser_t* parser,
                               uint32_t assembly_timeout_ms,
                               va_uart_frame_callback_t callback,
                               void* callback_context) {
    if (parser == NULL) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->assembly_timeout_ms = assembly_timeout_ms;
    parser->callback = callback;
    parser->callback_context = callback_context;
}

void va_uart_frame_parser_reset(va_uart_frame_parser_t* parser) {
    uint32_t timeout = 0U;
    va_uart_frame_callback_t callback = NULL;
    void* context = NULL;
    if (parser == NULL) {
        return;
    }
    timeout = parser->assembly_timeout_ms;
    callback = parser->callback;
    context = parser->callback_context;
    memset(parser, 0, sizeof(*parser));
    parser->assembly_timeout_ms = timeout;
    parser->callback = callback;
    parser->callback_context = context;
}

void va_uart_frame_parser_feed(va_uart_frame_parser_t* parser,
                               const uint8_t* data,
                               size_t size,
                               uint32_t now_ms) {
    size_t index = 0U;
    if (parser == NULL || data == NULL) {
        return;
    }
    va_uart_frame_parser_tick(parser, now_ms);
    for (index = 0U; index < size; ++index) {
        uint8_t value = data[index];
        ++parser->stats.input_bytes;
        if (value == VA_UART_FRAME_FLAG) {
            handle_flag(parser, now_ms);
            continue;
        }
        if (!parser->collecting) {
            ++parser->stats.noise_bytes;
            continue;
        }
        parser->has_activity_time = true;
        parser->last_activity_ms = now_ms;
        if (parser->discard_until_flag) {
            continue;
        }
        if (parser->escape_pending) {
            parser->escape_pending = false;
            if (value != (uint8_t)(VA_UART_FRAME_FLAG ^ VA_UART_ESCAPE_XOR) &&
                value != (uint8_t)(VA_UART_FRAME_ESCAPE ^ VA_UART_ESCAPE_XOR)) {
                ++parser->stats.escape_errors;
                parser->discard_until_flag = true;
                parser->raw_size = 0U;
                continue;
            }
            append_raw(parser, (uint8_t)(value ^ VA_UART_ESCAPE_XOR));
            continue;
        }
        if (value == VA_UART_FRAME_ESCAPE) {
            parser->escape_pending = true;
            continue;
        }
        append_raw(parser, value);
    }
}

void va_uart_frame_parser_tick(va_uart_frame_parser_t* parser,
                               uint32_t now_ms) {
    uint32_t elapsed = 0U;
    if (parser == NULL || !parser->collecting ||
        !parser->has_activity_time || parser->assembly_timeout_ms == 0U) {
        return;
    }
    elapsed = now_ms - parser->last_activity_ms;
    if (elapsed > parser->assembly_timeout_ms) {
        ++parser->stats.timeout_errors;
        reset_candidate(parser, false);
    }
}
