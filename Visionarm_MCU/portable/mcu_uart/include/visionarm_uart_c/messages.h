#ifndef VISIONARM_UART_C_MESSAGES_H
#define VISIONARM_UART_C_MESSAGES_H

#include "visionarm_uart_c/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct va_uart_hello {
    uint8_t device_role;
    uint8_t minimum_protocol_version;
    uint8_t maximum_protocol_version;
    uint16_t max_payload;
    uint16_t max_control_rate_hz;
    uint32_t capability_bits;
    uint16_t software_version_major;
    uint16_t software_version_minor;
    uint16_t software_version_patch;
} va_uart_hello_t;

typedef struct va_uart_hello_ack {
    uint32_t hello_wire_sequence;
    uint8_t accepted;
    uint8_t selected_protocol_version;
    uint8_t device_role;
    uint8_t result_code;
    uint16_t max_payload;
    uint16_t max_control_rate_hz;
    uint32_t capability_bits;
    uint16_t firmware_version_major;
    uint16_t firmware_version_minor;
    uint16_t firmware_version_patch;
} va_uart_hello_ack_t;

typedef struct va_uart_control_update {
    uint32_t source_capture_session_id;
    uint32_t source_frame_id;
    uint32_t source_v4l2_sequence;
    uint8_t target_state;
    uint8_t control_flags;
    int16_t dx_px;
    int16_t dy_px;
    int16_t error_x_q15;
    int16_t error_y_q15;
    uint16_t confidence_u16;
    uint16_t capture_age_at_tx_ms;
} va_uart_control_update_t;

typedef struct va_uart_status {
    uint8_t mcu_state;
    uint8_t link_state;
    uint8_t remote_stop_latched;
    uint8_t control_valid;
    uint32_t last_rx_wire_sequence;
    uint32_t last_control_wire_sequence;
    uint32_t rx_valid_frame_count;
    uint32_t rx_crc_error_count;
    uint32_t rx_length_error_count;
    uint32_t rx_version_error_count;
    uint32_t rx_unknown_type_count;
    uint32_t rx_sequence_gap_count;
    uint32_t rx_overflow_count;
    uint32_t control_mailbox_overwrite_count;
    uint32_t mcu_tick_ms;
    int16_t pan_stub_q15;
    int16_t tilt_stub_q15;
} va_uart_status_t;

typedef struct va_uart_remote_stop_request {
    uint32_t transaction_id;
    uint16_t reason_code;
} va_uart_remote_stop_request_t;

typedef struct va_uart_clear_remote_stop {
    uint32_t transaction_id;
} va_uart_clear_remote_stop_t;

typedef struct va_uart_ack {
    uint8_t acked_message_type;
    uint16_t result_code;
    uint32_t transaction_id;
    uint16_t detail_code;
} va_uart_ack_t;

typedef struct va_uart_ping {
    uint32_t ping_id;
} va_uart_ping_t;

typedef struct va_uart_pong {
    uint32_t ping_id;
    uint32_t ping_wire_sequence;
} va_uart_pong_t;

bool va_uart_encode_hello(const va_uart_hello_t* value,
                          uint8_t* payload, size_t capacity, size_t* size);
bool va_uart_decode_hello(const uint8_t* payload, size_t size,
                          va_uart_hello_t* value);
bool va_uart_encode_hello_ack(const va_uart_hello_ack_t* value,
                              uint8_t* payload, size_t capacity, size_t* size);
bool va_uart_decode_hello_ack(const uint8_t* payload, size_t size,
                              va_uart_hello_ack_t* value);
bool va_uart_encode_control_update(const va_uart_control_update_t* value,
                                   uint8_t* payload, size_t capacity,
                                   size_t* size);
bool va_uart_decode_control_update(const uint8_t* payload, size_t size,
                                   va_uart_control_update_t* value);
bool va_uart_encode_status(const va_uart_status_t* value,
                           uint8_t* payload, size_t capacity, size_t* size);
bool va_uart_decode_status(const uint8_t* payload, size_t size,
                           va_uart_status_t* value);
bool va_uart_encode_remote_stop_request(
    const va_uart_remote_stop_request_t* value,
    uint8_t* payload, size_t capacity, size_t* size);
bool va_uart_decode_remote_stop_request(
    const uint8_t* payload, size_t size,
    va_uart_remote_stop_request_t* value);
bool va_uart_encode_clear_remote_stop(
    const va_uart_clear_remote_stop_t* value,
    uint8_t* payload, size_t capacity, size_t* size);
bool va_uart_decode_clear_remote_stop(
    const uint8_t* payload, size_t size,
    va_uart_clear_remote_stop_t* value);
bool va_uart_encode_ack(const va_uart_ack_t* value,
                        uint8_t* payload, size_t capacity, size_t* size);
bool va_uart_decode_ack(const uint8_t* payload, size_t size,
                        va_uart_ack_t* value);
bool va_uart_encode_ping(const va_uart_ping_t* value,
                         uint8_t* payload, size_t capacity, size_t* size);
bool va_uart_decode_ping(const uint8_t* payload, size_t size,
                         va_uart_ping_t* value);
bool va_uart_encode_pong(const va_uart_pong_t* value,
                         uint8_t* payload, size_t capacity, size_t* size);
bool va_uart_decode_pong(const uint8_t* payload, size_t size,
                         va_uart_pong_t* value);

#ifdef __cplusplus
}
#endif

#endif
