#include "visionarm_uart_c/messages.h"

#include <string.h>

static bool need(size_t capacity, size_t required) {
    return capacity >= required;
}

static void put_u16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8U) & 0xFFU);
}

static void put_i16(uint8_t* p, int16_t v) {
    put_u16(p, (uint16_t)v);
}

static void put_u32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8U) & 0xFFU);
    p[2] = (uint8_t)((v >> 16U) & 0xFFU);
    p[3] = (uint8_t)((v >> 24U) & 0xFFU);
}

static uint16_t get_u16(const uint8_t* p) {
    return (uint16_t)((uint16_t)p[0] |
                      (uint16_t)((uint16_t)p[1] << 8U));
}

static int16_t get_i16(const uint8_t* p) {
    return (int16_t)get_u16(p);
}

static uint32_t get_u32(const uint8_t* p) {
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8U) |
           ((uint32_t)p[2] << 16U) |
           ((uint32_t)p[3] << 24U);
}

bool va_uart_encode_hello(const va_uart_hello_t* v,
                          uint8_t* p, size_t c, size_t* s) {
    if (v == NULL || p == NULL || s == NULL || !need(c, 20U)) return false;
    p[0] = v->device_role; p[1] = v->minimum_protocol_version;
    p[2] = v->maximum_protocol_version; p[3] = 0U;
    put_u16(&p[4], v->max_payload); put_u16(&p[6], v->max_control_rate_hz);
    put_u32(&p[8], v->capability_bits);
    put_u16(&p[12], v->software_version_major);
    put_u16(&p[14], v->software_version_minor);
    put_u16(&p[16], v->software_version_patch); put_u16(&p[18], 0U);
    *s = 20U; return true;
}

bool va_uart_decode_hello(const uint8_t* p, size_t s, va_uart_hello_t* v) {
    if (p == NULL || v == NULL || s != 20U || p[3] != 0U || get_u16(&p[18]) != 0U) return false;
    v->device_role = p[0]; v->minimum_protocol_version = p[1];
    v->maximum_protocol_version = p[2]; v->max_payload = get_u16(&p[4]);
    v->max_control_rate_hz = get_u16(&p[6]); v->capability_bits = get_u32(&p[8]);
    v->software_version_major = get_u16(&p[12]);
    v->software_version_minor = get_u16(&p[14]);
    v->software_version_patch = get_u16(&p[16]); return true;
}

bool va_uart_encode_hello_ack(const va_uart_hello_ack_t* v,
                              uint8_t* p, size_t c, size_t* s) {
    if (v == NULL || p == NULL || s == NULL || !need(c, 24U)) return false;
    put_u32(&p[0], v->hello_wire_sequence); p[4] = v->accepted;
    p[5] = v->selected_protocol_version; p[6] = v->device_role; p[7] = v->result_code;
    put_u16(&p[8], v->max_payload); put_u16(&p[10], v->max_control_rate_hz);
    put_u32(&p[12], v->capability_bits);
    put_u16(&p[16], v->firmware_version_major);
    put_u16(&p[18], v->firmware_version_minor);
    put_u16(&p[20], v->firmware_version_patch); put_u16(&p[22], 0U);
    *s = 24U; return true;
}

bool va_uart_decode_hello_ack(const uint8_t* p, size_t s, va_uart_hello_ack_t* v) {
    if (p == NULL || v == NULL || s != 24U || get_u16(&p[22]) != 0U) return false;
    v->hello_wire_sequence = get_u32(&p[0]); v->accepted = p[4];
    v->selected_protocol_version = p[5]; v->device_role = p[6]; v->result_code = p[7];
    v->max_payload = get_u16(&p[8]); v->max_control_rate_hz = get_u16(&p[10]);
    v->capability_bits = get_u32(&p[12]);
    v->firmware_version_major = get_u16(&p[16]);
    v->firmware_version_minor = get_u16(&p[18]);
    v->firmware_version_patch = get_u16(&p[20]); return true;
}

bool va_uart_encode_control_update(const va_uart_control_update_t* v,
                                   uint8_t* p, size_t c, size_t* s) {
    if (v == NULL || p == NULL || s == NULL || !need(c, 26U) ||
        (v->control_flags & 0xFEU) != 0U) return false;
    put_u32(&p[0], v->source_capture_session_id);
    put_u32(&p[4], v->source_frame_id); put_u32(&p[8], v->source_v4l2_sequence);
    p[12] = v->target_state; p[13] = v->control_flags;
    put_i16(&p[14], v->dx_px); put_i16(&p[16], v->dy_px);
    put_i16(&p[18], v->error_x_q15); put_i16(&p[20], v->error_y_q15);
    put_u16(&p[22], v->confidence_u16); put_u16(&p[24], v->capture_age_at_tx_ms);
    *s = 26U; return true;
}

bool va_uart_decode_control_update(const uint8_t* p, size_t s,
                                   va_uart_control_update_t* v) {
    if (p == NULL || v == NULL || s != 26U || (p[13] & 0xFEU) != 0U) return false;
    v->source_capture_session_id = get_u32(&p[0]); v->source_frame_id = get_u32(&p[4]);
    v->source_v4l2_sequence = get_u32(&p[8]); v->target_state = p[12];
    v->control_flags = p[13]; v->dx_px = get_i16(&p[14]); v->dy_px = get_i16(&p[16]);
    v->error_x_q15 = get_i16(&p[18]); v->error_y_q15 = get_i16(&p[20]);
    v->confidence_u16 = get_u16(&p[22]); v->capture_age_at_tx_ms = get_u16(&p[24]);
    return true;
}

bool va_uart_encode_status(const va_uart_status_t* v,
                           uint8_t* p, size_t c, size_t* s) {
    if (v == NULL || p == NULL || s == NULL || !need(c, 52U)) return false;
    p[0] = v->mcu_state; p[1] = v->link_state; p[2] = v->remote_stop_latched; p[3] = v->control_valid;
    put_u32(&p[4], v->last_rx_wire_sequence); put_u32(&p[8], v->last_control_wire_sequence);
    put_u32(&p[12], v->rx_valid_frame_count); put_u32(&p[16], v->rx_crc_error_count);
    put_u32(&p[20], v->rx_length_error_count); put_u32(&p[24], v->rx_version_error_count);
    put_u32(&p[28], v->rx_unknown_type_count); put_u32(&p[32], v->rx_sequence_gap_count);
    put_u32(&p[36], v->rx_overflow_count); put_u32(&p[40], v->control_mailbox_overwrite_count);
    put_u32(&p[44], v->mcu_tick_ms); put_i16(&p[48], v->pan_stub_q15); put_i16(&p[50], v->tilt_stub_q15);
    *s = 52U; return true;
}

bool va_uart_decode_status(const uint8_t* p, size_t s, va_uart_status_t* v) {
    if (p == NULL || v == NULL || s != 52U) return false;
    v->mcu_state = p[0]; v->link_state = p[1]; v->remote_stop_latched = p[2]; v->control_valid = p[3];
    v->last_rx_wire_sequence = get_u32(&p[4]); v->last_control_wire_sequence = get_u32(&p[8]);
    v->rx_valid_frame_count = get_u32(&p[12]); v->rx_crc_error_count = get_u32(&p[16]);
    v->rx_length_error_count = get_u32(&p[20]); v->rx_version_error_count = get_u32(&p[24]);
    v->rx_unknown_type_count = get_u32(&p[28]); v->rx_sequence_gap_count = get_u32(&p[32]);
    v->rx_overflow_count = get_u32(&p[36]); v->control_mailbox_overwrite_count = get_u32(&p[40]);
    v->mcu_tick_ms = get_u32(&p[44]); v->pan_stub_q15 = get_i16(&p[48]); v->tilt_stub_q15 = get_i16(&p[50]);
    return true;
}

bool va_uart_encode_remote_stop_request(const va_uart_remote_stop_request_t* v,
                                        uint8_t* p, size_t c, size_t* s) {
    if (v == NULL || p == NULL || s == NULL || !need(c, 8U)) return false;
    put_u32(&p[0], v->transaction_id); put_u16(&p[4], v->reason_code); put_u16(&p[6], 0U);
    *s = 8U; return true;
}

bool va_uart_decode_remote_stop_request(const uint8_t* p, size_t s,
                                        va_uart_remote_stop_request_t* v) {
    if (p == NULL || v == NULL || s != 8U || get_u16(&p[6]) != 0U) return false;
    v->transaction_id = get_u32(&p[0]); v->reason_code = get_u16(&p[4]); return true;
}

bool va_uart_encode_clear_remote_stop(const va_uart_clear_remote_stop_t* v,
                                      uint8_t* p, size_t c, size_t* s) {
    if (v == NULL || p == NULL || s == NULL || !need(c, 4U)) return false;
    put_u32(&p[0], v->transaction_id); *s = 4U; return true;
}

bool va_uart_decode_clear_remote_stop(const uint8_t* p, size_t s,
                                      va_uart_clear_remote_stop_t* v) {
    if (p == NULL || v == NULL || s != 4U) return false;
    v->transaction_id = get_u32(&p[0]); return true;
}

bool va_uart_encode_ack(const va_uart_ack_t* v,
                        uint8_t* p, size_t c, size_t* s) {
    if (v == NULL || p == NULL || s == NULL || !need(c, 12U)) return false;
    p[0] = v->acked_message_type; p[1] = 0U; put_u16(&p[2], v->result_code);
    put_u32(&p[4], v->transaction_id); put_u16(&p[8], v->detail_code); put_u16(&p[10], 0U);
    *s = 12U; return true;
}

bool va_uart_decode_ack(const uint8_t* p, size_t s, va_uart_ack_t* v) {
    if (p == NULL || v == NULL || s != 12U || p[1] != 0U || get_u16(&p[10]) != 0U) return false;
    v->acked_message_type = p[0]; v->result_code = get_u16(&p[2]);
    v->transaction_id = get_u32(&p[4]); v->detail_code = get_u16(&p[8]); return true;
}

bool va_uart_encode_ping(const va_uart_ping_t* v,
                         uint8_t* p, size_t c, size_t* s) {
    if (v == NULL || p == NULL || s == NULL || !need(c, 4U)) return false;
    put_u32(&p[0], v->ping_id); *s = 4U; return true;
}

bool va_uart_decode_ping(const uint8_t* p, size_t s, va_uart_ping_t* v) {
    if (p == NULL || v == NULL || s != 4U) return false;
    v->ping_id = get_u32(&p[0]); return true;
}

bool va_uart_encode_pong(const va_uart_pong_t* v,
                         uint8_t* p, size_t c, size_t* s) {
    if (v == NULL || p == NULL || s == NULL || !need(c, 8U)) return false;
    put_u32(&p[0], v->ping_id); put_u32(&p[4], v->ping_wire_sequence); *s = 8U; return true;
}

bool va_uart_decode_pong(const uint8_t* p, size_t s, va_uart_pong_t* v) {
    if (p == NULL || v == NULL || s != 8U) return false;
    v->ping_id = get_u32(&p[0]); v->ping_wire_sequence = get_u32(&p[4]); return true;
}
