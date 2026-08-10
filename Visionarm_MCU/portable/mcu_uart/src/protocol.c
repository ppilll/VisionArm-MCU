#include "visionarm_uart_c/protocol.h"

#include <string.h>

static void write_u16_le(uint8_t* output, uint16_t value) {
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void write_u32_le(uint8_t* output, uint32_t value) {
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)((value >> 8U) & 0xFFU);
    output[2] = (uint8_t)((value >> 16U) & 0xFFU);
    output[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static uint16_t read_u16_le(const uint8_t* input) {
    return (uint16_t)((uint16_t)input[0] |
                      (uint16_t)((uint16_t)input[1] << 8U));
}

static uint32_t read_u32_le(const uint8_t* input) {
    return (uint32_t)input[0] |
           ((uint32_t)input[1] << 8U) |
           ((uint32_t)input[2] << 16U) |
           ((uint32_t)input[3] << 24U);
}

static bool append_escaped(uint8_t value,
                           uint8_t* output,
                           size_t capacity,
                           size_t* offset) {
    if (value == VA_UART_FRAME_FLAG || value == VA_UART_FRAME_ESCAPE) {
        if (capacity - *offset < 2U) {
            return false;
        }
        output[*offset] = VA_UART_FRAME_ESCAPE;
        output[*offset + 1U] = (uint8_t)(value ^ VA_UART_ESCAPE_XOR);
        *offset += 2U;
        return true;
    }
    if (*offset >= capacity) {
        return false;
    }
    output[*offset] = value;
    ++(*offset);
    return true;
}

bool va_uart_is_known_message_type(uint8_t message_type) {
    return message_type >= (uint8_t)VA_UART_MSG_HELLO &&
           message_type <= (uint8_t)VA_UART_MSG_PONG;
}

uint16_t va_uart_crc16_ccitt_false(const uint8_t* data, size_t size) {
    uint16_t crc = 0xFFFFU;
    size_t index = 0U;
    if (data == NULL && size != 0U) {
        return crc;
    }
    for (index = 0U; index < size; ++index) {
        unsigned bit = 0U;
        crc ^= (uint16_t)((uint16_t)data[index] << 8U);
        for (bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((uint16_t)(crc << 1U) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1U);
            }
        }
    }
    return crc;
}

bool va_uart_encode_frame(const va_uart_header_t* header,
                          const uint8_t* payload,
                          size_t payload_size,
                          uint8_t* encoded,
                          size_t encoded_capacity,
                          size_t* encoded_size) {
    uint8_t raw[VA_UART_MAX_RAW_FRAME_SIZE];
    size_t raw_size = 0U;
    size_t output_offset = 0U;
    size_t index = 0U;
    uint16_t crc = 0U;

    if (header == NULL || encoded == NULL || encoded_size == NULL ||
        (payload == NULL && payload_size != 0U) ||
        payload_size > VA_UART_MAX_PAYLOAD_SIZE ||
        header->protocol_version != VA_UART_PROTOCOL_VERSION ||
        !va_uart_is_known_message_type(header->message_type) ||
        encoded_capacity < 2U) {
        return false;
    }

    raw[0] = header->protocol_version;
    raw[1] = header->message_type;
    write_u16_le(&raw[2], (uint16_t)payload_size);
    write_u32_le(&raw[4], header->wire_sequence);
    write_u32_le(&raw[8], header->sender_boot_id);
    write_u32_le(&raw[12], header->sender_uptime_ms);
    raw_size = VA_UART_HEADER_SIZE;
    if (payload_size != 0U) {
        memcpy(&raw[raw_size], payload, payload_size);
        raw_size += payload_size;
    }
    crc = va_uart_crc16_ccitt_false(raw, raw_size);
    write_u16_le(&raw[raw_size], crc);
    raw_size += VA_UART_CRC_SIZE;

    encoded[output_offset++] = VA_UART_FRAME_FLAG;
    for (index = 0U; index < raw_size; ++index) {
        if (!append_escaped(raw[index], encoded, encoded_capacity,
                            &output_offset)) {
            *encoded_size = 0U;
            return false;
        }
    }
    if (output_offset >= encoded_capacity) {
        *encoded_size = 0U;
        return false;
    }
    encoded[output_offset++] = VA_UART_FRAME_FLAG;
    *encoded_size = output_offset;
    return output_offset <= VA_UART_MAX_ENCODED_FRAME_SIZE;
}

bool va_uart_decode_raw_frame(const uint8_t* raw,
                              size_t raw_size,
                              va_uart_frame_t* frame,
                              va_uart_frame_error_t* error) {
    uint16_t payload_length = 0U;
    size_t expected_size = 0U;
    uint16_t received_crc = 0U;
    uint16_t computed_crc = 0U;

    if (error != NULL) {
        *error = VA_UART_FRAME_OK;
    }
    if (raw == NULL || frame == NULL ||
        raw_size < VA_UART_HEADER_SIZE + VA_UART_CRC_SIZE) {
        if (error != NULL) *error = VA_UART_FRAME_TOO_SHORT;
        return false;
    }
    if (raw_size > VA_UART_MAX_RAW_FRAME_SIZE) {
        if (error != NULL) *error = VA_UART_FRAME_OVERSIZE_ERROR;
        return false;
    }

    payload_length = read_u16_le(&raw[2]);
    expected_size = VA_UART_HEADER_SIZE + (size_t)payload_length +
                    VA_UART_CRC_SIZE;
    if ((size_t)payload_length > VA_UART_MAX_PAYLOAD_SIZE ||
        raw_size != expected_size) {
        if (error != NULL) *error = VA_UART_FRAME_LENGTH_ERROR;
        return false;
    }

    received_crc = read_u16_le(&raw[raw_size - VA_UART_CRC_SIZE]);
    computed_crc = va_uart_crc16_ccitt_false(
        raw, raw_size - VA_UART_CRC_SIZE);
    if (received_crc != computed_crc) {
        if (error != NULL) *error = VA_UART_FRAME_CRC_ERROR;
        return false;
    }
    if (raw[0] != VA_UART_PROTOCOL_VERSION) {
        if (error != NULL) *error = VA_UART_FRAME_VERSION_ERROR;
        return false;
    }
    if (!va_uart_is_known_message_type(raw[1])) {
        if (error != NULL) *error = VA_UART_FRAME_UNKNOWN_TYPE_ERROR;
        return false;
    }

    frame->header.protocol_version = raw[0];
    frame->header.message_type = raw[1];
    frame->header.payload_length = payload_length;
    frame->header.wire_sequence = read_u32_le(&raw[4]);
    frame->header.sender_boot_id = read_u32_le(&raw[8]);
    frame->header.sender_uptime_ms = read_u32_le(&raw[12]);
    frame->payload_size = (size_t)payload_length;
    memset(frame->payload, 0, sizeof(frame->payload));
    if (payload_length != 0U) {
        memcpy(frame->payload, &raw[VA_UART_HEADER_SIZE], payload_length);
    }
    return true;
}
