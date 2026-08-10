#ifndef VISIONARM_UART_C_PROTOCOL_H
#define VISIONARM_UART_C_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VA_UART_FRAME_FLAG ((uint8_t)0x7E)
#define VA_UART_FRAME_ESCAPE ((uint8_t)0x7D)
#define VA_UART_ESCAPE_XOR ((uint8_t)0x20)
#define VA_UART_PROTOCOL_VERSION ((uint8_t)1)
#define VA_UART_HEADER_SIZE ((size_t)16)
#define VA_UART_CRC_SIZE ((size_t)2)
#define VA_UART_MAX_PAYLOAD_SIZE ((size_t)128)
#define VA_UART_MAX_RAW_FRAME_SIZE ((size_t)146)
#define VA_UART_MAX_ENCODED_FRAME_SIZE ((size_t)294)

typedef enum va_uart_message_type {
    VA_UART_MSG_HELLO = 0x01,
    VA_UART_MSG_HELLO_ACK = 0x02,
    VA_UART_MSG_HEARTBEAT = 0x03,
    VA_UART_MSG_CONTROL_UPDATE = 0x04,
    VA_UART_MSG_STATUS = 0x05,
    VA_UART_MSG_REMOTE_STOP_REQUEST = 0x06,
    VA_UART_MSG_CLEAR_REMOTE_STOP = 0x07,
    VA_UART_MSG_ACK = 0x08,
    VA_UART_MSG_NACK = 0x09,
    VA_UART_MSG_PING = 0x0A,
    VA_UART_MSG_PONG = 0x0B
} va_uart_message_type_t;

typedef enum va_uart_frame_error {
    VA_UART_FRAME_OK = 0,
    VA_UART_FRAME_TOO_SHORT,
    VA_UART_FRAME_LENGTH_ERROR,
    VA_UART_FRAME_CRC_ERROR,
    VA_UART_FRAME_VERSION_ERROR,
    VA_UART_FRAME_UNKNOWN_TYPE_ERROR,
    VA_UART_FRAME_OVERSIZE_ERROR
} va_uart_frame_error_t;

typedef struct va_uart_header {
    uint8_t protocol_version;
    uint8_t message_type;
    uint16_t payload_length;
    uint32_t wire_sequence;
    uint32_t sender_boot_id;
    uint32_t sender_uptime_ms;
} va_uart_header_t;

typedef struct va_uart_frame {
    va_uart_header_t header;
    uint8_t payload[VA_UART_MAX_PAYLOAD_SIZE];
    size_t payload_size;
} va_uart_frame_t;

bool va_uart_is_known_message_type(uint8_t message_type);
uint16_t va_uart_crc16_ccitt_false(const uint8_t* data, size_t size);

bool va_uart_encode_frame(const va_uart_header_t* header,
                          const uint8_t* payload,
                          size_t payload_size,
                          uint8_t* encoded,
                          size_t encoded_capacity,
                          size_t* encoded_size);

bool va_uart_decode_raw_frame(const uint8_t* raw,
                              size_t raw_size,
                              va_uart_frame_t* frame,
                              va_uart_frame_error_t* error);

#ifdef __cplusplus
}
#endif

#endif
