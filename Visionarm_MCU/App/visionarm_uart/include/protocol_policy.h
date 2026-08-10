#ifndef VISIONARM_PROTOCOL_POLICY_H
#define VISIONARM_PROTOCOL_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#include "visionarm_uart_c/messages.h"
#include "visionarm_uart_c/protocol.h"

#define PROTOCOL_PEER_DEVICE_ROLE              1U
#define PROTOCOL_LOCAL_DEVICE_ROLE             2U
#define PROTOCOL_REQUIRED_CAPABILITIES         0x00000000UL
#define PROTOCOL_LOCAL_CAPABILITIES            0x00000000UL
#define PROTOCOL_MIN_PEER_MAX_PAYLOAD          52U
#define PROTOCOL_LOCAL_MAX_PAYLOAD             ((uint16_t)VA_UART_MAX_PAYLOAD_SIZE)
#define PROTOCOL_LOCAL_MAX_CONTROL_RATE_HZ      100U

#define APP_FIRMWARE_VERSION_MAJOR             5U
#define APP_FIRMWARE_VERSION_MINOR             4U
#define APP_FIRMWARE_VERSION_PATCH             0U

#define PROTOCOL_TARGET_STATE_DETECTED          2U
#define PROTOCOL_CONTROL_FLAG_VALID             0x01U
#define PROTOCOL_CONTROL_MAX_CAPTURE_AGE_MS     1000U

#define PROTOCOL_MCU_STATE_RUNNING              1U

#define PROTOCOL_HELLO_OK                       0U
#define PROTOCOL_HELLO_BAD_ROLE                 1U
#define PROTOCOL_HELLO_BAD_VERSION              2U
#define PROTOCOL_HELLO_BAD_MAX_PAYLOAD          3U
#define PROTOCOL_HELLO_MISSING_CAPABILITY       4U

#define PROTOCOL_ACK_OK                         0U
#define PROTOCOL_NACK_NOT_READY                 1U
#define PROTOCOL_NACK_STALE_SEQUENCE            2U
#define PROTOCOL_NACK_TRANSACTION_CONFLICT      3U
#define PROTOCOL_NACK_CLEAR_NOT_ALLOWED         4U
#define PROTOCOL_DETAIL_NONE                    0U

typedef struct
{
    bool accepted;
    uint8_t result_code;
} ProtocolHelloDecision;

ProtocolHelloDecision ProtocolPolicy_ValidateHello(const va_uart_hello_t *hello);
bool ProtocolPolicy_ValidateControl(const va_uart_control_update_t *control);
bool ProtocolPolicy_CanClearRemoteStop(bool link_ready, bool remote_stop_latched);

#endif /* VISIONARM_PROTOCOL_POLICY_H */
