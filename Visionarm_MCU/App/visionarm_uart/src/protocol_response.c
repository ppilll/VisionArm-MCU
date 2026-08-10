#include "protocol_response.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"

#include "app_config.h"
#include "control_mailbox.h"
#include "gimbal_stub.h"
#include "protocol_engine.h"
#include "protocol_policy.h"
#include "protocol_state.h"
#include "protocol_tx_task.h"
#include "uart_rx_ring.h"

static bool QueuePayload(uint8_t message_type,
                         const uint8_t *payload,
                         size_t payload_size);

bool ProtocolResponse_SendHelloAck(const va_uart_header_t *request_header,
                                   bool accepted,
                                   uint8_t result_code)
{
    va_uart_hello_ack_t ack;
    uint8_t payload[24U];
    size_t payload_size;

    if (request_header == NULL)
    {
        return false;
    }

    ack.hello_wire_sequence = request_header->wire_sequence;
    ack.accepted = accepted ? 1U : 0U;
    ack.selected_protocol_version = VA_UART_PROTOCOL_VERSION;
    ack.device_role = PROTOCOL_LOCAL_DEVICE_ROLE;
    ack.result_code = result_code;
    ack.max_payload = PROTOCOL_LOCAL_MAX_PAYLOAD;
    ack.max_control_rate_hz = PROTOCOL_LOCAL_MAX_CONTROL_RATE_HZ;
    ack.capability_bits = PROTOCOL_LOCAL_CAPABILITIES;
    ack.firmware_version_major = APP_FIRMWARE_VERSION_MAJOR;
    ack.firmware_version_minor = APP_FIRMWARE_VERSION_MINOR;
    ack.firmware_version_patch = APP_FIRMWARE_VERSION_PATCH;

    if (!va_uart_encode_hello_ack(&ack, payload, sizeof(payload), &payload_size))
    {
        return false;
    }

    return QueuePayload((uint8_t)VA_UART_MSG_HELLO_ACK, payload, payload_size);
}

bool ProtocolResponse_SendStatus(void)
{
    va_uart_status_t status = {0};
    ProtocolStateSnapshot state;
    ProtocolRxStats rx_stats;
    GimbalStubSnapshot gimbal;
    uint8_t payload[52U];
    size_t payload_size;

    ProtocolState_GetSnapshot(&state);
    ProtocolEngine_GetRxStats(&rx_stats);
    GimbalStub_GetSnapshot(&gimbal);

    status.mcu_state = PROTOCOL_MCU_STATE_RUNNING;
    status.link_state = (uint8_t)state.link_state;
    status.remote_stop_latched = state.remote_stop_latched ? 1U : 0U;
    status.control_valid = state.control_valid ? 1U : 0U;
    status.last_rx_wire_sequence = state.last_rx_wire_sequence;
    status.last_control_wire_sequence = state.last_control_wire_sequence;
    status.rx_valid_frame_count = rx_stats.valid_frame_count;
    status.rx_crc_error_count = rx_stats.crc_error_count;
    status.rx_length_error_count = rx_stats.length_error_count;
    status.rx_version_error_count = rx_stats.version_error_count;
    status.rx_unknown_type_count = rx_stats.unknown_type_count;
    status.rx_sequence_gap_count = state.sequence_gap_count;
    status.rx_overflow_count = UartRxRing_GetOverflowCount();
    status.control_mailbox_overwrite_count = ControlMailbox_GetOverwriteCount();
    status.mcu_tick_ms = HAL_GetTick();
    status.pan_stub_q15 = gimbal.pan_q15;
    status.tilt_stub_q15 = gimbal.tilt_q15;

    if (!va_uart_encode_status(&status, payload, sizeof(payload), &payload_size))
    {
        return false;
    }

    return QueuePayload((uint8_t)VA_UART_MSG_STATUS, payload, payload_size);
}

bool ProtocolResponse_SendPong(const va_uart_header_t *request_header,
                               const va_uart_ping_t *ping)
{
    va_uart_pong_t pong;
    uint8_t payload[8U];
    size_t payload_size;

    if ((request_header == NULL) || (ping == NULL))
    {
        return false;
    }

    pong.ping_id = ping->ping_id;
    pong.ping_wire_sequence = request_header->wire_sequence;

    if (!va_uart_encode_pong(&pong, payload, sizeof(payload), &payload_size))
    {
        return false;
    }

    return QueuePayload((uint8_t)VA_UART_MSG_PONG, payload, payload_size);
}

bool ProtocolResponse_SendAckOrNack(uint8_t request_message_type,
                                    uint32_t transaction_id,
                                    bool acknowledged,
                                    uint16_t result_code,
                                    uint16_t detail_code)
{
    va_uart_ack_t ack;
    uint8_t payload[12U];
    size_t payload_size;
    uint8_t response_type;

    ack.acked_message_type = request_message_type;
    ack.result_code = result_code;
    ack.transaction_id = transaction_id;
    ack.detail_code = detail_code;

    if (!va_uart_encode_ack(&ack, payload, sizeof(payload), &payload_size))
    {
        return false;
    }

    response_type = acknowledged ?
        (uint8_t)VA_UART_MSG_ACK : (uint8_t)VA_UART_MSG_NACK;

    return QueuePayload(response_type, payload, payload_size);
}

static bool QueuePayload(uint8_t message_type,
                         const uint8_t *payload,
                         size_t payload_size)
{
    return ProtocolTxTask_EnqueueResponse(
        message_type,
        payload,
        payload_size,
        pdMS_TO_TICKS(APP_RESPONSE_ENQUEUE_WAIT_MS));
}
