#include "protocol_core.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"

#include "gimbal_task.h"
#include "protocol_transport.h"
#include "visionarm_app.h"
#include "visionarm_uart_c/transaction_cache.h"

#define STM32F1_UID_WORD0_ADDRESS              ((uintptr_t)0x1FFFF7E8UL)
#define STM32F1_UID_WORD1_ADDRESS              ((uintptr_t)0x1FFFF7ECUL)
#define STM32F1_UID_WORD2_ADDRESS              ((uintptr_t)0x1FFFF7F0UL)

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

typedef enum
{
    PROTOCOL_SEQUENCE_NEW = 0,
    PROTOCOL_SEQUENCE_DUPLICATE,
    PROTOCOL_SEQUENCE_OLD
} ProtocolSequenceDecision;

typedef struct
{
    ProtocolSequenceDecision decision;
    uint32_t gap_count;
} ProtocolSequenceResult;

typedef struct
{
    bool accepted;
    uint8_t result_code;
} ProtocolHelloDecision;

typedef union
{
    va_uart_hello_t hello;
    va_uart_control_update_t control_update;
    va_uart_remote_stop_request_t remote_stop_request;
    va_uart_clear_remote_stop_t clear_remote_stop;
    va_uart_ping_t ping;
} ProtocolInboundPayload;

typedef struct
{
    va_uart_header_t header;
    uint16_t payload_crc;
    ProtocolInboundPayload payload;
} ProtocolMessage;

typedef struct
{
    ProtocolLinkState link_state;
    bool remote_stop_latched;
    bool control_valid;
    bool have_peer;
    bool sequence_valid;
    uint32_t mcu_boot_id;
    uint32_t peer_boot_id;
    uint32_t last_rx_wire_sequence;
    uint32_t last_control_wire_sequence;
    uint32_t sequence_gap_count;
} ProtocolRuntimeState;

static ProtocolRuntimeState s_state;
static ControlCommand s_command;
static uint32_t s_mailbox_overwrite_count;
static bool s_link_watchdog_armed;
static bool s_control_watchdog_armed;
static TickType_t s_last_link_tick;
static TickType_t s_last_control_tick;
static va_uart_transaction_cache_t s_transaction_cache;

static bool DecodeMessage(const va_uart_frame_t *frame, ProtocolMessage *message);
static void DispatchMessage(const ProtocolMessage *message);
static ProtocolHelloDecision ValidateHello(const va_uart_hello_t *hello);
static bool ValidateControl(const va_uart_control_update_t *control);
static bool CanClearRemoteStop(bool link_ready, bool remote_stop_latched);

static void StateInit(void);
static void StateAcceptHello(uint32_t peer_boot_id, uint32_t hello_wire_sequence);
static void StateRejectHello(void);
static void StatePeerMismatch(void);
static void StateLinkTimeout(void);
static bool StateIsReadyForPeer(uint32_t peer_boot_id);
static ProtocolSequenceResult StateObserveSequence(uint32_t wire_sequence);
static void StateSetControlValid(uint32_t wire_sequence);
static void StateInvalidateControl(void);
static void StateSetRemoteStop(bool latched);

static void MailboxInit(void);
static void MailboxPublish(uint32_t wire_sequence,
                           const va_uart_control_update_t *control,
                           bool valid);
static void MailboxInvalidate(void);

static void WatchdogInit(void);
static void WatchdogOnHelloAccepted(void);
static void WatchdogRefreshLink(void);
static void WatchdogRefreshControl(void);
static void WatchdogDisarmControl(void);
static void WatchdogDisarmAll(void);
static void WatchdogCheck(void);

static bool RequireReadyPeer(const ProtocolMessage *message);
static void ReplyCached(uint8_t request_type,
                        uint32_t transaction_id,
                        const va_uart_cached_result_t *cached);
static void RecordAndReply(uint8_t request_type,
                           uint32_t transaction_id,
                           uint16_t payload_crc,
                           bool acknowledged,
                           uint16_t result_code);

static bool SendHelloAck(const va_uart_header_t *request_header,
                         bool accepted,
                         uint8_t result_code);
static bool SendStatus(void);
static bool SendPong(const va_uart_header_t *request_header,
                     const va_uart_ping_t *ping);
static bool SendAckOrNack(uint8_t request_message_type,
                          uint32_t transaction_id,
                          bool acknowledged,
                          uint16_t result_code,
                          uint16_t detail_code);

static uint32_t GenerateBootId(void);
static uint32_t Mix32(uint32_t value);

void ProtocolCore_Init(void)
{
    StateInit();
    MailboxInit();
    WatchdogInit();
    va_uart_transaction_cache_reset(&s_transaction_cache);
}

void ProtocolCore_OnFrame(const va_uart_frame_t *frame)
{
    ProtocolMessage message;

    if (DecodeMessage(frame, &message))
    {
        DispatchMessage(&message);
    }
}

void ProtocolCore_Tick(void)
{
    WatchdogCheck();
}

void ProtocolCore_GetState(ProtocolStateSnapshot *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    snapshot->link_state = s_state.link_state;
    snapshot->remote_stop_latched = s_state.remote_stop_latched;
    snapshot->control_valid = s_state.control_valid;
    snapshot->mcu_boot_id = s_state.mcu_boot_id;
    snapshot->last_rx_wire_sequence = s_state.last_rx_wire_sequence;
    snapshot->last_control_wire_sequence = s_state.last_control_wire_sequence;
    snapshot->sequence_gap_count = s_state.sequence_gap_count;
    taskEXIT_CRITICAL();
}

bool ProtocolCore_ReadControl(ControlCommand *command)
{
    bool available;

    if (command == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    *command = s_command;
    available = (s_command.generation != 0U);
    taskEXIT_CRITICAL();
    return available;
}

uint32_t ProtocolCore_GetMcuBootId(void)
{
    return s_state.mcu_boot_id;
}

uint32_t ProtocolCore_GetMailboxOverwriteCount(void)
{
    uint32_t count;

    taskENTER_CRITICAL();
    count = s_mailbox_overwrite_count;
    taskEXIT_CRITICAL();
    return count;
}

static bool DecodeMessage(const va_uart_frame_t *frame, ProtocolMessage *message)
{
    bool ok = false;

    if ((frame == NULL) || (message == NULL))
    {
        return false;
    }

    (void)memset(message, 0, sizeof(*message));
    message->header = frame->header;

    switch ((va_uart_message_type_t)frame->header.message_type)
    {
        case VA_UART_MSG_HELLO:
            ok = va_uart_decode_hello(frame->payload,
                                      frame->payload_size,
                                      &message->payload.hello);
            break;

        case VA_UART_MSG_HEARTBEAT:
            ok = (frame->payload_size == 0U);
            break;

        case VA_UART_MSG_CONTROL_UPDATE:
            ok = va_uart_decode_control_update(
                frame->payload,
                frame->payload_size,
                &message->payload.control_update);
            break;

        case VA_UART_MSG_REMOTE_STOP_REQUEST:
            ok = va_uart_decode_remote_stop_request(
                frame->payload,
                frame->payload_size,
                &message->payload.remote_stop_request);
            if (ok)
            {
                message->payload_crc = va_uart_crc16_ccitt_false(
                    frame->payload, frame->payload_size);
            }
            break;

        case VA_UART_MSG_CLEAR_REMOTE_STOP:
            ok = va_uart_decode_clear_remote_stop(
                frame->payload,
                frame->payload_size,
                &message->payload.clear_remote_stop);
            if (ok)
            {
                message->payload_crc = va_uart_crc16_ccitt_false(
                    frame->payload, frame->payload_size);
            }
            break;

        case VA_UART_MSG_PING:
            ok = va_uart_decode_ping(frame->payload,
                                     frame->payload_size,
                                     &message->payload.ping);
            break;

        default:
            break;
    }

    return ok;
}

static void DispatchMessage(const ProtocolMessage *message)
{
    ProtocolSequenceResult sequence;
    ProtocolStateSnapshot state;
    ProtocolHelloDecision hello_decision;
    va_uart_cached_result_t cached;
    bool valid;
    bool allowed;

    if (message == NULL)
    {
        return;
    }

    switch ((va_uart_message_type_t)message->header.message_type)
    {
        case VA_UART_MSG_HELLO:
            hello_decision = ValidateHello(&message->payload.hello);

            if (hello_decision.accepted)
            {
                StateAcceptHello(message->header.sender_boot_id,
                                 message->header.wire_sequence);
                va_uart_transaction_cache_reset(&s_transaction_cache);
                StateInvalidateControl();
                MailboxInvalidate();
                WatchdogOnHelloAccepted();
            }
            else
            {
                StateRejectHello();
                MailboxInvalidate();
                WatchdogDisarmAll();
            }

            (void)SendHelloAck(&message->header,
                               hello_decision.accepted,
                               hello_decision.result_code);
            break;

        case VA_UART_MSG_HEARTBEAT:
            if (!RequireReadyPeer(message))
            {
                return;
            }

            sequence = StateObserveSequence(message->header.wire_sequence);
            if (sequence.decision != PROTOCOL_SEQUENCE_NEW)
            {
                return;
            }

            WatchdogRefreshLink();
            (void)SendStatus();
            break;

        case VA_UART_MSG_CONTROL_UPDATE:
            if (!RequireReadyPeer(message))
            {
                StateInvalidateControl();
                WatchdogDisarmControl();
                MailboxPublish(message->header.wire_sequence,
                               &message->payload.control_update,
                               false);
                return;
            }

            sequence = StateObserveSequence(message->header.wire_sequence);
            if (sequence.decision == PROTOCOL_SEQUENCE_NEW)
            {
                WatchdogRefreshLink();
            }

            ProtocolCore_GetState(&state);
            valid = (sequence.decision == PROTOCOL_SEQUENCE_NEW) &&
                    !state.remote_stop_latched &&
                    ValidateControl(&message->payload.control_update);

            MailboxPublish(message->header.wire_sequence,
                           &message->payload.control_update,
                           valid);

            if (valid)
            {
                StateSetControlValid(message->header.wire_sequence);
                WatchdogRefreshControl();
            }
            else
            {
                StateInvalidateControl();
                WatchdogDisarmControl();
            }
            break;

        case VA_UART_MSG_REMOTE_STOP_REQUEST:
            if (!RequireReadyPeer(message))
            {
                RecordAndReply((uint8_t)VA_UART_MSG_REMOTE_STOP_REQUEST,
                               message->payload.remote_stop_request.transaction_id,
                               message->payload_crc,
                               false,
                               PROTOCOL_NACK_NOT_READY);
                return;
            }

            sequence = StateObserveSequence(message->header.wire_sequence);
            if (sequence.decision == PROTOCOL_SEQUENCE_NEW)
            {
                WatchdogRefreshLink();
            }

            cached = va_uart_transaction_cache_lookup(
                &s_transaction_cache,
                (uint8_t)VA_UART_MSG_REMOTE_STOP_REQUEST,
                message->payload.remote_stop_request.transaction_id,
                message->payload_crc);

            if (cached.decision == VA_UART_CACHE_DUPLICATE)
            {
                ReplyCached((uint8_t)VA_UART_MSG_REMOTE_STOP_REQUEST,
                            message->payload.remote_stop_request.transaction_id,
                            &cached);
                return;
            }

            if (cached.decision == VA_UART_CACHE_CONFLICTING_DUPLICATE)
            {
                (void)SendAckOrNack(
                    (uint8_t)VA_UART_MSG_REMOTE_STOP_REQUEST,
                    message->payload.remote_stop_request.transaction_id,
                    false,
                    PROTOCOL_NACK_TRANSACTION_CONFLICT,
                    PROTOCOL_DETAIL_NONE);
                return;
            }

            if (sequence.decision != PROTOCOL_SEQUENCE_NEW)
            {
                RecordAndReply((uint8_t)VA_UART_MSG_REMOTE_STOP_REQUEST,
                               message->payload.remote_stop_request.transaction_id,
                               message->payload_crc,
                               false,
                               PROTOCOL_NACK_STALE_SEQUENCE);
                return;
            }

            StateSetRemoteStop(true);
            StateInvalidateControl();
            WatchdogDisarmControl();
            MailboxInvalidate();

            RecordAndReply((uint8_t)VA_UART_MSG_REMOTE_STOP_REQUEST,
                           message->payload.remote_stop_request.transaction_id,
                           message->payload_crc,
                           true,
                           PROTOCOL_ACK_OK);
            break;

        case VA_UART_MSG_CLEAR_REMOTE_STOP:
            if (!RequireReadyPeer(message))
            {
                RecordAndReply((uint8_t)VA_UART_MSG_CLEAR_REMOTE_STOP,
                               message->payload.clear_remote_stop.transaction_id,
                               message->payload_crc,
                               false,
                               PROTOCOL_NACK_NOT_READY);
                return;
            }

            sequence = StateObserveSequence(message->header.wire_sequence);
            if (sequence.decision == PROTOCOL_SEQUENCE_NEW)
            {
                WatchdogRefreshLink();
            }

            cached = va_uart_transaction_cache_lookup(
                &s_transaction_cache,
                (uint8_t)VA_UART_MSG_CLEAR_REMOTE_STOP,
                message->payload.clear_remote_stop.transaction_id,
                message->payload_crc);

            if (cached.decision == VA_UART_CACHE_DUPLICATE)
            {
                ReplyCached((uint8_t)VA_UART_MSG_CLEAR_REMOTE_STOP,
                            message->payload.clear_remote_stop.transaction_id,
                            &cached);
                return;
            }

            if (cached.decision == VA_UART_CACHE_CONFLICTING_DUPLICATE)
            {
                (void)SendAckOrNack(
                    (uint8_t)VA_UART_MSG_CLEAR_REMOTE_STOP,
                    message->payload.clear_remote_stop.transaction_id,
                    false,
                    PROTOCOL_NACK_TRANSACTION_CONFLICT,
                    PROTOCOL_DETAIL_NONE);
                return;
            }

            if (sequence.decision != PROTOCOL_SEQUENCE_NEW)
            {
                RecordAndReply((uint8_t)VA_UART_MSG_CLEAR_REMOTE_STOP,
                               message->payload.clear_remote_stop.transaction_id,
                               message->payload_crc,
                               false,
                               PROTOCOL_NACK_STALE_SEQUENCE);
                return;
            }

            ProtocolCore_GetState(&state);
            allowed = CanClearRemoteStop(
                state.link_state == PROTOCOL_LINK_READY,
                state.remote_stop_latched);

            if (!allowed)
            {
                RecordAndReply((uint8_t)VA_UART_MSG_CLEAR_REMOTE_STOP,
                               message->payload.clear_remote_stop.transaction_id,
                               message->payload_crc,
                               false,
                               PROTOCOL_NACK_CLEAR_NOT_ALLOWED);
                return;
            }

            StateSetRemoteStop(false);
            StateInvalidateControl();
            WatchdogDisarmControl();
            MailboxInvalidate();

            RecordAndReply((uint8_t)VA_UART_MSG_CLEAR_REMOTE_STOP,
                           message->payload.clear_remote_stop.transaction_id,
                           message->payload_crc,
                           true,
                           PROTOCOL_ACK_OK);
            break;

        case VA_UART_MSG_PING:
            if (!RequireReadyPeer(message))
            {
                return;
            }

            sequence = StateObserveSequence(message->header.wire_sequence);
            if (sequence.decision != PROTOCOL_SEQUENCE_NEW)
            {
                return;
            }

            WatchdogRefreshLink();
            (void)SendPong(&message->header, &message->payload.ping);
            break;

        default:
            break;
    }
}

static ProtocolHelloDecision ValidateHello(const va_uart_hello_t *hello)
{
    ProtocolHelloDecision decision = {false, PROTOCOL_HELLO_BAD_ROLE};

    if (hello == NULL)
    {
        return decision;
    }

    if (hello->device_role != PROTOCOL_PEER_DEVICE_ROLE)
    {
        return decision;
    }

    if ((hello->minimum_protocol_version > VA_UART_PROTOCOL_VERSION) ||
        (hello->maximum_protocol_version < VA_UART_PROTOCOL_VERSION))
    {
        decision.result_code = PROTOCOL_HELLO_BAD_VERSION;
        return decision;
    }

    if (hello->max_payload < PROTOCOL_MIN_PEER_MAX_PAYLOAD)
    {
        decision.result_code = PROTOCOL_HELLO_BAD_MAX_PAYLOAD;
        return decision;
    }

    if ((hello->capability_bits & PROTOCOL_REQUIRED_CAPABILITIES) !=
        PROTOCOL_REQUIRED_CAPABILITIES)
    {
        decision.result_code = PROTOCOL_HELLO_MISSING_CAPABILITY;
        return decision;
    }

    decision.accepted = true;
    decision.result_code = PROTOCOL_HELLO_OK;
    return decision;
}

static bool ValidateControl(const va_uart_control_update_t *control)
{
    if (control == NULL)
    {
        return false;
    }

    return (control->target_state == PROTOCOL_TARGET_STATE_DETECTED) &&
           ((control->control_flags & PROTOCOL_CONTROL_FLAG_VALID) != 0U) &&
           (control->capture_age_at_tx_ms <= PROTOCOL_CONTROL_MAX_CAPTURE_AGE_MS);
}

static bool CanClearRemoteStop(bool link_ready, bool remote_stop_latched)
{
    return link_ready && remote_stop_latched;
}

static void StateInit(void)
{
    (void)memset(&s_state, 0, sizeof(s_state));
    s_state.link_state = PROTOCOL_LINK_WAIT_HELLO;
    s_state.mcu_boot_id = GenerateBootId();
}

static void StateAcceptHello(uint32_t peer_boot_id, uint32_t hello_wire_sequence)
{
    s_state.have_peer = true;
    s_state.peer_boot_id = peer_boot_id;
    s_state.sequence_valid = true;
    s_state.last_rx_wire_sequence = hello_wire_sequence;
    s_state.control_valid = false;
    s_state.link_state = PROTOCOL_LINK_READY;
}

static void StateRejectHello(void)
{
    s_state.link_state = PROTOCOL_LINK_WAIT_HELLO;
    s_state.have_peer = false;
    s_state.sequence_valid = false;
    s_state.control_valid = false;
}

static void StatePeerMismatch(void)
{
    StateRejectHello();
}

static void StateLinkTimeout(void)
{
    s_state.link_state = PROTOCOL_LINK_LOST;
    s_state.have_peer = false;
    s_state.sequence_valid = false;
    s_state.control_valid = false;
}

static bool StateIsReadyForPeer(uint32_t peer_boot_id)
{
    return (s_state.link_state == PROTOCOL_LINK_READY) &&
           s_state.have_peer &&
           (s_state.peer_boot_id == peer_boot_id);
}

static ProtocolSequenceResult StateObserveSequence(uint32_t wire_sequence)
{
    ProtocolSequenceResult result = {PROTOCOL_SEQUENCE_NEW, 0U};
    uint32_t difference;

    if (!s_state.sequence_valid)
    {
        s_state.sequence_valid = true;
        s_state.last_rx_wire_sequence = wire_sequence;
        return result;
    }

    difference = (uint32_t)(wire_sequence - s_state.last_rx_wire_sequence);

    if (difference == 0U)
    {
        result.decision = PROTOCOL_SEQUENCE_DUPLICATE;
        return result;
    }

    if (difference < 0x80000000UL)
    {
        result.gap_count = difference - 1U;
        s_state.sequence_gap_count += result.gap_count;
        s_state.last_rx_wire_sequence = wire_sequence;
        return result;
    }

    result.decision = PROTOCOL_SEQUENCE_OLD;
    return result;
}

static void StateSetControlValid(uint32_t wire_sequence)
{
    s_state.control_valid = true;
    s_state.last_control_wire_sequence = wire_sequence;
}

static void StateInvalidateControl(void)
{
    s_state.control_valid = false;
}

static void StateSetRemoteStop(bool latched)
{
    s_state.remote_stop_latched = latched;
    if (latched)
    {
        s_state.control_valid = false;
    }
}

static void MailboxInit(void)
{
    (void)memset(&s_command, 0, sizeof(s_command));
    s_mailbox_overwrite_count = 0U;
}

static void MailboxPublish(uint32_t wire_sequence,
                           const va_uart_control_update_t *control,
                           bool valid)
{
    if (control == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();

    if (s_command.generation != 0U)
    {
        s_mailbox_overwrite_count++;
    }

    s_command.generation++;
    if (s_command.generation == 0U)
    {
        s_command.generation = 1U;
    }

    s_command.valid = valid;
    s_command.wire_sequence = wire_sequence;
    s_command.control = *control;

    taskEXIT_CRITICAL();
}

static void MailboxInvalidate(void)
{
    taskENTER_CRITICAL();
    s_command.valid = false;
    taskEXIT_CRITICAL();
}

static void WatchdogInit(void)
{
    s_link_watchdog_armed = false;
    s_control_watchdog_armed = false;
    s_last_link_tick = 0U;
    s_last_control_tick = 0U;
}

static void WatchdogOnHelloAccepted(void)
{
    s_link_watchdog_armed = true;
    s_last_link_tick = xTaskGetTickCount();
    s_control_watchdog_armed = false;
}

static void WatchdogRefreshLink(void)
{
    s_link_watchdog_armed = true;
    s_last_link_tick = xTaskGetTickCount();
}

static void WatchdogRefreshControl(void)
{
    s_control_watchdog_armed = true;
    s_last_control_tick = xTaskGetTickCount();
}

static void WatchdogDisarmControl(void)
{
    s_control_watchdog_armed = false;
}

static void WatchdogDisarmAll(void)
{
    s_link_watchdog_armed = false;
    s_control_watchdog_armed = false;
}

static void WatchdogCheck(void)
{
    TickType_t now = xTaskGetTickCount();

    if (s_link_watchdog_armed &&
        ((TickType_t)(now - s_last_link_tick) >=
         pdMS_TO_TICKS(APP_LINK_WATCHDOG_TIMEOUT_MS)))
    {
        s_link_watchdog_armed = false;
        s_control_watchdog_armed = false;
        StateLinkTimeout();
        MailboxInvalidate();
        return;
    }

    if (s_control_watchdog_armed &&
        ((TickType_t)(now - s_last_control_tick) >=
         pdMS_TO_TICKS(APP_CONTROL_WATCHDOG_TIMEOUT_MS)))
    {
        s_control_watchdog_armed = false;
        StateInvalidateControl();
        MailboxInvalidate();
    }
}

static bool RequireReadyPeer(const ProtocolMessage *message)
{
    ProtocolStateSnapshot state;

    if (message == NULL)
    {
        return false;
    }

    ProtocolCore_GetState(&state);
    if (state.link_state != PROTOCOL_LINK_READY)
    {
        return false;
    }

    if (!StateIsReadyForPeer(message->header.sender_boot_id))
    {
        StatePeerMismatch();
        StateInvalidateControl();
        MailboxInvalidate();
        WatchdogDisarmAll();
        return false;
    }

    return true;
}

static void ReplyCached(uint8_t request_type,
                        uint32_t transaction_id,
                        const va_uart_cached_result_t *cached)
{
    if (cached != NULL)
    {
        (void)SendAckOrNack(request_type,
                            transaction_id,
                            cached->acknowledged,
                            cached->result_code,
                            cached->detail_code);
    }
}

static void RecordAndReply(uint8_t request_type,
                           uint32_t transaction_id,
                           uint16_t payload_crc,
                           bool acknowledged,
                           uint16_t result_code)
{
    va_uart_transaction_cache_record(&s_transaction_cache,
                                     request_type,
                                     transaction_id,
                                     payload_crc,
                                     acknowledged,
                                     result_code,
                                     PROTOCOL_DETAIL_NONE);

    (void)SendAckOrNack(request_type,
                        transaction_id,
                        acknowledged,
                        result_code,
                        PROTOCOL_DETAIL_NONE);
}

static bool SendHelloAck(const va_uart_header_t *request_header,
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

    return ProtocolTransport_QueueResponse(
        (uint8_t)VA_UART_MSG_HELLO_ACK, payload, payload_size);
}

static bool SendStatus(void)
{
    va_uart_status_t status = {0};
    ProtocolStateSnapshot state;
    ProtocolRxStats rx_stats;
    GimbalRuntimeSnapshot gimbal;
    uint8_t payload[52U];
    size_t payload_size;

    ProtocolCore_GetState(&state);
    ProtocolTransport_GetRxStats(&rx_stats);
    GimbalTask_GetSnapshot(&gimbal);

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
    status.rx_overflow_count = ProtocolTransport_GetRxOverflowCount();
    status.control_mailbox_overwrite_count = ProtocolCore_GetMailboxOverwriteCount();
    status.mcu_tick_ms = HAL_GetTick();
    status.pan_stub_q15 = gimbal.pan_applied_q15;
    status.tilt_stub_q15 = gimbal.tilt_applied_q15;

    if (!va_uart_encode_status(&status, payload, sizeof(payload), &payload_size))
    {
        return false;
    }

    return ProtocolTransport_QueueResponse(
        (uint8_t)VA_UART_MSG_STATUS, payload, payload_size);
}

static bool SendPong(const va_uart_header_t *request_header,
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

    return ProtocolTransport_QueueResponse(
        (uint8_t)VA_UART_MSG_PONG, payload, payload_size);
}

static bool SendAckOrNack(uint8_t request_message_type,
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

    return ProtocolTransport_QueueResponse(response_type, payload, payload_size);
}

static uint32_t GenerateBootId(void)
{
    const volatile uint32_t *uid0 =
        (const volatile uint32_t *)STM32F1_UID_WORD0_ADDRESS;
    const volatile uint32_t *uid1 =
        (const volatile uint32_t *)STM32F1_UID_WORD1_ADDRESS;
    const volatile uint32_t *uid2 =
        (const volatile uint32_t *)STM32F1_UID_WORD2_ADDRESS;
    uint32_t seed;

    seed = *uid0;
    seed ^= (*uid1 << 7U) | (*uid1 >> 25U);
    seed ^= (*uid2 << 17U) | (*uid2 >> 15U);
    seed ^= HAL_GetTick();
    seed ^= DWT->CYCCNT;
    seed = Mix32(seed);

    return (seed != 0U) ? seed : 0x5641524DUL; /* "VARM" */
}

static uint32_t Mix32(uint32_t value)
{
    value ^= value >> 16U;
    value *= 0x7FEB352DUL;
    value ^= value >> 15U;
    value *= 0x846CA68BUL;
    value ^= value >> 16U;
    return value;
}
