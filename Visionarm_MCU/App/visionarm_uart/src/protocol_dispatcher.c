#include "protocol_dispatcher.h"

#include "control_mailbox.h"
#include "protocol_policy.h"
#include "protocol_response.h"
#include "protocol_state.h"
#include "protocol_watchdog.h"
#include "visionarm_uart_c/transaction_cache.h"

static va_uart_transaction_cache_t s_transaction_cache;

static bool RequireReadyPeer(const ProtocolMessage *message);
static void ReplyCached(uint8_t request_type,
                        uint32_t transaction_id,
                        const va_uart_cached_result_t *cached);
static void RecordAndReply(uint8_t request_type,
                           uint32_t transaction_id,
                           uint16_t payload_crc,
                           bool acknowledged,
                           uint16_t result_code);

void ProtocolDispatcher_Init(void)
{
    va_uart_transaction_cache_reset(&s_transaction_cache);
}

void ProtocolDispatcher_OnMessage(const ProtocolMessage *message)
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
            hello_decision = ProtocolPolicy_ValidateHello(&message->payload.hello);

            if (hello_decision.accepted)
            {
                ProtocolState_AcceptHello(message->header.sender_boot_id,
                                          message->header.wire_sequence);
                va_uart_transaction_cache_reset(&s_transaction_cache);
                ProtocolState_InvalidateControl();
                ControlMailbox_Invalidate();
                ProtocolWatchdog_OnHelloAccepted();
            }
            else
            {
                ProtocolState_RejectHello();
                ControlMailbox_Invalidate();
                ProtocolWatchdog_DisarmAll();
            }

            (void)ProtocolResponse_SendHelloAck(&message->header,
                                                hello_decision.accepted,
                                                hello_decision.result_code);
            break;

        case VA_UART_MSG_HEARTBEAT:
            if (!RequireReadyPeer(message))
            {
                return;
            }

            sequence = ProtocolState_ObserveSequence(message->header.wire_sequence);
            if (sequence.decision != PROTOCOL_SEQUENCE_NEW)
            {
                return;
            }

            ProtocolWatchdog_RefreshLink();
            (void)ProtocolResponse_SendStatus();
            break;

        case VA_UART_MSG_CONTROL_UPDATE:
            if (!RequireReadyPeer(message))
            {
                ProtocolState_InvalidateControl();
                ProtocolWatchdog_DisarmControl();
                ControlMailbox_Publish(message->header.wire_sequence,
                                       &message->payload.control_update,
                                       false);
                return;
            }

            sequence = ProtocolState_ObserveSequence(message->header.wire_sequence);
            if (sequence.decision == PROTOCOL_SEQUENCE_NEW)
            {
                ProtocolWatchdog_RefreshLink();
            }

            ProtocolState_GetSnapshot(&state);
            valid = (sequence.decision == PROTOCOL_SEQUENCE_NEW) &&
                    !state.remote_stop_latched &&
                    ProtocolPolicy_ValidateControl(&message->payload.control_update);

            ControlMailbox_Publish(message->header.wire_sequence,
                                   &message->payload.control_update,
                                   valid);

            if (valid)
            {
                ProtocolState_SetControlValid(message->header.wire_sequence);
                ProtocolWatchdog_RefreshControl();
            }
            else
            {
                ProtocolState_InvalidateControl();
                ProtocolWatchdog_DisarmControl();
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

            sequence = ProtocolState_ObserveSequence(message->header.wire_sequence);
            if (sequence.decision == PROTOCOL_SEQUENCE_NEW)
            {
                ProtocolWatchdog_RefreshLink();
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
                (void)ProtocolResponse_SendAckOrNack(
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

            ProtocolState_SetRemoteStop(true);
            ProtocolState_InvalidateControl();
            ProtocolWatchdog_DisarmControl();
            ControlMailbox_Invalidate();

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

            sequence = ProtocolState_ObserveSequence(message->header.wire_sequence);
            if (sequence.decision == PROTOCOL_SEQUENCE_NEW)
            {
                ProtocolWatchdog_RefreshLink();
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
                (void)ProtocolResponse_SendAckOrNack(
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

            ProtocolState_GetSnapshot(&state);
            allowed = ProtocolPolicy_CanClearRemoteStop(
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

            ProtocolState_SetRemoteStop(false);
            ProtocolState_InvalidateControl();
            ProtocolWatchdog_DisarmControl();
            ControlMailbox_Invalidate();

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

            sequence = ProtocolState_ObserveSequence(message->header.wire_sequence);
            if (sequence.decision != PROTOCOL_SEQUENCE_NEW)
            {
                return;
            }

            ProtocolWatchdog_RefreshLink();
            (void)ProtocolResponse_SendPong(&message->header,
                                            &message->payload.ping);
            break;

        default:
            break;
    }
}

static bool RequireReadyPeer(const ProtocolMessage *message)
{
    ProtocolStateSnapshot state;

    if (message == NULL)
    {
        return false;
    }

    ProtocolState_GetSnapshot(&state);
    if (state.link_state != PROTOCOL_LINK_READY)
    {
        return false;
    }

    if (!ProtocolState_IsReadyForPeer(message->header.sender_boot_id))
    {
        ProtocolState_PeerMismatch();
        ProtocolState_InvalidateControl();
        ControlMailbox_Invalidate();
        ProtocolWatchdog_DisarmAll();
        return false;
    }

    return true;
}

static void ReplyCached(uint8_t request_type,
                        uint32_t transaction_id,
                        const va_uart_cached_result_t *cached)
{
    if (cached == NULL)
    {
        return;
    }

    (void)ProtocolResponse_SendAckOrNack(request_type,
                                         transaction_id,
                                         cached->acknowledged,
                                         cached->result_code,
                                         cached->detail_code);
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

    (void)ProtocolResponse_SendAckOrNack(request_type,
                                         transaction_id,
                                         acknowledged,
                                         result_code,
                                         PROTOCOL_DETAIL_NONE);
}
