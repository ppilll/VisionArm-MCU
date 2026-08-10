#include "protocol_message.h"

#include <string.h>

bool ProtocolMessage_Decode(const va_uart_frame_t *frame,
                            ProtocolMessage *message)
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
            /* MCU response-only message types are never valid inputs. */
            ok = false;
            break;
    }

    return ok;
}
