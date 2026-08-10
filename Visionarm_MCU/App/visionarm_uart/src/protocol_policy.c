#include "protocol_policy.h"

ProtocolHelloDecision ProtocolPolicy_ValidateHello(const va_uart_hello_t *hello)
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

bool ProtocolPolicy_ValidateControl(const va_uart_control_update_t *control)
{
    if (control == NULL)
    {
        return false;
    }

    if (control->target_state != PROTOCOL_TARGET_STATE_DETECTED)
    {
        return false;
    }

    if ((control->control_flags & PROTOCOL_CONTROL_FLAG_VALID) == 0U)
    {
        return false;
    }

    return (control->capture_age_at_tx_ms <=
            PROTOCOL_CONTROL_MAX_CAPTURE_AGE_MS);
}

bool ProtocolPolicy_CanClearRemoteStop(bool link_ready,
                                       bool remote_stop_latched)
{
    return link_ready && remote_stop_latched;
}
