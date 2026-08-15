#include "gimbal_safety.h"

#include <stddef.h>

static GimbalSafetyReason EvaluateBaseReason(const GimbalSafetyInput *input);
static void BlockCurrentGeneration(GimbalSafetyContext *context,
                                   const GimbalSafetyInput *input);
static void RecordReason(GimbalSafetyContext *context,
                         GimbalSafetyReason reason);

void GimbalSafety_Init(GimbalSafetyContext *context)
{
    if (context == NULL)
    {
        return;
    }

    context->require_fresh_control = true;
    context->blocked_generation = 0U;
    context->accepted_generation = 0U;
    context->last_reason = GIMBAL_SAFETY_BOOT_SAFE;
    context->transition_count = 0U;
}

GimbalSafetyDecision GimbalSafety_Evaluate(
    GimbalSafetyContext *context,
    const GimbalSafetyInput *input)
{
    GimbalSafetyDecision decision = {0};
    GimbalSafetyReason reason;

    decision.reason = GIMBAL_SAFETY_LOCAL_FAULT;

    if ((context == NULL) || (input == NULL))
    {
        return decision;
    }

    decision.mailbox_generation = input->mailbox_generation;
    decision.accepted_generation = context->accepted_generation;

    reason = EvaluateBaseReason(input);
    if (reason != GIMBAL_SAFETY_ENABLED)
    {
        BlockCurrentGeneration(context, input);
        RecordReason(context, reason);

        decision.reason = reason;
        decision.waiting_for_fresh_control = context->require_fresh_control;
        return decision;
    }

    if (context->require_fresh_control &&
        (input->mailbox_generation == context->blocked_generation))
    {
        RecordReason(context, GIMBAL_SAFETY_WAIT_FRESH_CONTROL);

        decision.reason = GIMBAL_SAFETY_WAIT_FRESH_CONTROL;
        decision.waiting_for_fresh_control = true;
        return decision;
    }

    context->require_fresh_control = false;
    context->accepted_generation = input->mailbox_generation;
    RecordReason(context, GIMBAL_SAFETY_ENABLED);

    decision.control_enabled = true;
    decision.reason = GIMBAL_SAFETY_ENABLED;
    decision.accepted_generation = context->accepted_generation;
    decision.waiting_for_fresh_control = false;

    return decision;
}

bool GimbalSafety_RunSelfTest(void)
{
    GimbalSafetyContext context;
    GimbalSafetyInput input = {0};
    GimbalSafetyDecision decision;

    GimbalSafety_Init(&context);

#define EXPECT_REASON(expected_reason)                                     \
    do                                                                     \
    {                                                                      \
        decision = GimbalSafety_Evaluate(&context, &input);                \
        if (decision.reason != (expected_reason))                          \
        {                                                                  \
            return false;                                                  \
        }                                                                  \
    } while (0)

    /* Nothing initialized: boot-safe has priority. */
    EXPECT_REASON(GIMBAL_SAFETY_BOOT_SAFE);

    input.actuator_initialized = true;
    EXPECT_REASON(GIMBAL_SAFETY_LINK_NOT_READY);

    /* Remote stop dominates link/control state. */
    input.remote_stop_latched = true;
    EXPECT_REASON(GIMBAL_SAFETY_REMOTE_STOP);

    input.remote_stop_latched = false;
    input.link_ready = true;
    EXPECT_REASON(GIMBAL_SAFETY_CONTROL_INVALID);

    input.control_valid = true;
    EXPECT_REASON(GIMBAL_SAFETY_MAILBOX_UNAVAILABLE);

    input.mailbox_available = true;
    input.mailbox_generation = 4U;
    EXPECT_REASON(GIMBAL_SAFETY_MAILBOX_INVALID);

    /* Same generation that was blocked may not reactivate output. */
    input.mailbox_valid = true;
    EXPECT_REASON(GIMBAL_SAFETY_WAIT_FRESH_CONTROL);

    input.mailbox_generation = 5U;
    EXPECT_REASON(GIMBAL_SAFETY_ENABLED);
    if (!decision.control_enabled ||
        (decision.accepted_generation != 5U))
    {
        return false;
    }

    /* A stable valid snapshot remains enabled without requiring new data. */
    EXPECT_REASON(GIMBAL_SAFETY_ENABLED);

    /* Remote stop blocks generation 5. Clear alone must not resume it. */
    input.remote_stop_latched = true;
    EXPECT_REASON(GIMBAL_SAFETY_REMOTE_STOP);

    input.remote_stop_latched = false;
    EXPECT_REASON(GIMBAL_SAFETY_WAIT_FRESH_CONTROL);

    input.mailbox_generation = 6U;
    EXPECT_REASON(GIMBAL_SAFETY_ENABLED);

    /* Local fault has the highest software priority and also blocks resume. */
    input.local_fault = true;
    input.remote_stop_latched = true;
    EXPECT_REASON(GIMBAL_SAFETY_LOCAL_FAULT);

    input.local_fault = false;
    EXPECT_REASON(GIMBAL_SAFETY_REMOTE_STOP);

    input.remote_stop_latched = false;
    EXPECT_REASON(GIMBAL_SAFETY_WAIT_FRESH_CONTROL);

    input.mailbox_generation = 7U;
    EXPECT_REASON(GIMBAL_SAFETY_ENABLED);

#undef EXPECT_REASON

    return true;
}

const char *GimbalSafety_ReasonName(GimbalSafetyReason reason)
{
    switch (reason)
    {
        case GIMBAL_SAFETY_BOOT_SAFE:
            return "BOOT_SAFE";
        case GIMBAL_SAFETY_LOCAL_FAULT:
            return "LOCAL_FAULT";
        case GIMBAL_SAFETY_REMOTE_STOP:
            return "REMOTE_STOP";
        case GIMBAL_SAFETY_LINK_NOT_READY:
            return "LINK_NOT_READY";
        case GIMBAL_SAFETY_CONTROL_INVALID:
            return "CONTROL_INVALID";
        case GIMBAL_SAFETY_MAILBOX_UNAVAILABLE:
            return "MAILBOX_UNAVAILABLE";
        case GIMBAL_SAFETY_MAILBOX_INVALID:
            return "MAILBOX_INVALID";
        case GIMBAL_SAFETY_WAIT_FRESH_CONTROL:
            return "WAIT_FRESH_CONTROL";
        case GIMBAL_SAFETY_ENABLED:
            return "ENABLED";
        default:
            return "UNKNOWN";
    }
}

static GimbalSafetyReason EvaluateBaseReason(const GimbalSafetyInput *input)
{
    if (!input->actuator_initialized)
    {
        return GIMBAL_SAFETY_BOOT_SAFE;
    }

    if (input->local_fault)
    {
        return GIMBAL_SAFETY_LOCAL_FAULT;
    }

    if (input->remote_stop_latched)
    {
        return GIMBAL_SAFETY_REMOTE_STOP;
    }

    if (!input->link_ready)
    {
        return GIMBAL_SAFETY_LINK_NOT_READY;
    }

    if (!input->control_valid)
    {
        return GIMBAL_SAFETY_CONTROL_INVALID;
    }

    if (!input->mailbox_available)
    {
        return GIMBAL_SAFETY_MAILBOX_UNAVAILABLE;
    }

    if (!input->mailbox_valid)
    {
        return GIMBAL_SAFETY_MAILBOX_INVALID;
    }

    return GIMBAL_SAFETY_ENABLED;
}

static void BlockCurrentGeneration(GimbalSafetyContext *context,
                                   const GimbalSafetyInput *input)
{
    context->require_fresh_control = true;

    if (input->mailbox_available &&
        (input->mailbox_generation != 0U))
    {
        context->blocked_generation = input->mailbox_generation;
    }
}

static void RecordReason(GimbalSafetyContext *context,
                         GimbalSafetyReason reason)
{
    if (context->last_reason != reason)
    {
        context->last_reason = reason;
        context->transition_count++;
    }
}
