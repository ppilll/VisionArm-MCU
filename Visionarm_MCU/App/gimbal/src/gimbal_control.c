#include "gimbal_control.h"

#include <stddef.h>

typedef struct
{
    uint16_t min_us;
    uint16_t center_us;
    uint16_t max_us;
    int16_t dead_zone_q15;
    uint16_t gain_us_per_full_scale_cycle;
    uint16_t max_delta_us_per_cycle;
    bool invert_error;
} AxisConfig;

static const AxisConfig s_pan_config =
{
    ACTUATOR_PAN_SAFE_MIN_US,
    ACTUATOR_PAN_CENTER_US,
    ACTUATOR_PAN_SAFE_MAX_US,
    GIMBAL_PAN_DEAD_ZONE_Q15,
    GIMBAL_PAN_GAIN_US_PER_FULL_SCALE,
    GIMBAL_PAN_MAX_DELTA_US_PER_CYCLE,
    (GIMBAL_PAN_ERROR_INVERT != 0U)
};

static const AxisConfig s_tilt_config =
{
    ACTUATOR_TILT_SAFE_MIN_US,
    ACTUATOR_TILT_CENTER_US,
    ACTUATOR_TILT_SAFE_MAX_US,
    GIMBAL_TILT_DEAD_ZONE_Q15,
    GIMBAL_TILT_GAIN_US_PER_FULL_SCALE,
    GIMBAL_TILT_MAX_DELTA_US_PER_CYCLE,
    (GIMBAL_TILT_ERROR_INVERT != 0U)
};

static bool AxisConfigIsValid(const AxisConfig *config);
static bool AxisInit(GimbalAxisState *axis, const AxisConfig *config);
static void AxisResetCenter(GimbalAxisState *axis, const AxisConfig *config);
static bool AxisStep(GimbalAxisState *axis,
                     const AxisConfig *config,
                     int16_t error_q15,
                     GimbalAxisResult *result);
static int32_t ClampS32(int32_t value, int32_t minimum, int32_t maximum);
static int32_t AbsS32(int32_t value);
static GimbalSafetyReason EvaluateSafetyBase(const GimbalSafetyInput *input);
static void BlockCurrentGeneration(GimbalSafetyContext *context,
                                   const GimbalSafetyInput *input);
static void RecordSafetyReason(GimbalSafetyContext *context,
                               GimbalSafetyReason reason);

void GimbalSafety_Init(GimbalSafetyContext *context)
{
    if (context == NULL)
    {
        return;
    }

    *context = (GimbalSafetyContext){0};
    context->require_fresh_control = true;
    context->last_reason = GIMBAL_SAFETY_BOOT_SAFE;
}

GimbalSafetyDecision GimbalSafety_Evaluate(GimbalSafetyContext *context,
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

    reason = EvaluateSafetyBase(input);
    if (reason != GIMBAL_SAFETY_ENABLED)
    {
        BlockCurrentGeneration(context, input);
        RecordSafetyReason(context, reason);
        decision.reason = reason;
        decision.waiting_for_fresh_control = context->require_fresh_control;
        return decision;
    }

    if (context->require_fresh_control &&
        (input->mailbox_generation == context->blocked_generation))
    {
        RecordSafetyReason(context, GIMBAL_SAFETY_WAIT_FRESH_CONTROL);
        decision.reason = GIMBAL_SAFETY_WAIT_FRESH_CONTROL;
        decision.waiting_for_fresh_control = true;
        return decision;
    }

    context->require_fresh_control = false;
    context->accepted_generation = input->mailbox_generation;
    RecordSafetyReason(context, GIMBAL_SAFETY_ENABLED);

    decision.control_enabled = true;
    decision.reason = GIMBAL_SAFETY_ENABLED;
    decision.accepted_generation = context->accepted_generation;
    return decision;
}

bool GimbalControl_Init(GimbalControlState *control)
{
    if (control == NULL)
    {
        return false;
    }

    *control = (GimbalControlState){0};

    return AxisInit(&control->pan, &s_pan_config) &&
           AxisInit(&control->tilt, &s_tilt_config);
}

void GimbalControl_ResetCenter(GimbalControlState *control)
{
    if (control == NULL)
    {
        return;
    }

    AxisResetCenter(&control->pan, &s_pan_config);
    AxisResetCenter(&control->tilt, &s_tilt_config);
}

bool GimbalControl_Step(GimbalControlState *control,
                        int16_t error_x_q15,
                        int16_t error_y_q15,
                        GimbalControlOutput *output)
{
    GimbalControlOutput local_output = {0};

    if (control == NULL)
    {
        return false;
    }

    if (!AxisStep(&control->pan,
                  &s_pan_config,
                  error_x_q15,
                  &local_output.pan) ||
        !AxisStep(&control->tilt,
                  &s_tilt_config,
                  error_y_q15,
                  &local_output.tilt))
    {
        return false;
    }

    local_output.command.pan_position_us = (int32_t)local_output.pan.target_us;
    local_output.command.tilt_position_us = (int32_t)local_output.tilt.target_us;
    control->step_count++;

    if (output != NULL)
    {
        *output = local_output;
    }

    return true;
}

static bool AxisConfigIsValid(const AxisConfig *config)
{
    if (config == NULL)
    {
        return false;
    }

    return (config->min_us < config->center_us) &&
           (config->center_us < config->max_us) &&
           (config->dead_zone_q15 >= 0) &&
           (config->dead_zone_q15 < 32767) &&
           (config->gain_us_per_full_scale_cycle > 0U) &&
           (config->max_delta_us_per_cycle > 0U);
}

static bool AxisInit(GimbalAxisState *axis, const AxisConfig *config)
{
    if ((axis == NULL) || !AxisConfigIsValid(config))
    {
        return false;
    }

    *axis = (GimbalAxisState){0};
    axis->target_us = (int32_t)config->center_us;
    axis->dead_zone_active = true;
    return true;
}

static void AxisResetCenter(GimbalAxisState *axis, const AxisConfig *config)
{
    if ((axis == NULL) || !AxisConfigIsValid(config))
    {
        return;
    }

    axis->target_us = (int32_t)config->center_us;
    axis->last_input_q15 = 0;
    axis->signed_error_q15 = 0;
    axis->requested_delta_us = 0;
    axis->applied_delta_us = 0;
    axis->dead_zone_active = true;
    axis->limit_blocked = false;
}

static bool AxisStep(GimbalAxisState *axis,
                     const AxisConfig *config,
                     int16_t error_q15,
                     GimbalAxisResult *result)
{
    GimbalAxisResult local_result = {0};
    int32_t signed_error;
    int32_t requested_delta = 0;
    int32_t slew_delta = 0;
    int32_t old_target;
    int32_t new_target;
    bool slew_limited = false;
    bool limit_blocked = false;

    if ((axis == NULL) || !AxisConfigIsValid(config))
    {
        return false;
    }

    signed_error = (int32_t)error_q15;
    if (config->invert_error)
    {
        signed_error = -signed_error;
    }

    local_result.dead_zone_active =
        (AbsS32(signed_error) <= (int32_t)config->dead_zone_q15);

    if (!local_result.dead_zone_active)
    {
        requested_delta =
            (signed_error * (int32_t)config->gain_us_per_full_scale_cycle) /
            32767L;

        /* Do not let integer division create an unintended second dead-zone. */
        if (requested_delta == 0L)
        {
            requested_delta = (signed_error > 0L) ? 1L : -1L;
        }

        slew_delta = ClampS32(
            requested_delta,
            -(int32_t)config->max_delta_us_per_cycle,
             (int32_t)config->max_delta_us_per_cycle);
        slew_limited = (slew_delta != requested_delta);
    }

    old_target = axis->target_us;
    new_target = old_target + slew_delta;

    if (new_target < (int32_t)config->min_us)
    {
        new_target = (int32_t)config->min_us;
        limit_blocked = (slew_delta < 0L);
    }
    else if (new_target > (int32_t)config->max_us)
    {
        new_target = (int32_t)config->max_us;
        limit_blocked = (slew_delta > 0L);
    }

    axis->target_us = new_target;
    axis->last_input_q15 = error_q15;
    axis->signed_error_q15 = signed_error;
    axis->requested_delta_us = requested_delta;
    axis->applied_delta_us = new_target - old_target;
    axis->dead_zone_active = local_result.dead_zone_active;
    axis->limit_blocked = limit_blocked;
    axis->update_count++;

    if (local_result.dead_zone_active)
    {
        axis->dead_zone_count++;
    }
    if (slew_limited)
    {
        axis->slew_limit_count++;
    }
    if (limit_blocked)
    {
        axis->limit_hit_count++;
    }

    local_result.target_us = (uint16_t)new_target;
    local_result.signed_error_q15 = signed_error;
    local_result.requested_delta_us = requested_delta;
    local_result.applied_delta_us = new_target - old_target;
    local_result.slew_limited = slew_limited;
    local_result.limit_blocked = limit_blocked;

    if (result != NULL)
    {
        *result = local_result;
    }

    return true;
}

static int32_t ClampS32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static int32_t AbsS32(int32_t value)
{
    return (value < 0L) ? -value : value;
}

static GimbalSafetyReason EvaluateSafetyBase(const GimbalSafetyInput *input)
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

    if (input->mailbox_available && (input->mailbox_generation != 0U))
    {
        context->blocked_generation = input->mailbox_generation;
    }
}

static void RecordSafetyReason(GimbalSafetyContext *context,
                               GimbalSafetyReason reason)
{
    if (context->last_reason != reason)
    {
        context->last_reason = reason;
        context->transition_count++;
    }
}
