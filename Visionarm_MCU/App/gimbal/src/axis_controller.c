#include "axis_controller.h"

#include <stddef.h>

static bool ConfigIsValid(const AxisControllerConfig *config);
static int32_t ClampS32(int32_t value, int32_t minimum, int32_t maximum);
static int32_t AbsS32(int32_t value);

bool AxisController_Init(AxisController *controller,
                         const AxisControllerConfig *config)
{
    if ((controller == NULL) || !ConfigIsValid(config))
    {
        return false;
    }

    *controller = (AxisController){0};
    controller->target_us = (int32_t)config->center_us;

    return true;
}

void AxisController_ResetCenter(AxisController *controller,
                                const AxisControllerConfig *config)
{
    if ((controller == NULL) || !ConfigIsValid(config))
    {
        return;
    }

    controller->target_us = (int32_t)config->center_us;
    controller->last_input_q15 = 0;
    controller->signed_error_q15 = 0;
    controller->requested_delta_us = 0;
    controller->applied_delta_us = 0;
    controller->dead_zone_active = true;
    controller->limit_blocked = false;
}

bool AxisController_Step(AxisController *controller,
                         const AxisControllerConfig *config,
                         int16_t error_q15,
                         AxisControllerStepResult *result)
{
    AxisControllerStepResult local_result = {0};
    int32_t signed_error;
    int32_t requested_delta;
    int32_t slew_delta;
    int32_t old_target;
    int32_t new_target;
    bool slew_limited;
    bool limit_blocked;

    if ((controller == NULL) || !ConfigIsValid(config))
    {
        return false;
    }

    signed_error = (int32_t)error_q15;
    if (config->invert_error)
    {
        signed_error = -signed_error;
    }

    requested_delta = 0;
    slew_delta = 0;
    slew_limited = false;
    limit_blocked = false;

    local_result.dead_zone_active =
        (AbsS32(signed_error) <= (int32_t)config->dead_zone_q15);

    if (!local_result.dead_zone_active)
    {
        requested_delta =
            (signed_error *
             (int32_t)config->gain_us_per_full_scale_cycle) /
            32767L;

        /*
         * Integer math must not create a second, accidental dead-zone.
         * Once the configured dead-zone is exceeded, guarantee at least
         * one microsecond of requested motion in the correct direction.
         */
        if (requested_delta == 0)
        {
            requested_delta = (signed_error > 0) ? 1L : -1L;
        }

        slew_delta = ClampS32(
            requested_delta,
            -(int32_t)config->max_delta_us_per_cycle,
             (int32_t)config->max_delta_us_per_cycle);

        slew_limited = (slew_delta != requested_delta);
    }

    old_target = controller->target_us;
    new_target = old_target + slew_delta;

    if (new_target < (int32_t)config->min_us)
    {
        new_target = (int32_t)config->min_us;
        limit_blocked = (slew_delta < 0);
    }
    else if (new_target > (int32_t)config->max_us)
    {
        new_target = (int32_t)config->max_us;
        limit_blocked = (slew_delta > 0);
    }

    controller->target_us = new_target;
    controller->last_input_q15 = error_q15;
    controller->signed_error_q15 = signed_error;
    controller->requested_delta_us = requested_delta;
    controller->applied_delta_us = new_target - old_target;
    controller->dead_zone_active = local_result.dead_zone_active;
    controller->limit_blocked = limit_blocked;
    controller->update_count++;

    if (local_result.dead_zone_active)
    {
        controller->dead_zone_count++;
    }

    if (slew_limited)
    {
        controller->slew_limit_count++;
    }

    if (limit_blocked)
    {
        controller->limit_hit_count++;
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

bool AxisController_RunSelfTest(void)
{
    const AxisControllerConfig config =
    {
        1000U,
        1500U,
        2000U,
        2048,
        4U,
        2U,
        false
    };

    AxisController controller;
    AxisControllerStepResult result;
    uint32_t i;

    if (!AxisController_Init(&controller, &config))
    {
        return false;
    }

    /* Configured dead-zone must produce no motion. */
    if (!AxisController_Step(&controller, &config, 1024, &result) ||
        !result.dead_zone_active ||
        (result.target_us != 1500U))
    {
        return false;
    }

    /* Quarter-scale positive error must move in the positive pulse direction. */
    if (!AxisController_Step(&controller, &config, 8192, &result) ||
        result.dead_zone_active ||
        (result.applied_delta_us != 1L) ||
        (result.target_us != 1501U))
    {
        return false;
    }

    /* Full-scale request is slew-limited to two microseconds per cycle. */
    if (!AxisController_Step(&controller, &config, 32767, &result) ||
        !result.slew_limited ||
        (result.applied_delta_us != 2L))
    {
        return false;
    }

    /* Drive to the configured upper limit and prove no overshoot. */
    for (i = 0U; i < 300U; ++i)
    {
        if (!AxisController_Step(&controller, &config, 32767, &result))
        {
            return false;
        }
    }

    if ((result.target_us != 2000U) || !result.limit_blocked)
    {
        return false;
    }

    /* Reverse error must be allowed to leave the limit immediately. */
    if (!AxisController_Step(&controller, &config, -32768, &result) ||
        (result.applied_delta_us >= 0L) ||
        result.limit_blocked)
    {
        return false;
    }

    AxisController_ResetCenter(&controller, &config);
    if (controller.target_us != 1500L)
    {
        return false;
    }

    return true;
}

static bool ConfigIsValid(const AxisControllerConfig *config)
{
    if (config == NULL)
    {
        return false;
    }

    if ((config->min_us >= config->center_us) ||
        (config->center_us >= config->max_us))
    {
        return false;
    }

    if ((config->dead_zone_q15 < 0) ||
        (config->dead_zone_q15 >= 32767))
    {
        return false;
    }

    if ((config->gain_us_per_full_scale_cycle == 0U) ||
        (config->max_delta_us_per_cycle == 0U))
    {
        return false;
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
