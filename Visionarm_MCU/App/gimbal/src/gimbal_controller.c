#include "gimbal_controller.h"

#include <stddef.h>

#include "gimbal_config.h"

static const AxisControllerConfig s_pan_config =
{
    GIMBAL_PAN_SAFE_MIN_US,
    GIMBAL_PAN_CENTER_US,
    GIMBAL_PAN_SAFE_MAX_US,
    GIMBAL_PAN_DEAD_ZONE_Q15,
    GIMBAL_PAN_GAIN_US_PER_FULL_SCALE,
    GIMBAL_PAN_MAX_DELTA_US_PER_CYCLE,
    (GIMBAL_PAN_ERROR_INVERT != 0U)
};

static const AxisControllerConfig s_tilt_config =
{
    GIMBAL_TILT_SAFE_MIN_US,
    GIMBAL_TILT_CENTER_US,
    GIMBAL_TILT_SAFE_MAX_US,
    GIMBAL_TILT_DEAD_ZONE_Q15,
    GIMBAL_TILT_GAIN_US_PER_FULL_SCALE,
    GIMBAL_TILT_MAX_DELTA_US_PER_CYCLE,
    (GIMBAL_TILT_ERROR_INVERT != 0U)
};

bool GimbalController_Init(GimbalController *controller)
{
    if (controller == NULL)
    {
        return false;
    }

    *controller = (GimbalController){0};

    if (!AxisController_Init(&controller->pan, &s_pan_config) ||
        !AxisController_Init(&controller->tilt, &s_tilt_config))
    {
        return false;
    }

    return true;
}

void GimbalController_ResetCenter(GimbalController *controller)
{
    if (controller == NULL)
    {
        return;
    }

    AxisController_ResetCenter(&controller->pan, &s_pan_config);
    AxisController_ResetCenter(&controller->tilt, &s_tilt_config);
}

bool GimbalController_Step(GimbalController *controller,
                           int16_t error_x_q15,
                           int16_t error_y_q15,
                           GimbalControllerOutput *output)
{
    GimbalControllerOutput local_output = {0};

    if (controller == NULL)
    {
        return false;
    }

    if (!AxisController_Step(&controller->pan,
                             &s_pan_config,
                             error_x_q15,
                             &local_output.pan) ||
        !AxisController_Step(&controller->tilt,
                             &s_tilt_config,
                             error_y_q15,
                             &local_output.tilt))
    {
        return false;
    }

    local_output.command.pan_position_us =
        (int32_t)local_output.pan.target_us;

    local_output.command.tilt_position_us =
        (int32_t)local_output.tilt.target_us;

    controller->step_count++;

    if (output != NULL)
    {
        *output = local_output;
    }

    return true;
}

bool GimbalController_RunSelfTest(void)
{
    GimbalController controller;
    GimbalControllerOutput output;
    uint32_t i;

    if (!AxisController_RunSelfTest() ||
        !GimbalController_Init(&controller))
    {
        return false;
    }

    /* Zero error: remain at calibrated centers. */
    if (!GimbalController_Step(&controller, 0, 0, &output) ||
        (output.command.pan_position_us != (int32_t)GIMBAL_PAN_CENTER_US) ||
        (output.command.tilt_position_us != (int32_t)GIMBAL_TILT_CENTER_US))
    {
        return false;
    }

    /*
     * Synthetic contract:
     * +X means target right. Pan PWM must decrease because measured PWM
     * increase moves the camera left.
     * +Y means target below. Tilt PWM must increase because measured PWM
     * increase moves the camera down.
     */
    if (!GimbalController_Step(&controller, 8192, 8192, &output) ||
        (output.pan.applied_delta_us >= 0L) ||
        (output.tilt.applied_delta_us <= 0L))
    {
        return false;
    }

    GimbalController_ResetCenter(&controller);

    /* Full-scale motion must never exceed the configured per-cycle slew. */
    if (!GimbalController_Step(&controller, 32767, 32767, &output) ||
        (output.pan.applied_delta_us <
         -(int32_t)GIMBAL_PAN_MAX_DELTA_US_PER_CYCLE) ||
        (output.pan.applied_delta_us >
          (int32_t)GIMBAL_PAN_MAX_DELTA_US_PER_CYCLE) ||
        (output.tilt.applied_delta_us <
         -(int32_t)GIMBAL_TILT_MAX_DELTA_US_PER_CYCLE) ||
        (output.tilt.applied_delta_us >
          (int32_t)GIMBAL_TILT_MAX_DELTA_US_PER_CYCLE))
    {
        return false;
    }

    /* Drive Tilt down to its tight calibrated maximum; no overshoot allowed. */
    for (i = 0U; i < 100U; ++i)
    {
        if (!GimbalController_Step(&controller, 0, 32767, &output))
        {
            return false;
        }
    }

    if ((output.command.tilt_position_us !=
         (int32_t)GIMBAL_TILT_SAFE_MAX_US) ||
        !output.tilt.limit_blocked)
    {
        return false;
    }

    /* Reverse direction must leave the limit. */
    if (!GimbalController_Step(&controller, 0, -32768, &output) ||
        (output.tilt.applied_delta_us >= 0L) ||
        output.tilt.limit_blocked)
    {
        return false;
    }

    return true;
}
