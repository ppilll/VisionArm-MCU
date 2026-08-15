#include "actuator_driver.h"

#include <stddef.h>

#include "gimbal_config.h"
#include "servo_driver.h"

static ActuatorDriverSnapshot s_snapshot;

bool ActuatorDriver_InitSafe(void)
{
    s_snapshot = (ActuatorDriverSnapshot){0};

    if (!ServoDriver_InitSafe())
    {
        s_snapshot.fault_count++;
        return false;
    }

    s_snapshot.initialized = true;
    s_snapshot.enabled = false;
    s_snapshot.applied_pan_us = GIMBAL_PAN_CENTER_US;
    s_snapshot.applied_tilt_us = GIMBAL_TILT_CENTER_US;

    return true;
}

bool ActuatorDriver_Enable(void)
{
    if (!s_snapshot.initialized)
    {
        return false;
    }

    if (!ServoDriver_EnableAtCenter())
    {
        ActuatorDriver_Disable();
        s_snapshot.fault_count++;
        return false;
    }

    s_snapshot.enabled = true;
    s_snapshot.applied_pan_us = GIMBAL_PAN_CENTER_US;
    s_snapshot.applied_tilt_us = GIMBAL_TILT_CENTER_US;
    s_snapshot.enable_count++;

    return true;
}

bool ActuatorDriver_Apply(const ActuatorCommand *command,
                          ActuatorApplyResult *result)
{
    ServoDriverApplyResult servo_result;
    ActuatorApplyResult local_result;

    servo_result = (ServoDriverApplyResult){0};
    local_result = (ActuatorApplyResult){0};

    if (command == NULL || !s_snapshot.initialized || !s_snapshot.enabled)
    {
        if (result != NULL)
        {
            *result = local_result;
        }
        return false;
    }

    if (!ServoDriver_Apply(command->pan_position_us,
                           command->tilt_position_us,
                           &servo_result))
    {
        ActuatorDriver_Disable();
        s_snapshot.fault_count++;

        if (result != NULL)
        {
            *result = local_result;
        }
        return false;
    }

    local_result.requested_pan_us = servo_result.requested_pan_us;
    local_result.requested_tilt_us = servo_result.requested_tilt_us;
    local_result.applied_pan_us = servo_result.applied_pan_us;
    local_result.applied_tilt_us = servo_result.applied_tilt_us;
    local_result.pan_saturated = servo_result.pan_clamped;
    local_result.tilt_saturated = servo_result.tilt_clamped;

    s_snapshot.applied_pan_us = local_result.applied_pan_us;
    s_snapshot.applied_tilt_us = local_result.applied_tilt_us;
    s_snapshot.command_count++;

    if (local_result.pan_saturated)
    {
        s_snapshot.pan_saturation_count++;
    }

    if (local_result.tilt_saturated)
    {
        s_snapshot.tilt_saturation_count++;
    }

    if (result != NULL)
    {
        *result = local_result;
    }

    return true;
}

void ActuatorDriver_Disable(void)
{
    bool was_enabled;

    was_enabled = s_snapshot.enabled;

    ServoDriver_Disable();
    s_snapshot.enabled = false;

    if (was_enabled)
    {
        s_snapshot.disable_count++;
    }
}

void ActuatorDriver_GetSnapshot(ActuatorDriverSnapshot *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    *snapshot = s_snapshot;
}
