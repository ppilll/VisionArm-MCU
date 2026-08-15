#include "servo_driver.h"

#include <stddef.h>

#include "gimbal_config.h"
#include "timer_pwm.h"

static ServoDriverSnapshot s_snapshot;

static uint16_t ClampPulse(int32_t requested,
                           uint16_t minimum,
                           uint16_t maximum,
                           bool *clamped);
static bool CalibrationFitsElectricalLimits(void);

bool ServoDriver_InitSafe(void)
{
    s_snapshot = (ServoDriverSnapshot){0};

    if (!CalibrationFitsElectricalLimits())
    {
        TimerPwm_ForceSafeOutput();
        s_snapshot.fault_count++;
        return false;
    }

    if (!TimerPwm_InitSafe())
    {
        TimerPwm_ForceSafeOutput();
        s_snapshot.fault_count++;
        return false;
    }

    s_snapshot.initialized = true;
    s_snapshot.enabled = false;
    s_snapshot.applied_pan_us = GIMBAL_PAN_CENTER_US;
    s_snapshot.applied_tilt_us = GIMBAL_TILT_CENTER_US;

    return true;
}

bool ServoDriver_EnableAtCenter(void)
{
    if (!s_snapshot.initialized)
    {
        return false;
    }

    if (!TimerPwm_Enable(GIMBAL_PAN_CENTER_US,
                         GIMBAL_TILT_CENTER_US))
    {
        ServoDriver_Disable();
        s_snapshot.fault_count++;
        return false;
    }

    s_snapshot.enabled = true;
    s_snapshot.applied_pan_us = GIMBAL_PAN_CENTER_US;
    s_snapshot.applied_tilt_us = GIMBAL_TILT_CENTER_US;
    s_snapshot.enable_count++;

    return true;
}

bool ServoDriver_Apply(int32_t pan_us,
                       int32_t tilt_us,
                       ServoDriverApplyResult *result)
{
    ServoDriverApplyResult local_result;

    local_result = (ServoDriverApplyResult){0};
    local_result.requested_pan_us = pan_us;
    local_result.requested_tilt_us = tilt_us;

    if (!s_snapshot.initialized || !s_snapshot.enabled)
    {
        if (result != NULL)
        {
            *result = local_result;
        }
        return false;
    }

    local_result.applied_pan_us =
        ClampPulse(pan_us,
                   GIMBAL_PAN_SAFE_MIN_US,
                   GIMBAL_PAN_SAFE_MAX_US,
                   &local_result.pan_clamped);

    local_result.applied_tilt_us =
        ClampPulse(tilt_us,
                   GIMBAL_TILT_SAFE_MIN_US,
                   GIMBAL_TILT_SAFE_MAX_US,
                   &local_result.tilt_clamped);

    if (!TimerPwm_SetBothPulseUs(local_result.applied_pan_us,
                                 local_result.applied_tilt_us))
    {
        ServoDriver_Disable();
        s_snapshot.fault_count++;

        if (result != NULL)
        {
            *result = local_result;
        }
        return false;
    }

    s_snapshot.applied_pan_us = local_result.applied_pan_us;
    s_snapshot.applied_tilt_us = local_result.applied_tilt_us;
    s_snapshot.apply_count++;

    if (local_result.pan_clamped)
    {
        s_snapshot.pan_clamp_count++;
    }

    if (local_result.tilt_clamped)
    {
        s_snapshot.tilt_clamp_count++;
    }

    if (result != NULL)
    {
        *result = local_result;
    }

    return true;
}

void ServoDriver_Disable(void)
{
    bool was_enabled;

    was_enabled = s_snapshot.enabled;

    TimerPwm_Disable();
    s_snapshot.enabled = false;

    if (was_enabled)
    {
        s_snapshot.disable_count++;
    }
}

void ServoDriver_GetSnapshot(ServoDriverSnapshot *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    *snapshot = s_snapshot;
}

static uint16_t ClampPulse(int32_t requested,
                           uint16_t minimum,
                           uint16_t maximum,
                           bool *clamped)
{
    if (clamped != NULL)
    {
        *clamped = false;
    }

    if (requested < (int32_t)minimum)
    {
        if (clamped != NULL)
        {
            *clamped = true;
        }
        return minimum;
    }

    if (requested > (int32_t)maximum)
    {
        if (clamped != NULL)
        {
            *clamped = true;
        }
        return maximum;
    }

    return (uint16_t)requested;
}

static bool CalibrationFitsElectricalLimits(void)
{
    if ((GIMBAL_PAN_SAFE_MIN_US < TIMER_PWM_SERVO_MIN_US) ||
        (GIMBAL_PAN_SAFE_MAX_US > TIMER_PWM_SERVO_MAX_US) ||
        (GIMBAL_TILT_SAFE_MIN_US < TIMER_PWM_SERVO_MIN_US) ||
        (GIMBAL_TILT_SAFE_MAX_US > TIMER_PWM_SERVO_MAX_US))
    {
        return false;
    }

    return true;
}
