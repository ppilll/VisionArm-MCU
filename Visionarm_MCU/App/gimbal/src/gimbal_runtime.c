#include "gimbal_runtime.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

#include "gimbal_config.h"

static GimbalRuntimeSnapshot s_runtime;

static int16_t PulseToQ15(uint16_t pulse_us,
                          uint16_t minimum_us,
                          uint16_t center_us,
                          uint16_t maximum_us);

void GimbalRuntime_Init(bool safety_self_test_passed,
                        bool controller_self_test_passed)
{
    /* Called before scheduler start; no concurrent reader exists yet. */
    s_runtime = (GimbalRuntimeSnapshot){0};
    s_runtime.safety_self_test_passed = safety_self_test_passed;
    s_runtime.controller_self_test_passed = controller_self_test_passed;
    s_runtime.safety_reason = GIMBAL_SAFETY_BOOT_SAFE;
    s_runtime.pan_target_us = GIMBAL_PAN_CENTER_US;
    s_runtime.tilt_target_us = GIMBAL_TILT_CENTER_US;
    s_runtime.pan_applied_us = GIMBAL_PAN_CENTER_US;
    s_runtime.tilt_applied_us = GIMBAL_TILT_CENTER_US;
    s_runtime.pan_applied_q15 = 0;
    s_runtime.tilt_applied_q15 = 0;
}

void GimbalRuntime_GetSnapshot(GimbalRuntimeSnapshot *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = s_runtime;
    taskEXIT_CRITICAL();
}

void GimbalRuntime_RecordSafe(
    GimbalSafetyReason reason,
    bool waiting_for_fresh_control,
    uint32_t mailbox_generation,
    uint32_t accepted_generation,
    uint16_t pan_applied_us,
    uint16_t tilt_applied_us)
{
    taskENTER_CRITICAL();

    s_runtime.task_cycle_count++;
    s_runtime.active = false;
    s_runtime.actuator_enabled = false;
    s_runtime.waiting_for_fresh_control = waiting_for_fresh_control;
    s_runtime.safety_reason = reason;
    s_runtime.input_error_x_q15 = 0;
    s_runtime.input_error_y_q15 = 0;
    s_runtime.pan_target_us = GIMBAL_PAN_CENTER_US;
    s_runtime.tilt_target_us = GIMBAL_TILT_CENTER_US;
    s_runtime.pan_applied_us = pan_applied_us;
    s_runtime.tilt_applied_us = tilt_applied_us;
    s_runtime.pan_applied_q15 = GimbalRuntime_PanPulseToQ15(pan_applied_us);
    s_runtime.tilt_applied_q15 = GimbalRuntime_TiltPulseToQ15(tilt_applied_us);
    s_runtime.pan_dead_zone_active = true;
    s_runtime.tilt_dead_zone_active = true;
    s_runtime.pan_limit_blocked = false;
    s_runtime.tilt_limit_blocked = false;
    s_runtime.mailbox_generation = mailbox_generation;
    s_runtime.accepted_generation = accepted_generation;
    s_runtime.safety_reject_count++;

    taskEXIT_CRITICAL();
}

void GimbalRuntime_RecordActive(
    GimbalSafetyReason reason,
    uint32_t mailbox_generation,
    uint32_t accepted_generation,
    int16_t error_x_q15,
    int16_t error_y_q15,
    uint16_t pan_target_us,
    uint16_t tilt_target_us,
    uint16_t pan_applied_us,
    uint16_t tilt_applied_us,
    bool pan_dead_zone_active,
    bool tilt_dead_zone_active,
    bool pan_limit_blocked,
    bool tilt_limit_blocked,
    bool pan_slew_limited,
    bool tilt_slew_limited)
{
    taskENTER_CRITICAL();

    s_runtime.task_cycle_count++;
    s_runtime.control_cycle_count++;
    s_runtime.active = true;
    s_runtime.actuator_enabled = true;
    s_runtime.waiting_for_fresh_control = false;
    s_runtime.safety_reason = reason;
    s_runtime.input_error_x_q15 = error_x_q15;
    s_runtime.input_error_y_q15 = error_y_q15;
    s_runtime.pan_target_us = pan_target_us;
    s_runtime.tilt_target_us = tilt_target_us;
    s_runtime.pan_applied_us = pan_applied_us;
    s_runtime.tilt_applied_us = tilt_applied_us;
    s_runtime.pan_applied_q15 = GimbalRuntime_PanPulseToQ15(pan_applied_us);
    s_runtime.tilt_applied_q15 = GimbalRuntime_TiltPulseToQ15(tilt_applied_us);
    s_runtime.pan_dead_zone_active = pan_dead_zone_active;
    s_runtime.tilt_dead_zone_active = tilt_dead_zone_active;
    s_runtime.pan_limit_blocked = pan_limit_blocked;
    s_runtime.tilt_limit_blocked = tilt_limit_blocked;
    s_runtime.mailbox_generation = mailbox_generation;
    s_runtime.accepted_generation = accepted_generation;
    s_runtime.last_applied_generation = mailbox_generation;

    if (pan_limit_blocked)
    {
        s_runtime.pan_limit_hit_count++;
    }

    if (tilt_limit_blocked)
    {
        s_runtime.tilt_limit_hit_count++;
    }

    if (pan_slew_limited)
    {
        s_runtime.pan_slew_limit_count++;
    }

    if (tilt_slew_limited)
    {
        s_runtime.tilt_slew_limit_count++;
    }

    taskEXIT_CRITICAL();
}

void GimbalRuntime_RecordControllerFault(void)
{
    taskENTER_CRITICAL();
    s_runtime.controller_fault_count++;
    taskEXIT_CRITICAL();
}

void GimbalRuntime_RecordActuatorApplyFailure(void)
{
    taskENTER_CRITICAL();
    s_runtime.actuator_apply_failure_count++;
    taskEXIT_CRITICAL();
}

int16_t GimbalRuntime_PanPulseToQ15(uint16_t pulse_us)
{
    return PulseToQ15(pulse_us,
                      GIMBAL_PAN_SAFE_MIN_US,
                      GIMBAL_PAN_CENTER_US,
                      GIMBAL_PAN_SAFE_MAX_US);
}

int16_t GimbalRuntime_TiltPulseToQ15(uint16_t pulse_us)
{
    return PulseToQ15(pulse_us,
                      GIMBAL_TILT_SAFE_MIN_US,
                      GIMBAL_TILT_CENTER_US,
                      GIMBAL_TILT_SAFE_MAX_US);
}

static int16_t PulseToQ15(uint16_t pulse_us,
                          uint16_t minimum_us,
                          uint16_t center_us,
                          uint16_t maximum_us)
{
    int32_t value;

    if (pulse_us <= minimum_us)
    {
        return (int16_t)-32767;
    }

    if (pulse_us >= maximum_us)
    {
        return (int16_t)32767;
    }

    if (pulse_us == center_us)
    {
        return 0;
    }

    if (pulse_us < center_us)
    {
        value =
            -((int32_t)(center_us - pulse_us) * 32767L) /
             (int32_t)(center_us - minimum_us);
    }
    else
    {
        value =
            ((int32_t)(pulse_us - center_us) * 32767L) /
            (int32_t)(maximum_us - center_us);
    }

    if (value < -32767L)
    {
        value = -32767L;
    }
    else if (value > 32767L)
    {
        value = 32767L;
    }

    return (int16_t)value;
}
