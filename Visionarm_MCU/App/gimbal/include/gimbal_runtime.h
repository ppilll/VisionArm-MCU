#ifndef VISIONARM_GIMBAL_RUNTIME_H
#define VISIONARM_GIMBAL_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

#include "gimbal_safety.h"

typedef struct
{
    bool active;
    bool actuator_enabled;
    bool waiting_for_fresh_control;

    bool safety_self_test_passed;
    bool controller_self_test_passed;

    GimbalSafetyReason safety_reason;

    int16_t input_error_x_q15;
    int16_t input_error_y_q15;

    uint16_t pan_target_us;
    uint16_t tilt_target_us;
    uint16_t pan_applied_us;
    uint16_t tilt_applied_us;

    /* Existing STATUS wire fields are re-used with these normalized values. */
    int16_t pan_applied_q15;
    int16_t tilt_applied_q15;

    bool pan_dead_zone_active;
    bool tilt_dead_zone_active;
    bool pan_limit_blocked;
    bool tilt_limit_blocked;

    uint32_t mailbox_generation;
    uint32_t accepted_generation;
    uint32_t last_applied_generation;

    uint32_t task_cycle_count;
    uint32_t control_cycle_count;
    uint32_t safety_reject_count;
    uint32_t controller_fault_count;
    uint32_t actuator_apply_failure_count;

    uint32_t pan_limit_hit_count;
    uint32_t tilt_limit_hit_count;
    uint32_t pan_slew_limit_count;
    uint32_t tilt_slew_limit_count;
} GimbalRuntimeSnapshot;

void GimbalRuntime_Init(bool safety_self_test_passed,
                        bool controller_self_test_passed);

void GimbalRuntime_GetSnapshot(GimbalRuntimeSnapshot *snapshot);

void GimbalRuntime_RecordSafe(
    GimbalSafetyReason reason,
    bool waiting_for_fresh_control,
    uint32_t mailbox_generation,
    uint32_t accepted_generation,
    uint16_t pan_applied_us,
    uint16_t tilt_applied_us);

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
    bool tilt_slew_limited);

void GimbalRuntime_RecordControllerFault(void);
void GimbalRuntime_RecordActuatorApplyFailure(void);

int16_t GimbalRuntime_PanPulseToQ15(uint16_t pulse_us);
int16_t GimbalRuntime_TiltPulseToQ15(uint16_t pulse_us);

#endif /* VISIONARM_GIMBAL_RUNTIME_H */
