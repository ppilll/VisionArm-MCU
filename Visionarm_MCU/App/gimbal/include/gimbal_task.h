#ifndef VISIONARM_GIMBAL_TASK_H
#define VISIONARM_GIMBAL_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "gimbal_control.h"

typedef struct
{
    bool active;
    bool actuator_enabled;
    bool waiting_for_fresh_control;
    GimbalSafetyReason safety_reason;

    int16_t input_error_x_q15;
    int16_t input_error_y_q15;

    uint16_t pan_target_us;
    uint16_t tilt_target_us;
    uint16_t pan_applied_us;
    uint16_t tilt_applied_us;

    /* Existing STATUS V1 fields use these normalized actuator positions. */
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

bool GimbalTask_Create(void);
void GimbalTask_GetSnapshot(GimbalRuntimeSnapshot *snapshot);

#endif /* VISIONARM_GIMBAL_TASK_H */
