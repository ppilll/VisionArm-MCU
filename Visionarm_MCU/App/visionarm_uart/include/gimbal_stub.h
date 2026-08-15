#ifndef VISIONARM_GIMBAL_STUB_H
#define VISIONARM_GIMBAL_STUB_H

#include <stdbool.h>
#include <stdint.h>

#include "gimbal_safety.h"

typedef struct
{
    /* Existing V5 STATUS-compatible diagnostic values. */
    int16_t pan_q15;
    int16_t tilt_q15;
    bool active;

    /* V6 Step-H runtime diagnostics; not added to protocol V1 wire format. */
    GimbalSafetyReason safety_reason;
    bool actuator_enabled;
    bool waiting_for_fresh_control;
    bool safety_self_test_passed;
    uint32_t mailbox_generation;
    uint32_t accepted_generation;
    uint32_t safety_transition_count;
    uint32_t safety_reject_count;
    uint32_t remote_stop_count;
    uint32_t link_not_ready_count;
    uint32_t control_invalid_count;
} GimbalStubSnapshot;

bool GimbalStubTask_Create(void);
void GimbalStub_GetSnapshot(GimbalStubSnapshot *snapshot);

#endif /* VISIONARM_GIMBAL_STUB_H */
