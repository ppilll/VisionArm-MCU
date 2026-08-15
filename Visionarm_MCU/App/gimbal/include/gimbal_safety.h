#ifndef VISIONARM_GIMBAL_SAFETY_H
#define VISIONARM_GIMBAL_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    GIMBAL_SAFETY_BOOT_SAFE = 0,
    GIMBAL_SAFETY_LOCAL_FAULT,
    GIMBAL_SAFETY_REMOTE_STOP,
    GIMBAL_SAFETY_LINK_NOT_READY,
    GIMBAL_SAFETY_CONTROL_INVALID,
    GIMBAL_SAFETY_MAILBOX_UNAVAILABLE,
    GIMBAL_SAFETY_MAILBOX_INVALID,
    GIMBAL_SAFETY_WAIT_FRESH_CONTROL,
    GIMBAL_SAFETY_ENABLED
} GimbalSafetyReason;

typedef struct
{
    bool link_ready;
    bool remote_stop_latched;
    bool control_valid;

    bool mailbox_available;
    bool mailbox_valid;
    uint32_t mailbox_generation;

    bool actuator_initialized;
    bool local_fault;
} GimbalSafetyInput;

typedef struct
{
    bool control_enabled;
    GimbalSafetyReason reason;
    uint32_t mailbox_generation;
    uint32_t accepted_generation;
    bool waiting_for_fresh_control;
} GimbalSafetyDecision;

typedef struct
{
    bool require_fresh_control;
    uint32_t blocked_generation;
    uint32_t accepted_generation;
    GimbalSafetyReason last_reason;
    uint32_t transition_count;
} GimbalSafetyContext;

void GimbalSafety_Init(GimbalSafetyContext *context);

GimbalSafetyDecision GimbalSafety_Evaluate(
    GimbalSafetyContext *context,
    const GimbalSafetyInput *input);

/*
 * Deterministic startup self-test of priority and fresh-generation semantics.
 * This test does not touch protocol state, RTOS objects, timers, or actuators.
 */
bool GimbalSafety_RunSelfTest(void);

const char *GimbalSafety_ReasonName(GimbalSafetyReason reason);

#endif /* VISIONARM_GIMBAL_SAFETY_H */
