#ifndef VISIONARM_GIMBAL_CONTROL_H
#define VISIONARM_GIMBAL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "actuator.h"

/*
 * V6 controller contract for synthetic/control inputs.
 *
 * +X: target is to the right of the image center.
 * +Y: target is below the image center.
 *
 * Measured actuator polarity:
 *   Pan  PWM increase -> camera LEFT, therefore X error is inverted.
 *   Tilt PWM increase -> camera DOWN, therefore Y error is not inverted.
 */
#define GIMBAL_PAN_ERROR_INVERT                 1U
#define GIMBAL_TILT_ERROR_INVERT                0U

#define GIMBAL_PAN_DEAD_ZONE_Q15                2048
#define GIMBAL_TILT_DEAD_ZONE_Q15               2048
#define GIMBAL_PAN_GAIN_US_PER_FULL_SCALE       4U
#define GIMBAL_TILT_GAIN_US_PER_FULL_SCALE      4U
#define GIMBAL_PAN_MAX_DELTA_US_PER_CYCLE       2U
#define GIMBAL_TILT_MAX_DELTA_US_PER_CYCLE      2U

#if ((GIMBAL_PAN_ERROR_INVERT != 0U) && (GIMBAL_PAN_ERROR_INVERT != 1U))
#error "GIMBAL_PAN_ERROR_INVERT must be 0 or 1"
#endif

#if ((GIMBAL_TILT_ERROR_INVERT != 0U) && (GIMBAL_TILT_ERROR_INVERT != 1U))
#error "GIMBAL_TILT_ERROR_INVERT must be 0 or 1"
#endif

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

typedef struct
{
    int32_t target_us;
    int16_t last_input_q15;
    int32_t signed_error_q15;
    int32_t requested_delta_us;
    int32_t applied_delta_us;
    bool dead_zone_active;
    bool limit_blocked;
    uint32_t update_count;
    uint32_t dead_zone_count;
    uint32_t slew_limit_count;
    uint32_t limit_hit_count;
} GimbalAxisState;

typedef struct
{
    GimbalAxisState pan;
    GimbalAxisState tilt;
    uint32_t step_count;
} GimbalControlState;

typedef struct
{
    uint16_t target_us;
    int32_t signed_error_q15;
    int32_t requested_delta_us;
    int32_t applied_delta_us;
    bool dead_zone_active;
    bool slew_limited;
    bool limit_blocked;
} GimbalAxisResult;

typedef struct
{
    ActuatorCommand command;
    GimbalAxisResult pan;
    GimbalAxisResult tilt;
} GimbalControlOutput;

void GimbalSafety_Init(GimbalSafetyContext *context);
GimbalSafetyDecision GimbalSafety_Evaluate(GimbalSafetyContext *context,
                                           const GimbalSafetyInput *input);

bool GimbalControl_Init(GimbalControlState *control);
void GimbalControl_ResetCenter(GimbalControlState *control);
bool GimbalControl_Step(GimbalControlState *control,
                        int16_t error_x_q15,
                        int16_t error_y_q15,
                        GimbalControlOutput *output);

#endif /* VISIONARM_GIMBAL_CONTROL_H */
