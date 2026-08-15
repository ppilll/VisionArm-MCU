#ifndef VISIONARM_SERVO_DRIVER_H
#define VISIONARM_SERVO_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int32_t requested_pan_us;
    int32_t requested_tilt_us;

    uint16_t applied_pan_us;
    uint16_t applied_tilt_us;

    bool pan_clamped;
    bool tilt_clamped;
} ServoDriverApplyResult;

typedef struct
{
    bool initialized;
    bool enabled;

    uint16_t applied_pan_us;
    uint16_t applied_tilt_us;

    uint32_t apply_count;
    uint32_t pan_clamp_count;
    uint32_t tilt_clamp_count;
    uint32_t enable_count;
    uint32_t disable_count;
    uint32_t fault_count;
} ServoDriverSnapshot;

/*
 * Initialize the physical servo layer in a disabled/LOW state.
 * No PWM is emitted by this function.
 */
bool ServoDriver_InitSafe(void);

/* Enable both physical servo outputs at the frozen installed center. */
bool ServoDriver_EnableAtCenter(void);

/*
 * Apply an absolute physical pulse-width command.
 *
 * The request is always clamped to the installed software-safe limits before
 * it reaches TimerPwm.  TimerPwm independently retains the lower-level
 * electrical 500..2500 us clamp/validation boundary.
 */
bool ServoDriver_Apply(int32_t pan_us,
                       int32_t tilt_us,
                       ServoDriverApplyResult *result);

/* Disable PWM and force PA6/PA7 LOW through TimerPwm. */
void ServoDriver_Disable(void);

void ServoDriver_GetSnapshot(ServoDriverSnapshot *snapshot);

#endif /* VISIONARM_SERVO_DRIVER_H */
