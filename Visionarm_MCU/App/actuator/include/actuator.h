#ifndef VISIONARM_ACTUATOR_H
#define VISIONARM_ACTUATOR_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Physical actuator configuration frozen by V6 calibration.
 *
 * Electrical S20F command envelope:
 *   500..2500 us, 50 Hz.
 *
 * Installed software-safe envelope:
 *   Pan  1000..2000 us, center 1500 us.
 *   Tilt 1200..1600 us, center 1500 us.
 */
#define ACTUATOR_PWM_PERIOD_US             20000U
#define ACTUATOR_PWM_COUNTER_HZ            1000000U
#define ACTUATOR_SERVO_ELECTRICAL_MIN_US   500U
#define ACTUATOR_SERVO_CENTER_US           1500U
#define ACTUATOR_SERVO_ELECTRICAL_MAX_US   2500U

#define ACTUATOR_PAN_SAFE_MIN_US           1000U
#define ACTUATOR_PAN_CENTER_US             1500U
#define ACTUATOR_PAN_SAFE_MAX_US           2000U

#define ACTUATOR_TILT_SAFE_MIN_US          1200U
#define ACTUATOR_TILT_CENTER_US            1500U
#define ACTUATOR_TILT_SAFE_MAX_US          1600U

#if (ACTUATOR_PAN_SAFE_MIN_US < ACTUATOR_SERVO_ELECTRICAL_MIN_US) || \
    (ACTUATOR_PAN_SAFE_MAX_US > ACTUATOR_SERVO_ELECTRICAL_MAX_US)
#error "Pan software limits exceed servo electrical limits"
#endif

#if (ACTUATOR_TILT_SAFE_MIN_US < ACTUATOR_SERVO_ELECTRICAL_MIN_US) || \
    (ACTUATOR_TILT_SAFE_MAX_US > ACTUATOR_SERVO_ELECTRICAL_MAX_US)
#error "Tilt software limits exceed servo electrical limits"
#endif

#if (ACTUATOR_PAN_SAFE_MIN_US >= ACTUATOR_PAN_CENTER_US) || \
    (ACTUATOR_PAN_CENTER_US >= ACTUATOR_PAN_SAFE_MAX_US)
#error "Invalid Pan calibration ordering"
#endif

#if (ACTUATOR_TILT_SAFE_MIN_US >= ACTUATOR_TILT_CENTER_US) || \
    (ACTUATOR_TILT_CENTER_US >= ACTUATOR_TILT_SAFE_MAX_US)
#error "Invalid Tilt calibration ordering"
#endif

typedef struct
{
    int32_t pan_position_us;
    int32_t tilt_position_us;
} ActuatorCommand;

typedef struct
{
    int32_t requested_pan_us;
    int32_t requested_tilt_us;
    uint16_t applied_pan_us;
    uint16_t applied_tilt_us;
    bool pan_saturated;
    bool tilt_saturated;
} ActuatorApplyResult;

typedef struct
{
    bool initialized;
    bool enabled;
    uint16_t applied_pan_us;
    uint16_t applied_tilt_us;
    uint32_t command_count;
    uint32_t pan_saturation_count;
    uint32_t tilt_saturation_count;
    uint32_t enable_count;
    uint32_t disable_count;
    uint32_t fault_count;
} ActuatorSnapshot;

/* Configure TIM3/PA6/PA7 and leave both servo outputs disabled and LOW. */
bool Actuator_InitSafe(void);

/* Enable both servo outputs at their calibrated center positions. */
bool Actuator_Enable(void);

/* Apply one absolute pulse-width command through installed software limits. */
bool Actuator_Apply(const ActuatorCommand *command,
                    ActuatorApplyResult *result);

/* Disable PWM and force PA6/PA7 LOW. */
void Actuator_Disable(void);

/*
 * Register-level emergency primitive. Safe to call with interrupts disabled
 * and without a running scheduler or initialized HAL state.
 */
void Actuator_ForceSafeOutput(void);

void Actuator_GetSnapshot(ActuatorSnapshot *snapshot);

#endif /* VISIONARM_ACTUATOR_H */
