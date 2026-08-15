#ifndef VISIONARM_ACTUATOR_DRIVER_H
#define VISIONARM_ACTUATOR_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

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
} ActuatorDriverSnapshot;

/* Initialize hardware in safe-disabled state. */
bool ActuatorDriver_InitSafe(void);

/* Enable both actuators at the calibrated centers. */
bool ActuatorDriver_Enable(void);

/* Apply one absolute physical command through the servo backend. */
bool ActuatorDriver_Apply(const ActuatorCommand *command,
                          ActuatorApplyResult *result);

/* Immediate software-safe output: PWM OFF and PA6/PA7 LOW. */
void ActuatorDriver_Disable(void);

void ActuatorDriver_GetSnapshot(ActuatorDriverSnapshot *snapshot);

#endif /* VISIONARM_ACTUATOR_DRIVER_H */
