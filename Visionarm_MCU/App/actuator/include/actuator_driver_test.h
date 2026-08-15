#ifndef VISIONARM_ACTUATOR_DRIVER_TEST_H
#define VISIONARM_ACTUATOR_DRIVER_TEST_H

#include <stdbool.h>
#include <stdint.h>

/*
 * V6 Step-G physical validation.
 *
 * All targets are only +/-50 us around the measured centers and are well
 * inside the frozen installed limits:
 *   Pan  1000..2000 us, center 1500 us
 *   Tilt 1200..1600 us, center 1500 us
 *
 * The task is one-shot and ends with both PWM outputs disabled/LOW.
 */
#define ACTUATOR_TEST_TASK_PERIOD_MS            20U
#define ACTUATOR_TEST_BOOT_SAFE_MS              10000U
#define ACTUATOR_TEST_INITIAL_CENTER_HOLD_MS    3000U
#define ACTUATOR_TEST_TARGET_HOLD_MS            2000U
#define ACTUATOR_TEST_CENTER_HOLD_MS            2000U
#define ACTUATOR_TEST_SLEW_US_PER_CYCLE         2U

#define ACTUATOR_TEST_PAN_LEFT_US               1550U
#define ACTUATOR_TEST_PAN_RIGHT_US              1450U
#define ACTUATOR_TEST_TILT_DOWN_US              1550U
#define ACTUATOR_TEST_TILT_UP_US                1450U

typedef enum
{
    ACTUATOR_TEST_STATE_BOOT_SAFE = 0,
    ACTUATOR_TEST_STATE_CENTER,
    ACTUATOR_TEST_STATE_PAN_LEFT,
    ACTUATOR_TEST_STATE_PAN_RIGHT,
    ACTUATOR_TEST_STATE_TILT_DOWN,
    ACTUATOR_TEST_STATE_TILT_UP,
    ACTUATOR_TEST_STATE_DUAL_LEFT_DOWN,
    ACTUATOR_TEST_STATE_DUAL_RIGHT_UP,
    ACTUATOR_TEST_STATE_RETURN_CENTER,
    ACTUATOR_TEST_STATE_DONE_SAFE,
    ACTUATOR_TEST_STATE_FAULT
} ActuatorDriverTestState;

typedef struct
{
    ActuatorDriverTestState state;
    uint32_t task_cycles;
    uint32_t command_count;
    uint32_t failure_count;

    uint16_t applied_pan_us;
    uint16_t applied_tilt_us;

    bool last_pan_saturated;
    bool last_tilt_saturated;
} ActuatorDriverTestSnapshot;

bool ActuatorDriverTest_Create(void);
void ActuatorDriverTest_GetSnapshot(ActuatorDriverTestSnapshot *snapshot);

#endif /* VISIONARM_ACTUATOR_DRIVER_TEST_H */
