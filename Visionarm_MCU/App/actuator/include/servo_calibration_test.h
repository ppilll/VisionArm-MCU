#ifndef VISIONARM_SERVO_CALIBRATION_TEST_H
#define VISIONARM_SERVO_CALIBRATION_TEST_H

#include <stdbool.h>
#include <stdint.h>

/*
 * V6 Step D/E R1 calibration configuration.
 *
 * IMPORTANT:
 * The low-level timer driver in this revision is intentionally the exact
 * dual-channel implementation already validated in Step C.  During Pan-only
 * mechanical testing the Tilt servo SIGNAL wire must therefore be physically
 * disconnected.  PA7 still outputs a clean 1500 us / 50 Hz reference waveform
 * for logic-analyzer comparison, but it must not drive the Tilt servo yet.
 */
#define PAN_CAL_REFERENCE_US             1500U
#define PAN_CAL_TARGET_US                1550U

#define PAN_CAL_BOOT_SAFE_MS             10000U
#define PAN_CAL_REFERENCE_HOLD_MS        5000U
#define PAN_CAL_TARGET_HOLD_MS           5000U
#define PAN_CAL_FINAL_HOLD_MS            3000U

/* 2 us every 20 ms = 100 us/s command slew rate. */
#define PAN_CAL_SLEW_US_PER_CYCLE        2U
#define PAN_CAL_TASK_PERIOD_MS           20U

typedef enum
{
    SERVO_CAL_STATE_BOOT_SAFE = 0,
    SERVO_CAL_STATE_REFERENCE,
    SERVO_CAL_STATE_MOVE_TO_TARGET,
    SERVO_CAL_STATE_TARGET_HOLD,
    SERVO_CAL_STATE_RETURN_TO_REFERENCE,
    SERVO_CAL_STATE_FINAL_REFERENCE,
    SERVO_CAL_STATE_DONE_SAFE,
    SERVO_CAL_STATE_FAULT
} ServoCalibrationState;

typedef struct
{
    ServoCalibrationState state;
    uint16_t reference_us;
    uint16_t target_us;
    uint16_t applied_pan_us;
    uint16_t applied_tilt_us;
    uint32_t task_cycles;
    uint32_t pwm_update_count;
    uint32_t failure_count;
    bool outputs_enabled;
} ServoCalibrationSnapshot;

bool ServoCalibrationTest_Create(void);
void ServoCalibrationTest_GetSnapshot(ServoCalibrationSnapshot *snapshot);

#endif /* VISIONARM_SERVO_CALIBRATION_TEST_H */
