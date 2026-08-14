#ifndef VISIONARM_SERVO_CALIBRATION_TEST_H
#define VISIONARM_SERVO_CALIBRATION_TEST_H

#include <stdbool.h>
#include <stdint.h>

/*
 * VisionArm BallTrack
 * V6 Step E1 - Pan Center Calibration
 *
 * Purpose:
 *   Find the installed camera's real Pan center command.
 *
 * The Pan command is intentionally fixed during one test run:
 *
 *   reference == target
 *
 * Example:
 *   1500 us -> hold -> safe disable
 *
 * To explore camera center:
 *
 *   1500
 *   1520 / 1480
 *   1540 / 1460
 *   ...
 *
 * Once close to the desired center, reduce adjustment to 10 us.
 *
 * IMPORTANT:
 *   - Only Pan servo is mechanically under test.
 *   - Tilt servo SIGNAL must remain physically disconnected.
 *   - PA7 still carries the fixed 1500 us logic-analyzer reference
 *     while PWM is active.
 */

/*
 * Current center candidate.
 *
 * For Step E1, these two values MUST remain equal.
 */
#define PAN_CAL_REFERENCE_US             1500U
#define PAN_CAL_TARGET_US                1000U

/*
 * Keep a clear boot-safe observation window.
 */
#define PAN_CAL_BOOT_SAFE_MS             10000U

/*
 * Hold the candidate center long enough for visual inspection.
 */
#define PAN_CAL_REFERENCE_HOLD_MS        8000U

/*
 * reference == target in center calibration, so this phase produces
 * no additional mechanical movement. It is retained because the same
 * calibration task implementation is reused later for range testing.
 */
#define PAN_CAL_TARGET_HOLD_MS           3000U

/*
 * Final reference hold before PWM is disabled.
 */
#define PAN_CAL_FINAL_HOLD_MS            3000U

/*
 * Used later by range calibration.
 *
 * 2 us every 20 ms:
 *   50 cycles/s * 2 us/cycle
 *   = 100 us/s command slew rate.
 */
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

void ServoCalibrationTest_GetSnapshot(
    ServoCalibrationSnapshot *snapshot);

#endif /* VISIONARM_SERVO_CALIBRATION_TEST_H */