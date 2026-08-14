#ifndef VISIONARM_TILT_CALIBRATION_TEST_H
#define VISIONARM_TILT_CALIBRATION_TEST_H

#include <stdbool.h>
#include <stdint.h>

/*
 * VisionArm BallTrack
 * V6 Step F - Tilt single-axis calibration.
 *
 * The low-level TimerPwm driver remains the dual-channel implementation that
 * passed Step C and the Pan calibration baseline.
 *
 * During Step F:
 *   - Tilt servo SIGNAL is connected to PA7 / TIM3_CH2.
 *   - Pan servo SIGNAL must be physically disconnected.
 *   - PA6 still outputs a fixed 1500 us / 50 Hz reference waveform while the
 *     calibration PWM is active; this is useful as a logic-analyzer reference.
 *
 * Initial direction test:
 *   1500 us -> 1550 us -> 1500 us
 *
 * After that test passes, change TILT_CAL_TARGET_US to 1450U for the opposite
 * direction test.  For center calibration, set REFERENCE and TARGET equal.
 */
#define TILT_CAL_REFERENCE_US            1500U
#define TILT_CAL_TARGET_US               1200U

/*
 * Vendor reference source for the 180 degree upper servo limits its arm
 * command to approximately 9..171 degrees, corresponding to 600..2400 us
 * under the 500..2500 us / 0..180 degree mapping.  Treat these only as a
 * provisional calibration envelope; the installed software-safe limits must
 * be determined from this actual gimbal.
 */
#define TILT_CAL_PROVISIONAL_MIN_US      600U
#define TILT_CAL_PROVISIONAL_MAX_US      2400U

#define TILT_CAL_BOOT_SAFE_MS            10000U
#define TILT_CAL_REFERENCE_HOLD_MS       5000U
#define TILT_CAL_TARGET_HOLD_MS          5000U
#define TILT_CAL_FINAL_HOLD_MS           3000U

/* 2 us every 20 ms = 100 us/s command slew rate. */
#define TILT_CAL_SLEW_US_PER_CYCLE       2U
#define TILT_CAL_TASK_PERIOD_MS          20U

typedef enum
{
    TILT_CAL_STATE_BOOT_SAFE = 0,
    TILT_CAL_STATE_REFERENCE,
    TILT_CAL_STATE_MOVE_TO_TARGET,
    TILT_CAL_STATE_TARGET_HOLD,
    TILT_CAL_STATE_RETURN_TO_REFERENCE,
    TILT_CAL_STATE_FINAL_REFERENCE,
    TILT_CAL_STATE_DONE_SAFE,
    TILT_CAL_STATE_FAULT
} TiltCalibrationState;

typedef struct
{
    TiltCalibrationState state;
    uint16_t reference_us;
    uint16_t target_us;
    uint16_t applied_pan_us;
    uint16_t applied_tilt_us;
    uint32_t task_cycles;
    uint32_t pwm_update_count;
    uint32_t failure_count;
    bool outputs_enabled;
} TiltCalibrationSnapshot;

bool TiltCalibrationTest_Create(void);
void TiltCalibrationTest_GetSnapshot(TiltCalibrationSnapshot *snapshot);

#endif /* VISIONARM_TILT_CALIBRATION_TEST_H */
