#ifndef VISIONARM_GIMBAL_CONFIG_H
#define VISIONARM_GIMBAL_CONFIG_H

#include <stdint.h>

/*
 * VisionArm BallTrack - V6 frozen actuator calibration.
 *
 * Installed software-safe limits measured on the assembled gimbal.
 * These are intentionally tighter than the S20F electrical 500..2500 us
 * range that remains independently enforced by TimerPwm.
 */
#define GIMBAL_PAN_SAFE_MIN_US                  1000U
#define GIMBAL_PAN_CENTER_US                    1500U
#define GIMBAL_PAN_SAFE_MAX_US                  2000U

#define GIMBAL_TILT_SAFE_MIN_US                 1200U
#define GIMBAL_TILT_CENTER_US                   1500U
#define GIMBAL_TILT_SAFE_MAX_US                 1600U

/* Measured physical actuator direction facts. */
#define GIMBAL_PAN_PWM_INCREASES_CAMERA_LEFT    1U
#define GIMBAL_TILT_PWM_INCREASES_CAMERA_DOWN   1U

/*
 * V6 Step-I/K synthetic error contract.
 *
 * This defines what the MCU controller expects from the synthetic source:
 *   error_x_q15 > 0 : target is to the RIGHT of image center.
 *   error_x_q15 < 0 : target is to the LEFT of image center.
 *   error_y_q15 > 0 : target is BELOW image center.
 *   error_y_q15 < 0 : target is ABOVE image center.
 *
 * This is the deterministic Step-K contract. The real V4 ControlResult sign
 * is still verified later by the limited live-direction test before V7.
 *
 * Because Pan PWM increase moves camera LEFT, positive X error must DECREASE
 * Pan PWM. Because Tilt PWM increase moves camera DOWN, positive Y error must
 * INCREASE Tilt PWM.
 */
#define GIMBAL_PAN_ERROR_INVERT                 1U
#define GIMBAL_TILT_ERROR_INVERT                0U

/*
 * Conservative V6 controller parameters.
 *
 * Dead-zone:
 *   2048 / 32767 ~= 6.25 % of normalized full-scale image error.
 *
 * Gain:
 *   4 us/cycle at full-scale before slew limiting.
 *
 * Slew:
 *   maximum physical target change = 2 us per 20 ms cycle
 *   = 100 us/s.
 *
 * This is the same command slew already validated during calibration.
 * V7 may tune gain/dead-zone based on closed-loop response measurements.
 */
#define GIMBAL_PAN_DEAD_ZONE_Q15                2048
#define GIMBAL_TILT_DEAD_ZONE_Q15               2048

#define GIMBAL_PAN_GAIN_US_PER_FULL_SCALE       4U
#define GIMBAL_TILT_GAIN_US_PER_FULL_SCALE      4U

#define GIMBAL_PAN_MAX_DELTA_US_PER_CYCLE       2U
#define GIMBAL_TILT_MAX_DELTA_US_PER_CYCLE      2U

#define GIMBAL_CONTROL_PERIOD_MS                20U

/* Step-G autonomous validation is permanently disabled in production path. */
#define GIMBAL_STEP_G_SELF_TEST_ENABLED         0U

#if (GIMBAL_PAN_SAFE_MIN_US >= GIMBAL_PAN_CENTER_US)
#error "Pan safe minimum must be below center"
#endif

#if (GIMBAL_PAN_CENTER_US >= GIMBAL_PAN_SAFE_MAX_US)
#error "Pan center must be below safe maximum"
#endif

#if (GIMBAL_TILT_SAFE_MIN_US >= GIMBAL_TILT_CENTER_US)
#error "Tilt safe minimum must be below center"
#endif

#if (GIMBAL_TILT_CENTER_US >= GIMBAL_TILT_SAFE_MAX_US)
#error "Tilt center must be below safe maximum"
#endif

#if ((GIMBAL_PAN_ERROR_INVERT != 0U) && (GIMBAL_PAN_ERROR_INVERT != 1U))
#error "GIMBAL_PAN_ERROR_INVERT must be 0 or 1"
#endif

#if ((GIMBAL_TILT_ERROR_INVERT != 0U) && (GIMBAL_TILT_ERROR_INVERT != 1U))
#error "GIMBAL_TILT_ERROR_INVERT must be 0 or 1"
#endif

#if ((GIMBAL_STEP_G_SELF_TEST_ENABLED != 0U) && \
     (GIMBAL_STEP_G_SELF_TEST_ENABLED != 1U))
#error "GIMBAL_STEP_G_SELF_TEST_ENABLED must be 0 or 1"
#endif

#endif /* VISIONARM_GIMBAL_CONFIG_H */
