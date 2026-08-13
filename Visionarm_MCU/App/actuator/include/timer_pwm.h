#ifndef VISIONARM_TIMER_PWM_H
#define VISIONARM_TIMER_PWM_H

#include <stdbool.h>
#include <stdint.h>

#define TIMER_PWM_PERIOD_US              20000U
#define TIMER_PWM_COUNTER_HZ             1000000U

#define TIMER_PWM_SERVO_MIN_US           500U
#define TIMER_PWM_SERVO_CENTER_US        1500U
#define TIMER_PWM_SERVO_MAX_US           2500U

typedef struct
{
    uint32_t pclk1_hz;
    uint32_t tim3_clock_hz;
    uint16_t prescaler;
    uint16_t auto_reload;
    uint16_t pan_pulse_us;
    uint16_t tilt_pulse_us;
    bool initialized;
    bool enabled;
} TimerPwmSnapshot;

/*
 * Initialize TIM3 for 50 Hz servo PWM while keeping PA6/PA7 driven low.
 * This function never enables PWM output.
 */
bool TimerPwm_InitSafe(void);

/*
 * Enable TIM3_CH1/CH2 with explicit initial pulse widths.
 * Invalid pulse widths cause a safe disable and return false.
 */
bool TimerPwm_Enable(uint16_t pan_pulse_us, uint16_t tilt_pulse_us);

/*
 * Update one or both PWM compare values. CCR preload makes the new value take
 * effect on a timer update event rather than tearing the current PWM period.
 */
bool TimerPwm_SetBothPulseUs(uint16_t pan_pulse_us, uint16_t tilt_pulse_us);
bool TimerPwm_SetPanPulseUs(uint16_t pulse_us);
bool TimerPwm_SetTiltPulseUs(uint16_t pulse_us);

/* Stop both PWM channels and force PA6/PA7 to push-pull low. */
void TimerPwm_Disable(void);

/*
 * Emergency-safe primitive. It does not depend on the scheduler or HAL state.
 * It may be called after interrupts are disabled or from a fatal path.
 */
void TimerPwm_ForceSafeOutput(void);

void TimerPwm_GetSnapshot(TimerPwmSnapshot *snapshot);

#endif /* VISIONARM_TIMER_PWM_H */
