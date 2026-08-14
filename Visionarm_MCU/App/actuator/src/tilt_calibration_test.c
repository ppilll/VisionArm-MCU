#include "tilt_calibration_test.h"

#include "FreeRTOS.h"
#include "task.h"

#include "timer_pwm.h"

#define TILT_CAL_TASK_PRIORITY           1U
#define TILT_CAL_TASK_STACK_WORDS        192U

static TaskHandle_t s_task;
static StaticTask_t s_task_tcb;
static StackType_t s_task_stack[TILT_CAL_TASK_STACK_WORDS];
static TiltCalibrationSnapshot s_snapshot;

static void TiltCalibrationTask(void *argument);
static bool ValidateConfiguration(void);
static bool MoveTiltToward(uint16_t target_us);
static void SetState(TiltCalibrationState state);
static void EnterFault(void);

bool TiltCalibrationTest_Create(void)
{
    if ((s_task != NULL) || !ValidateConfiguration())
    {
        return false;
    }

    s_snapshot = (TiltCalibrationSnapshot){0};
    s_snapshot.state = TILT_CAL_STATE_BOOT_SAFE;
    s_snapshot.reference_us = TILT_CAL_REFERENCE_US;
    s_snapshot.target_us = TILT_CAL_TARGET_US;
    s_snapshot.applied_pan_us = TIMER_PWM_SERVO_CENTER_US;
    s_snapshot.applied_tilt_us = TILT_CAL_REFERENCE_US;
    s_snapshot.outputs_enabled = false;

    s_task = xTaskCreateStatic(TiltCalibrationTask,
                               "TiltCal",
                               TILT_CAL_TASK_STACK_WORDS,
                               NULL,
                               TILT_CAL_TASK_PRIORITY,
                               s_task_stack,
                               &s_task_tcb);

    return (s_task != NULL);
}

void TiltCalibrationTest_GetSnapshot(TiltCalibrationSnapshot *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = s_snapshot;
    taskEXIT_CRITICAL();
}

static void TiltCalibrationTask(void *argument)
{
    TickType_t last_wake;
    TickType_t phase_start;
    const TickType_t period_ticks = pdMS_TO_TICKS(TILT_CAL_TASK_PERIOD_MS);

    (void)argument;

    /* Boot-safe phase: neither channel produces PWM. */
    TimerPwm_Disable();
    SetState(TILT_CAL_STATE_BOOT_SAFE);

    last_wake = xTaskGetTickCount();
    phase_start = last_wake;

    while ((xTaskGetTickCount() - phase_start) <
           pdMS_TO_TICKS(TILT_CAL_BOOT_SAFE_MS))
    {
        (void)xTaskDelayUntil(&last_wake, period_ticks);
        s_snapshot.task_cycles++;
    }

    /*
     * Reuse the exact dual-channel path already validated in Step C.
     * PA6 stays at 1500 us as the analyzer reference; PA7 is the Tilt command.
     * The physical Pan SIGNAL must be disconnected throughout Step F.
     */
    if (!TimerPwm_Enable(TIMER_PWM_SERVO_CENTER_US,
                         TILT_CAL_REFERENCE_US))
    {
        EnterFault();
    }

    s_snapshot.outputs_enabled = true;
    s_snapshot.applied_pan_us = TIMER_PWM_SERVO_CENTER_US;
    s_snapshot.applied_tilt_us = TILT_CAL_REFERENCE_US;
    SetState(TILT_CAL_STATE_REFERENCE);

    phase_start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - phase_start) <
           pdMS_TO_TICKS(TILT_CAL_REFERENCE_HOLD_MS))
    {
        (void)xTaskDelayUntil(&last_wake, period_ticks);
        s_snapshot.task_cycles++;
    }

    SetState(TILT_CAL_STATE_MOVE_TO_TARGET);
    while (s_snapshot.applied_tilt_us != TILT_CAL_TARGET_US)
    {
        if (!MoveTiltToward(TILT_CAL_TARGET_US))
        {
            EnterFault();
        }

        (void)xTaskDelayUntil(&last_wake, period_ticks);
        s_snapshot.task_cycles++;
    }

    SetState(TILT_CAL_STATE_TARGET_HOLD);
    phase_start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - phase_start) <
           pdMS_TO_TICKS(TILT_CAL_TARGET_HOLD_MS))
    {
        (void)xTaskDelayUntil(&last_wake, period_ticks);
        s_snapshot.task_cycles++;
    }

    SetState(TILT_CAL_STATE_RETURN_TO_REFERENCE);
    while (s_snapshot.applied_tilt_us != TILT_CAL_REFERENCE_US)
    {
        if (!MoveTiltToward(TILT_CAL_REFERENCE_US))
        {
            EnterFault();
        }

        (void)xTaskDelayUntil(&last_wake, period_ticks);
        s_snapshot.task_cycles++;
    }

    SetState(TILT_CAL_STATE_FINAL_REFERENCE);
    phase_start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - phase_start) <
           pdMS_TO_TICKS(TILT_CAL_FINAL_HOLD_MS))
    {
        (void)xTaskDelayUntil(&last_wake, period_ticks);
        s_snapshot.task_cycles++;
    }

    TimerPwm_Disable();
    s_snapshot.outputs_enabled = false;
    SetState(TILT_CAL_STATE_DONE_SAFE);

    /* One-shot test. Never automatically repeat a mechanical calibration. */
    for (;;)
    {
        (void)xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000U));
        s_snapshot.task_cycles++;
    }
}

static bool ValidateConfiguration(void)
{
    if ((TILT_CAL_PROVISIONAL_MIN_US < TIMER_PWM_SERVO_MIN_US) ||
        (TILT_CAL_PROVISIONAL_MAX_US > TIMER_PWM_SERVO_MAX_US) ||
        (TILT_CAL_PROVISIONAL_MIN_US >= TILT_CAL_PROVISIONAL_MAX_US) ||
        (TILT_CAL_REFERENCE_US < TILT_CAL_PROVISIONAL_MIN_US) ||
        (TILT_CAL_REFERENCE_US > TILT_CAL_PROVISIONAL_MAX_US) ||
        (TILT_CAL_TARGET_US < TILT_CAL_PROVISIONAL_MIN_US) ||
        (TILT_CAL_TARGET_US > TILT_CAL_PROVISIONAL_MAX_US) ||
        (TILT_CAL_SLEW_US_PER_CYCLE == 0U) ||
        (TILT_CAL_TASK_PERIOD_MS == 0U))
    {
        return false;
    }

    return true;
}

static bool MoveTiltToward(uint16_t target_us)
{
    uint16_t current_us;
    uint16_t next_us;
    uint16_t delta_us;

    current_us = s_snapshot.applied_tilt_us;
    next_us = current_us;

    if (current_us < target_us)
    {
        delta_us = (uint16_t)(target_us - current_us);
        if (delta_us > TILT_CAL_SLEW_US_PER_CYCLE)
        {
            delta_us = TILT_CAL_SLEW_US_PER_CYCLE;
        }
        next_us = (uint16_t)(current_us + delta_us);
    }
    else if (current_us > target_us)
    {
        delta_us = (uint16_t)(current_us - target_us);
        if (delta_us > TILT_CAL_SLEW_US_PER_CYCLE)
        {
            delta_us = TILT_CAL_SLEW_US_PER_CYCLE;
        }
        next_us = (uint16_t)(current_us - delta_us);
    }

    if (next_us == current_us)
    {
        return true;
    }

    if (!TimerPwm_SetTiltPulseUs(next_us))
    {
        return false;
    }

    s_snapshot.applied_tilt_us = next_us;
    s_snapshot.pwm_update_count++;
    return true;
}

static void SetState(TiltCalibrationState state)
{
    s_snapshot.state = state;
}

static void EnterFault(void)
{
    TickType_t last_wake;

    TimerPwm_Disable();
    s_snapshot.outputs_enabled = false;
    s_snapshot.failure_count++;
    SetState(TILT_CAL_STATE_FAULT);

    last_wake = xTaskGetTickCount();
    for (;;)
    {
        (void)xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000U));
        s_snapshot.task_cycles++;
    }
}
