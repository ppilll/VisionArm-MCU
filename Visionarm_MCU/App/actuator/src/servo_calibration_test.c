#include "servo_calibration_test.h"

#include "FreeRTOS.h"
#include "task.h"

#include "timer_pwm.h"

#define SERVO_CAL_TASK_PRIORITY          1U
#define SERVO_CAL_TASK_STACK_WORDS       192U

static TaskHandle_t s_task;
static StaticTask_t s_task_tcb;
static StackType_t s_task_stack[SERVO_CAL_TASK_STACK_WORDS];
static ServoCalibrationSnapshot s_snapshot;

static void ServoCalibrationTask(void *argument);
static bool ValidateConfiguration(void);
static bool MovePanToward(uint16_t target_us);
static void SetState(ServoCalibrationState state);
static void EnterFault(void);

bool ServoCalibrationTest_Create(void)
{
    if ((s_task != NULL) || !ValidateConfiguration())
    {
        return false;
    }

    s_snapshot = (ServoCalibrationSnapshot){0};
    s_snapshot.state = SERVO_CAL_STATE_BOOT_SAFE;
    s_snapshot.reference_us = PAN_CAL_REFERENCE_US;
    s_snapshot.target_us = PAN_CAL_TARGET_US;
    s_snapshot.applied_pan_us = PAN_CAL_REFERENCE_US;
    s_snapshot.applied_tilt_us = TIMER_PWM_SERVO_CENTER_US;
    s_snapshot.outputs_enabled = false;

    s_task = xTaskCreateStatic(ServoCalibrationTask,
                               "ServoCal",
                               SERVO_CAL_TASK_STACK_WORDS,
                               NULL,
                               SERVO_CAL_TASK_PRIORITY,
                               s_task_stack,
                               &s_task_tcb);

    return (s_task != NULL);
}

void ServoCalibrationTest_GetSnapshot(ServoCalibrationSnapshot *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = s_snapshot;
    taskEXIT_CRITICAL();
}

static void ServoCalibrationTask(void *argument)
{
    TickType_t last_wake;
    TickType_t phase_start;
    const TickType_t period_ticks = pdMS_TO_TICKS(PAN_CAL_TASK_PERIOD_MS);

    (void)argument;

    /* Step B safe state: both timer outputs physically disabled and LOW. */
    TimerPwm_Disable();
    SetState(SERVO_CAL_STATE_BOOT_SAFE);

    last_wake = xTaskGetTickCount();
    phase_start = last_wake;

    while ((xTaskGetTickCount() - phase_start) <
           pdMS_TO_TICKS(PAN_CAL_BOOT_SAFE_MS))
    {
        (void)xTaskDelayUntil(&last_wake, period_ticks);
        s_snapshot.task_cycles++;
    }

    /*
     * Deliberately reuse the exact dual-channel start path already proven by
     * Step C.  Pan begins at the reference pulse; PA7 remains at 1500 us only
     * as an analyzer reference.  The physical Tilt SIGNAL must be disconnected
     * during Step D/E Pan calibration.
     */
    if (!TimerPwm_Enable(PAN_CAL_REFERENCE_US,
                         TIMER_PWM_SERVO_CENTER_US))
    {
        EnterFault();
    }

    s_snapshot.outputs_enabled = true;
    s_snapshot.applied_pan_us = PAN_CAL_REFERENCE_US;
    s_snapshot.applied_tilt_us = TIMER_PWM_SERVO_CENTER_US;
    SetState(SERVO_CAL_STATE_REFERENCE);

    phase_start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - phase_start) <
           pdMS_TO_TICKS(PAN_CAL_REFERENCE_HOLD_MS))
    {
        (void)xTaskDelayUntil(&last_wake, period_ticks);
        s_snapshot.task_cycles++;
    }

    SetState(SERVO_CAL_STATE_MOVE_TO_TARGET);
    while (s_snapshot.applied_pan_us != PAN_CAL_TARGET_US)
    {
        if (!MovePanToward(PAN_CAL_TARGET_US))
        {
            EnterFault();
        }

        (void)xTaskDelayUntil(&last_wake, period_ticks);
        s_snapshot.task_cycles++;
    }

    SetState(SERVO_CAL_STATE_TARGET_HOLD);
    phase_start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - phase_start) <
           pdMS_TO_TICKS(PAN_CAL_TARGET_HOLD_MS))
    {
        (void)xTaskDelayUntil(&last_wake, period_ticks);
        s_snapshot.task_cycles++;
    }

    SetState(SERVO_CAL_STATE_RETURN_TO_REFERENCE);
    while (s_snapshot.applied_pan_us != PAN_CAL_REFERENCE_US)
    {
        if (!MovePanToward(PAN_CAL_REFERENCE_US))
        {
            EnterFault();
        }

        (void)xTaskDelayUntil(&last_wake, period_ticks);
        s_snapshot.task_cycles++;
    }

    SetState(SERVO_CAL_STATE_FINAL_REFERENCE);
    phase_start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - phase_start) <
           pdMS_TO_TICKS(PAN_CAL_FINAL_HOLD_MS))
    {
        (void)xTaskDelayUntil(&last_wake, period_ticks);
        s_snapshot.task_cycles++;
    }

    TimerPwm_Disable();
    s_snapshot.outputs_enabled = false;
    SetState(SERVO_CAL_STATE_DONE_SAFE);

    for (;;)
    {
        (void)xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000U));
        s_snapshot.task_cycles++;
    }
}

static bool ValidateConfiguration(void)
{
    if ((PAN_CAL_REFERENCE_US < TIMER_PWM_SERVO_MIN_US) ||
        (PAN_CAL_REFERENCE_US > TIMER_PWM_SERVO_MAX_US) ||
        (PAN_CAL_TARGET_US < TIMER_PWM_SERVO_MIN_US) ||
        (PAN_CAL_TARGET_US > TIMER_PWM_SERVO_MAX_US) ||
        (PAN_CAL_SLEW_US_PER_CYCLE == 0U) ||
        (PAN_CAL_TASK_PERIOD_MS == 0U))
    {
        return false;
    }

    return true;
}

static bool MovePanToward(uint16_t target_us)
{
    uint16_t current_us;
    uint16_t next_us;
    uint16_t delta_us;

    current_us = s_snapshot.applied_pan_us;
    next_us = current_us;

    if (current_us < target_us)
    {
        delta_us = (uint16_t)(target_us - current_us);
        if (delta_us > PAN_CAL_SLEW_US_PER_CYCLE)
        {
            delta_us = PAN_CAL_SLEW_US_PER_CYCLE;
        }
        next_us = (uint16_t)(current_us + delta_us);
    }
    else if (current_us > target_us)
    {
        delta_us = (uint16_t)(current_us - target_us);
        if (delta_us > PAN_CAL_SLEW_US_PER_CYCLE)
        {
            delta_us = PAN_CAL_SLEW_US_PER_CYCLE;
        }
        next_us = (uint16_t)(current_us - delta_us);
    }

    if (next_us == current_us)
    {
        return true;
    }

    if (!TimerPwm_SetPanPulseUs(next_us))
    {
        return false;
    }

    s_snapshot.applied_pan_us = next_us;
    s_snapshot.pwm_update_count++;
    return true;
}

static void SetState(ServoCalibrationState state)
{
    s_snapshot.state = state;
}

static void EnterFault(void)
{
    TickType_t last_wake;

    TimerPwm_Disable();
    s_snapshot.outputs_enabled = false;
    s_snapshot.failure_count++;
    SetState(SERVO_CAL_STATE_FAULT);

    last_wake = xTaskGetTickCount();
    for (;;)
    {
        (void)xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000U));
        s_snapshot.task_cycles++;
    }
}
