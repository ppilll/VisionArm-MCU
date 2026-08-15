#include "actuator_driver_test.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

#include "actuator_driver.h"
#include "gimbal_config.h"

#define ACTUATOR_TEST_TASK_PRIORITY             1U
#define ACTUATOR_TEST_TASK_STACK_WORDS          256U

static TaskHandle_t s_task;
static StaticTask_t s_task_tcb;
static StackType_t s_task_stack[ACTUATOR_TEST_TASK_STACK_WORDS];
static ActuatorDriverTestSnapshot s_snapshot;

static void ActuatorDriverTestTask(void *argument);
static bool ApplyCommand(int32_t pan_us, int32_t tilt_us);
static bool RampTo(uint16_t pan_target_us,
                   uint16_t tilt_target_us,
                   TickType_t *last_wake,
                   TickType_t period_ticks);
static bool ExerciseTarget(ActuatorDriverTestState target_state,
                           uint16_t pan_target_us,
                           uint16_t tilt_target_us,
                           TickType_t *last_wake,
                           TickType_t period_ticks);
static uint16_t StepToward(uint16_t current,
                           uint16_t target,
                           uint16_t max_step);
static void HoldForMs(uint32_t hold_ms,
                      TickType_t *last_wake,
                      TickType_t period_ticks);
static void SetState(ActuatorDriverTestState state);
static void EnterFault(void);

bool ActuatorDriverTest_Create(void)
{
    if (s_task != NULL)
    {
        return false;
    }

    s_snapshot = (ActuatorDriverTestSnapshot){0};
    s_snapshot.state = ACTUATOR_TEST_STATE_BOOT_SAFE;
    s_snapshot.applied_pan_us = GIMBAL_PAN_CENTER_US;
    s_snapshot.applied_tilt_us = GIMBAL_TILT_CENTER_US;

    s_task = xTaskCreateStatic(ActuatorDriverTestTask,
                               "ActDrvTest",
                               ACTUATOR_TEST_TASK_STACK_WORDS,
                               NULL,
                               ACTUATOR_TEST_TASK_PRIORITY,
                               s_task_stack,
                               &s_task_tcb);

    return (s_task != NULL);
}

void ActuatorDriverTest_GetSnapshot(ActuatorDriverTestSnapshot *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = s_snapshot;
    taskEXIT_CRITICAL();
}

static void ActuatorDriverTestTask(void *argument)
{
    TickType_t last_wake;
    TickType_t period_ticks;

    (void)argument;

    period_ticks = pdMS_TO_TICKS(ACTUATOR_TEST_TASK_PERIOD_MS);
    last_wake = xTaskGetTickCount();

    ActuatorDriver_Disable();
    SetState(ACTUATOR_TEST_STATE_BOOT_SAFE);
    HoldForMs(ACTUATOR_TEST_BOOT_SAFE_MS, &last_wake, period_ticks);

    if (!ActuatorDriver_Enable())
    {
        EnterFault();
    }

    s_snapshot.applied_pan_us = GIMBAL_PAN_CENTER_US;
    s_snapshot.applied_tilt_us = GIMBAL_TILT_CENTER_US;
    SetState(ACTUATOR_TEST_STATE_CENTER);
    HoldForMs(ACTUATOR_TEST_INITIAL_CENTER_HOLD_MS,
              &last_wake,
              period_ticks);

    /* Pan only: center -> left -> center. */
    if (!ExerciseTarget(ACTUATOR_TEST_STATE_PAN_LEFT,
                        ACTUATOR_TEST_PAN_LEFT_US,
                        GIMBAL_TILT_CENTER_US,
                        &last_wake,
                        period_ticks))
    {
        EnterFault();
    }

    /* Pan only: center -> right -> center. */
    if (!ExerciseTarget(ACTUATOR_TEST_STATE_PAN_RIGHT,
                        ACTUATOR_TEST_PAN_RIGHT_US,
                        GIMBAL_TILT_CENTER_US,
                        &last_wake,
                        period_ticks))
    {
        EnterFault();
    }

    /* Tilt only: center -> down -> center. */
    if (!ExerciseTarget(ACTUATOR_TEST_STATE_TILT_DOWN,
                        GIMBAL_PAN_CENTER_US,
                        ACTUATOR_TEST_TILT_DOWN_US,
                        &last_wake,
                        period_ticks))
    {
        EnterFault();
    }

    /* Tilt only: center -> up -> center. */
    if (!ExerciseTarget(ACTUATOR_TEST_STATE_TILT_UP,
                        GIMBAL_PAN_CENTER_US,
                        ACTUATOR_TEST_TILT_UP_US,
                        &last_wake,
                        period_ticks))
    {
        EnterFault();
    }

    /* Both channels update together: camera left + down. */
    if (!ExerciseTarget(ACTUATOR_TEST_STATE_DUAL_LEFT_DOWN,
                        ACTUATOR_TEST_PAN_LEFT_US,
                        ACTUATOR_TEST_TILT_DOWN_US,
                        &last_wake,
                        period_ticks))
    {
        EnterFault();
    }

    /* Both channels update together: camera right + up. */
    if (!ExerciseTarget(ACTUATOR_TEST_STATE_DUAL_RIGHT_UP,
                        ACTUATOR_TEST_PAN_RIGHT_US,
                        ACTUATOR_TEST_TILT_UP_US,
                        &last_wake,
                        period_ticks))
    {
        EnterFault();
    }

    ActuatorDriver_Disable();
    SetState(ACTUATOR_TEST_STATE_DONE_SAFE);

    /* One-shot test: never repeat autonomous physical motion. */
    for (;;)
    {
        (void)xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000U));
        s_snapshot.task_cycles++;
    }
}

static bool ExerciseTarget(ActuatorDriverTestState target_state,
                           uint16_t pan_target_us,
                           uint16_t tilt_target_us,
                           TickType_t *last_wake,
                           TickType_t period_ticks)
{
    SetState(target_state);

    if (!RampTo(pan_target_us,
                tilt_target_us,
                last_wake,
                period_ticks))
    {
        return false;
    }

    HoldForMs(ACTUATOR_TEST_TARGET_HOLD_MS,
              last_wake,
              period_ticks);

    SetState(ACTUATOR_TEST_STATE_RETURN_CENTER);

    if (!RampTo(GIMBAL_PAN_CENTER_US,
                GIMBAL_TILT_CENTER_US,
                last_wake,
                period_ticks))
    {
        return false;
    }

    HoldForMs(ACTUATOR_TEST_CENTER_HOLD_MS,
              last_wake,
              period_ticks);

    return true;
}

static bool ApplyCommand(int32_t pan_us, int32_t tilt_us)
{
    ActuatorCommand command;
    ActuatorApplyResult result;

    command.pan_position_us = pan_us;
    command.tilt_position_us = tilt_us;
    result = (ActuatorApplyResult){0};

    if (!ActuatorDriver_Apply(&command, &result))
    {
        return false;
    }

    s_snapshot.command_count++;
    s_snapshot.applied_pan_us = result.applied_pan_us;
    s_snapshot.applied_tilt_us = result.applied_tilt_us;
    s_snapshot.last_pan_saturated = result.pan_saturated;
    s_snapshot.last_tilt_saturated = result.tilt_saturated;

    /* Physical-small profile must never hit installed software limits. */
    if (result.pan_saturated || result.tilt_saturated)
    {
        return false;
    }

    return true;
}

static bool RampTo(uint16_t pan_target_us,
                   uint16_t tilt_target_us,
                   TickType_t *last_wake,
                   TickType_t period_ticks)
{
    uint16_t pan_current;
    uint16_t tilt_current;

    pan_current = s_snapshot.applied_pan_us;
    tilt_current = s_snapshot.applied_tilt_us;

    while ((pan_current != pan_target_us) ||
           (tilt_current != tilt_target_us))
    {
        pan_current = StepToward(pan_current,
                                 pan_target_us,
                                 ACTUATOR_TEST_SLEW_US_PER_CYCLE);

        tilt_current = StepToward(tilt_current,
                                  tilt_target_us,
                                  ACTUATOR_TEST_SLEW_US_PER_CYCLE);

        if (!ApplyCommand(pan_current, tilt_current))
        {
            return false;
        }

        (void)xTaskDelayUntil(last_wake, period_ticks);
        s_snapshot.task_cycles++;
    }

    return true;
}

static uint16_t StepToward(uint16_t current,
                           uint16_t target,
                           uint16_t max_step)
{
    uint16_t delta;

    if (current < target)
    {
        delta = (uint16_t)(target - current);
        if (delta > max_step)
        {
            delta = max_step;
        }
        return (uint16_t)(current + delta);
    }

    if (current > target)
    {
        delta = (uint16_t)(current - target);
        if (delta > max_step)
        {
            delta = max_step;
        }
        return (uint16_t)(current - delta);
    }

    return current;
}

static void HoldForMs(uint32_t hold_ms,
                      TickType_t *last_wake,
                      TickType_t period_ticks)
{
    TickType_t start;

    start = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(hold_ms))
    {
        (void)xTaskDelayUntil(last_wake, period_ticks);
        s_snapshot.task_cycles++;
    }
}

static void SetState(ActuatorDriverTestState state)
{
    s_snapshot.state = state;
}

static void EnterFault(void)
{
    TickType_t last_wake;

    ActuatorDriver_Disable();
    s_snapshot.failure_count++;
    SetState(ACTUATOR_TEST_STATE_FAULT);

    last_wake = xTaskGetTickCount();

    for (;;)
    {
        (void)xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000U));
        s_snapshot.task_cycles++;
    }
}
