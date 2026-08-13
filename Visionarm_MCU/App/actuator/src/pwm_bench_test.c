#include "pwm_bench_test.h"

#include "FreeRTOS.h"
#include "task.h"

#include "timer_pwm.h"

#define PWM_BENCH_TASK_PRIORITY          1U
#define PWM_BENCH_TASK_STACK_WORDS       192U
#define PWM_BENCH_PHASE_DURATION_MS      3000U

static TaskHandle_t s_task;
static StaticTask_t s_task_tcb;
static StackType_t s_task_stack[PWM_BENCH_TASK_STACK_WORDS];
static PwmBenchSnapshot s_snapshot;

static void PwmBenchTask(void *argument);
static void SetPhase(PwmBenchPhase phase);
static void EnterFault(void);

bool PwmBenchTest_Create(void)
{
    if (s_task != NULL)
    {
        return false;
    }

    s_snapshot = (PwmBenchSnapshot){0};
    s_snapshot.phase = PWM_BENCH_PHASE_BOOT_SAFE;

    s_task = xTaskCreateStatic(PwmBenchTask,
                               "PwmBench",
                               PWM_BENCH_TASK_STACK_WORDS,
                               NULL,
                               PWM_BENCH_TASK_PRIORITY,
                               s_task_stack,
                               &s_task_tcb);

    return (s_task != NULL);
}

void PwmBenchTest_GetSnapshot(PwmBenchSnapshot *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = s_snapshot;
    taskEXIT_CRITICAL();
}

static void PwmBenchTask(void *argument)
{
    TickType_t last_wake;
    const TickType_t phase_ticks = pdMS_TO_TICKS(PWM_BENCH_PHASE_DURATION_MS);

    (void)argument;
    last_wake = xTaskGetTickCount();

    /*
     * Phase 0: deliberate low-output window after the scheduler starts.
     * This gives the logic analyzer an obvious boot-safe interval.
     */
    TimerPwm_Disable();
    SetPhase(PWM_BENCH_PHASE_BOOT_SAFE);
    (void)xTaskDelayUntil(&last_wake, phase_ticks);

    for (;;)
    {
        /* PA6 / TIM3_CH1 = 500 us, PA7 / TIM3_CH2 = 2500 us. */
        if (!TimerPwm_Enable(500U, 2500U))
        {
            EnterFault();
        }
        SetPhase(PWM_BENCH_PHASE_PAN_500_TILT_2500);
        (void)xTaskDelayUntil(&last_wake, phase_ticks);

        /* Both channels at nominal center pulse width. */
        if (!TimerPwm_SetBothPulseUs(1500U, 1500U))
        {
            EnterFault();
        }
        SetPhase(PWM_BENCH_PHASE_BOTH_1500);
        (void)xTaskDelayUntil(&last_wake, phase_ticks);

        /* Reverse the channel widths to prove channel/pin identity. */
        if (!TimerPwm_SetBothPulseUs(2500U, 500U))
        {
            EnterFault();
        }
        SetPhase(PWM_BENCH_PHASE_PAN_2500_TILT_500);
        (void)xTaskDelayUntil(&last_wake, phase_ticks);

        /* Explicit safe-output phase: both pins are push-pull low. */
        TimerPwm_Disable();
        SetPhase(PWM_BENCH_PHASE_OUTPUT_DISABLED);
        (void)xTaskDelayUntil(&last_wake, phase_ticks);

        taskENTER_CRITICAL();
        s_snapshot.completed_cycles++;
        taskEXIT_CRITICAL();
    }
}

static void SetPhase(PwmBenchPhase phase)
{
    taskENTER_CRITICAL();
    s_snapshot.phase = phase;
    taskEXIT_CRITICAL();
}

static void EnterFault(void)
{
    TickType_t last_wake;

    TimerPwm_Disable();

    taskENTER_CRITICAL();
    s_snapshot.phase = PWM_BENCH_PHASE_FAULT;
    s_snapshot.failure_count++;
    taskEXIT_CRITICAL();

    /*
     * This project intentionally has INCLUDE_vTaskSuspend == 0 and
     * INCLUDE_vTaskDelete == 0. Park the failed bench task with the already
     * enabled xTaskDelayUntil() API instead of spinning or changing V5 RTOS
     * configuration.
     */
    last_wake = xTaskGetTickCount();
    for (;;)
    {
        (void)xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000U));
    }
}
