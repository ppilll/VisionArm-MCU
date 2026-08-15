#include "gimbal_stub.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "actuator_driver.h"
#include "app_config.h"
#include "control_mailbox.h"
#include "protocol_state.h"

static GimbalStubSnapshot s_output;
static GimbalSafetyContext s_safety;
static TaskHandle_t s_gimbal_task;
static StaticTask_t s_gimbal_task_tcb;
static StackType_t s_gimbal_task_stack[APP_GIMBAL_TASK_STACK_WORDS];

static void GimbalTask(void *argument);
static void SetOutput(int16_t pan_q15,
                      int16_t tilt_q15,
                      bool active,
                      const GimbalSafetyDecision *decision,
                      bool actuator_enabled);
static void RecordSafetyTransition(GimbalSafetyReason previous,
                                   GimbalSafetyReason current);

bool GimbalStubTask_Create(void)
{
    bool self_test_passed;

    if (s_gimbal_task != NULL)
    {
        return false;
    }

    (void)memset(&s_output, 0, sizeof(s_output));
    GimbalSafety_Init(&s_safety);

    self_test_passed = GimbalSafety_RunSelfTest();
    s_output.safety_self_test_passed = self_test_passed;
    s_output.safety_reason = GIMBAL_SAFETY_BOOT_SAFE;

    /* A failed safety-policy self-test is a startup-fatal condition. */
    if (!self_test_passed)
    {
        ActuatorDriver_Disable();
        return false;
    }

    s_gimbal_task = xTaskCreateStatic(GimbalTask,
                                      "GimbalSafety",
                                      APP_GIMBAL_TASK_STACK_WORDS,
                                      NULL,
                                      APP_GIMBAL_TASK_PRIORITY,
                                      s_gimbal_task_stack,
                                      &s_gimbal_task_tcb);

    return (s_gimbal_task != NULL);
}

void GimbalStub_GetSnapshot(GimbalStubSnapshot *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = s_output;
    taskEXIT_CRITICAL();
}

static void GimbalTask(void *argument)
{
    TickType_t last_wake;
    ControlCommand command = {0};
    ProtocolStateSnapshot state;
    ActuatorDriverSnapshot actuator;
    GimbalSafetyInput input;
    GimbalSafetyDecision decision;
    GimbalSafetyReason previous_reason;
    bool have_command;

    (void)argument;
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        (void)xTaskDelayUntil(&last_wake,
                              pdMS_TO_TICKS(APP_GIMBAL_TASK_PERIOD_MS));

        ProtocolState_GetSnapshot(&state);
        have_command = ControlMailbox_Read(&command);
        ActuatorDriver_GetSnapshot(&actuator);

        input.link_ready = (state.link_state == PROTOCOL_LINK_READY);
        input.remote_stop_latched = state.remote_stop_latched;
        input.control_valid = state.control_valid;
        input.mailbox_available = have_command;
        input.mailbox_valid = have_command && command.valid;
        input.mailbox_generation = have_command ? command.generation : 0U;
        input.actuator_initialized = actuator.initialized;
        input.local_fault = (actuator.fault_count != 0U);

        previous_reason = s_safety.last_reason;
        decision = GimbalSafety_Evaluate(&s_safety, &input);
        RecordSafetyTransition(previous_reason, decision.reason);

        if (!decision.control_enabled)
        {
            ActuatorDriver_Disable();
            SetOutput(0,
                      0,
                      false,
                      &decision,
                      false);
            continue;
        }

        /*
         * Step H intentionally contains no AxisController/GimbalController.
         * A valid safety lease only enables the calibrated CENTER command.
         * Error-to-motion mapping is introduced later in Steps I/J.
         */
        if (!actuator.enabled)
        {
            if (!ActuatorDriver_Enable())
            {
                ActuatorDriver_Disable();

                decision.control_enabled = false;
                decision.reason = GIMBAL_SAFETY_LOCAL_FAULT;

                SetOutput(0,
                          0,
                          false,
                          &decision,
                          false);
                continue;
            }
        }

        SetOutput(command.control.error_x_q15,
                  command.control.error_y_q15,
                  true,
                  &decision,
                  true);
    }
}

static void SetOutput(int16_t pan_q15,
                      int16_t tilt_q15,
                      bool active,
                      const GimbalSafetyDecision *decision,
                      bool actuator_enabled)
{
    taskENTER_CRITICAL();

    s_output.pan_q15 = pan_q15;
    s_output.tilt_q15 = tilt_q15;
    s_output.active = active;
    s_output.actuator_enabled = actuator_enabled;

    if (decision != NULL)
    {
        s_output.safety_reason = decision->reason;
        s_output.waiting_for_fresh_control =
            decision->waiting_for_fresh_control;
        s_output.mailbox_generation = decision->mailbox_generation;
        s_output.accepted_generation = decision->accepted_generation;
    }

    s_output.safety_transition_count = s_safety.transition_count;

    if (!active)
    {
        s_output.safety_reject_count++;
    }

    taskEXIT_CRITICAL();
}

static void RecordSafetyTransition(GimbalSafetyReason previous,
                                   GimbalSafetyReason current)
{
    if (previous == current)
    {
        return;
    }

    taskENTER_CRITICAL();

    if (current == GIMBAL_SAFETY_REMOTE_STOP)
    {
        s_output.remote_stop_count++;
    }
    else if (current == GIMBAL_SAFETY_LINK_NOT_READY)
    {
        s_output.link_not_ready_count++;
    }
    else if (current == GIMBAL_SAFETY_CONTROL_INVALID)
    {
        s_output.control_invalid_count++;
    }

    taskEXIT_CRITICAL();
}
