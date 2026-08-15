#include "gimbal_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "actuator_driver.h"
#include "app_config.h"
#include "control_mailbox.h"
#include "gimbal_controller.h"
#include "gimbal_runtime.h"
#include "gimbal_safety.h"
#include "protocol_state.h"

static TaskHandle_t s_task;
static StaticTask_t s_task_tcb;
static StackType_t s_task_stack[APP_GIMBAL_TASK_STACK_WORDS];

static GimbalSafetyContext s_safety;
static GimbalController s_controller;
static bool s_local_fault;

static void GimbalTaskEntry(void *argument);
static GimbalSafetyInput BuildSafetyInput(
    const ProtocolStateSnapshot *state,
    const ActuatorDriverSnapshot *actuator,
    bool have_command,
    const ControlCommand *command);

bool GimbalTask_Create(void)
{
    bool safety_self_test_passed;
    bool controller_self_test_passed;

    if (s_task != NULL)
    {
        return false;
    }

    safety_self_test_passed = GimbalSafety_RunSelfTest();
    controller_self_test_passed = GimbalController_RunSelfTest();

    GimbalRuntime_Init(safety_self_test_passed,
                       controller_self_test_passed);

    if (!safety_self_test_passed ||
        !controller_self_test_passed)
    {
        ActuatorDriver_Disable();
        return false;
    }

    GimbalSafety_Init(&s_safety);
    s_local_fault = false;

    if (!GimbalController_Init(&s_controller))
    {
        ActuatorDriver_Disable();
        return false;
    }

    s_task = xTaskCreateStatic(GimbalTaskEntry,
                               "GimbalCtrl",
                               APP_GIMBAL_TASK_STACK_WORDS,
                               NULL,
                               APP_GIMBAL_TASK_PRIORITY,
                               s_task_stack,
                               &s_task_tcb);

    return (s_task != NULL);
}

static void GimbalTaskEntry(void *argument)
{
    TickType_t last_wake;
    ControlCommand command = {0};
    ProtocolStateSnapshot state;
    ActuatorDriverSnapshot actuator;
    ActuatorApplyResult actuator_result;
    GimbalSafetyInput safety_input;
    GimbalSafetyDecision safety_decision;
    GimbalControllerOutput controller_output;
    bool have_command;

    (void)argument;

    last_wake = xTaskGetTickCount();

    for (;;)
    {
        (void)xTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(APP_GIMBAL_TASK_PERIOD_MS));

        ProtocolState_GetSnapshot(&state);
        have_command = ControlMailbox_Read(&command);
        ActuatorDriver_GetSnapshot(&actuator);

        safety_input = BuildSafetyInput(&state,
                                        &actuator,
                                        have_command,
                                        &command);

        safety_decision = GimbalSafety_Evaluate(&s_safety,
                                                &safety_input);

        if (!safety_decision.control_enabled)
        {
            ActuatorDriver_Disable();
            GimbalController_ResetCenter(&s_controller);
            ActuatorDriver_GetSnapshot(&actuator);

            GimbalRuntime_RecordSafe(
                safety_decision.reason,
                safety_decision.waiting_for_fresh_control,
                safety_decision.mailbox_generation,
                safety_decision.accepted_generation,
                actuator.applied_pan_us,
                actuator.applied_tilt_us);

            continue;
        }

        if (!actuator.enabled)
        {
            /*
             * A fresh safety lease always re-enters through calibrated center.
             * Internal controller state is reset to the same center so the
             * first controlled step is bounded by the configured slew limit.
             */
            if (!ActuatorDriver_Enable())
            {
                s_local_fault = true;
                ActuatorDriver_Disable();
                GimbalRuntime_RecordActuatorApplyFailure();
                GimbalController_ResetCenter(&s_controller);
                ActuatorDriver_GetSnapshot(&actuator);
                GimbalRuntime_RecordSafe(
                    GIMBAL_SAFETY_LOCAL_FAULT,
                    true,
                    safety_decision.mailbox_generation,
                    safety_decision.accepted_generation,
                    actuator.applied_pan_us,
                    actuator.applied_tilt_us);
                continue;
            }

            GimbalController_ResetCenter(&s_controller);
        }

        if (!GimbalController_Step(
                &s_controller,
                command.control.error_x_q15,
                command.control.error_y_q15,
                &controller_output))
        {
            s_local_fault = true;
            ActuatorDriver_Disable();
            GimbalRuntime_RecordControllerFault();
            GimbalController_ResetCenter(&s_controller);
            ActuatorDriver_GetSnapshot(&actuator);
            GimbalRuntime_RecordSafe(
                GIMBAL_SAFETY_LOCAL_FAULT,
                true,
                safety_decision.mailbox_generation,
                safety_decision.accepted_generation,
                actuator.applied_pan_us,
                actuator.applied_tilt_us);
            continue;
        }

        actuator_result = (ActuatorApplyResult){0};

        if (!ActuatorDriver_Apply(
                &controller_output.command,
                &actuator_result))
        {
            s_local_fault = true;
            ActuatorDriver_Disable();
            GimbalRuntime_RecordActuatorApplyFailure();
            GimbalController_ResetCenter(&s_controller);
            ActuatorDriver_GetSnapshot(&actuator);
            GimbalRuntime_RecordSafe(
                GIMBAL_SAFETY_LOCAL_FAULT,
                true,
                safety_decision.mailbox_generation,
                safety_decision.accepted_generation,
                actuator.applied_pan_us,
                actuator.applied_tilt_us);
            continue;
        }

        GimbalRuntime_RecordActive(
            safety_decision.reason,
            safety_decision.mailbox_generation,
            safety_decision.accepted_generation,
            command.control.error_x_q15,
            command.control.error_y_q15,
            controller_output.pan.target_us,
            controller_output.tilt.target_us,
            actuator_result.applied_pan_us,
            actuator_result.applied_tilt_us,
            controller_output.pan.dead_zone_active,
            controller_output.tilt.dead_zone_active,
            controller_output.pan.limit_blocked || actuator_result.pan_saturated,
            controller_output.tilt.limit_blocked || actuator_result.tilt_saturated,
            controller_output.pan.slew_limited,
            controller_output.tilt.slew_limited);
    }
}

static GimbalSafetyInput BuildSafetyInput(
    const ProtocolStateSnapshot *state,
    const ActuatorDriverSnapshot *actuator,
    bool have_command,
    const ControlCommand *command)
{
    GimbalSafetyInput input = {0};

    if ((state == NULL) || (actuator == NULL))
    {
        input.local_fault = true;
        return input;
    }

    input.link_ready =
        (state->link_state == PROTOCOL_LINK_READY);

    input.remote_stop_latched =
        state->remote_stop_latched;

    input.control_valid =
        state->control_valid;

    input.mailbox_available =
        have_command && (command != NULL);

    input.mailbox_valid =
        input.mailbox_available && command->valid;

    input.mailbox_generation =
        input.mailbox_available ? command->generation : 0U;

    input.actuator_initialized =
        actuator->initialized;

    input.local_fault =
        s_local_fault || (actuator->fault_count != 0U);

    return input;
}
