#include "gimbal_task.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

#include "actuator.h"
#include "protocol_core.h"
#include "visionarm_app.h"

static TaskHandle_t s_task;
static StaticTask_t s_task_tcb;
static StackType_t s_task_stack[APP_GIMBAL_TASK_STACK_WORDS];

static GimbalSafetyContext s_safety;
static GimbalControlState s_control;
static GimbalRuntimeSnapshot s_runtime;
static bool s_local_fault;

static void GimbalTaskEntry(void *argument);
static GimbalSafetyInput BuildSafetyInput(
    const ProtocolStateSnapshot *state,
    const ActuatorSnapshot *actuator,
    bool have_command,
    const ControlCommand *command);
static void RuntimeInit(void);
static void RuntimeRecordSafe(const GimbalSafetyDecision *decision,
                              uint16_t pan_applied_us,
                              uint16_t tilt_applied_us);
static void RuntimeRecordActive(const GimbalSafetyDecision *decision,
                                const ControlCommand *command,
                                const GimbalControlOutput *control_output,
                                const ActuatorApplyResult *actuator_result);
static void RuntimeRecordControllerFault(void);
static void RuntimeRecordActuatorFailure(void);
static int16_t PulseToQ15(uint16_t pulse_us,
                          uint16_t minimum_us,
                          uint16_t center_us,
                          uint16_t maximum_us);

bool GimbalTask_Create(void)
{
    if (s_task != NULL)
    {
        return false;
    }

    GimbalSafety_Init(&s_safety);
    s_local_fault = false;
    RuntimeInit();

    if (!GimbalControl_Init(&s_control))
    {
        Actuator_Disable();
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

void GimbalTask_GetSnapshot(GimbalRuntimeSnapshot *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = s_runtime;
    taskEXIT_CRITICAL();
}

static void GimbalTaskEntry(void *argument)
{
    TickType_t last_wake;
    ControlCommand command = {0};
    ProtocolStateSnapshot state;
    ActuatorSnapshot actuator;
    ActuatorApplyResult actuator_result;
    GimbalSafetyInput safety_input;
    GimbalSafetyDecision safety_decision;
    GimbalControlOutput control_output;
    bool have_command;

    (void)argument;
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        (void)xTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(APP_GIMBAL_TASK_PERIOD_MS));

        ProtocolCore_GetState(&state);
        have_command = ProtocolCore_ReadControl(&command);
        Actuator_GetSnapshot(&actuator);

        safety_input = BuildSafetyInput(&state,
                                        &actuator,
                                        have_command,
                                        &command);
        safety_decision = GimbalSafety_Evaluate(&s_safety, &safety_input);

        if (!safety_decision.control_enabled)
        {
            Actuator_Disable();
            GimbalControl_ResetCenter(&s_control);
            Actuator_GetSnapshot(&actuator);
            RuntimeRecordSafe(&safety_decision,
                              actuator.applied_pan_us,
                              actuator.applied_tilt_us);
            continue;
        }

        if (!actuator.enabled)
        {
            /* Every fresh safety lease re-enters through calibrated center. */
            if (!Actuator_Enable())
            {
                s_local_fault = true;
                Actuator_Disable();
                RuntimeRecordActuatorFailure();
                GimbalControl_ResetCenter(&s_control);
                Actuator_GetSnapshot(&actuator);
                safety_decision.control_enabled = false;
                safety_decision.reason = GIMBAL_SAFETY_LOCAL_FAULT;
                safety_decision.waiting_for_fresh_control = true;
                RuntimeRecordSafe(&safety_decision,
                                  actuator.applied_pan_us,
                                  actuator.applied_tilt_us);
                continue;
            }

            GimbalControl_ResetCenter(&s_control);
        }

        if (!GimbalControl_Step(&s_control,
                                command.control.error_x_q15,
                                command.control.error_y_q15,
                                &control_output))
        {
            s_local_fault = true;
            Actuator_Disable();
            RuntimeRecordControllerFault();
            GimbalControl_ResetCenter(&s_control);
            Actuator_GetSnapshot(&actuator);
            safety_decision.control_enabled = false;
            safety_decision.reason = GIMBAL_SAFETY_LOCAL_FAULT;
            safety_decision.waiting_for_fresh_control = true;
            RuntimeRecordSafe(&safety_decision,
                              actuator.applied_pan_us,
                              actuator.applied_tilt_us);
            continue;
        }

        actuator_result = (ActuatorApplyResult){0};
        if (!Actuator_Apply(&control_output.command, &actuator_result))
        {
            s_local_fault = true;
            Actuator_Disable();
            RuntimeRecordActuatorFailure();
            GimbalControl_ResetCenter(&s_control);
            Actuator_GetSnapshot(&actuator);
            safety_decision.control_enabled = false;
            safety_decision.reason = GIMBAL_SAFETY_LOCAL_FAULT;
            safety_decision.waiting_for_fresh_control = true;
            RuntimeRecordSafe(&safety_decision,
                              actuator.applied_pan_us,
                              actuator.applied_tilt_us);
            continue;
        }

        RuntimeRecordActive(&safety_decision,
                            &command,
                            &control_output,
                            &actuator_result);
    }
}

static GimbalSafetyInput BuildSafetyInput(
    const ProtocolStateSnapshot *state,
    const ActuatorSnapshot *actuator,
    bool have_command,
    const ControlCommand *command)
{
    GimbalSafetyInput input = {0};

    if ((state == NULL) || (actuator == NULL))
    {
        input.local_fault = true;
        return input;
    }

    input.link_ready = (state->link_state == PROTOCOL_LINK_READY);
    input.remote_stop_latched = state->remote_stop_latched;
    input.control_valid = state->control_valid;
    input.mailbox_available = have_command && (command != NULL);
    input.mailbox_valid = input.mailbox_available && command->valid;
    input.mailbox_generation =
        input.mailbox_available ? command->generation : 0U;
    input.actuator_initialized = actuator->initialized;
    input.local_fault = s_local_fault || (actuator->fault_count != 0U);
    return input;
}

static void RuntimeInit(void)
{
    s_runtime = (GimbalRuntimeSnapshot){0};
    s_runtime.safety_reason = GIMBAL_SAFETY_BOOT_SAFE;
    s_runtime.pan_target_us = ACTUATOR_PAN_CENTER_US;
    s_runtime.tilt_target_us = ACTUATOR_TILT_CENTER_US;
    s_runtime.pan_applied_us = ACTUATOR_PAN_CENTER_US;
    s_runtime.tilt_applied_us = ACTUATOR_TILT_CENTER_US;
}

static void RuntimeRecordSafe(const GimbalSafetyDecision *decision,
                              uint16_t pan_applied_us,
                              uint16_t tilt_applied_us)
{
    if (decision == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();

    s_runtime.task_cycle_count++;
    s_runtime.active = false;
    s_runtime.actuator_enabled = false;
    s_runtime.waiting_for_fresh_control =
        decision->waiting_for_fresh_control;
    s_runtime.safety_reason = decision->reason;
    s_runtime.input_error_x_q15 = 0;
    s_runtime.input_error_y_q15 = 0;
    s_runtime.pan_target_us = ACTUATOR_PAN_CENTER_US;
    s_runtime.tilt_target_us = ACTUATOR_TILT_CENTER_US;
    s_runtime.pan_applied_us = pan_applied_us;
    s_runtime.tilt_applied_us = tilt_applied_us;
    s_runtime.pan_applied_q15 =
        PulseToQ15(pan_applied_us,
                   ACTUATOR_PAN_SAFE_MIN_US,
                   ACTUATOR_PAN_CENTER_US,
                   ACTUATOR_PAN_SAFE_MAX_US);
    s_runtime.tilt_applied_q15 =
        PulseToQ15(tilt_applied_us,
                   ACTUATOR_TILT_SAFE_MIN_US,
                   ACTUATOR_TILT_CENTER_US,
                   ACTUATOR_TILT_SAFE_MAX_US);
    s_runtime.pan_dead_zone_active = true;
    s_runtime.tilt_dead_zone_active = true;
    s_runtime.pan_limit_blocked = false;
    s_runtime.tilt_limit_blocked = false;
    s_runtime.mailbox_generation = decision->mailbox_generation;
    s_runtime.accepted_generation = decision->accepted_generation;
    s_runtime.safety_reject_count++;

    taskEXIT_CRITICAL();
}

static void RuntimeRecordActive(const GimbalSafetyDecision *decision,
                                const ControlCommand *command,
                                const GimbalControlOutput *control_output,
                                const ActuatorApplyResult *actuator_result)
{
    if ((decision == NULL) ||
        (command == NULL) ||
        (control_output == NULL) ||
        (actuator_result == NULL))
    {
        return;
    }

    taskENTER_CRITICAL();

    s_runtime.task_cycle_count++;
    s_runtime.control_cycle_count++;
    s_runtime.active = true;
    s_runtime.actuator_enabled = true;
    s_runtime.waiting_for_fresh_control = false;
    s_runtime.safety_reason = decision->reason;
    s_runtime.input_error_x_q15 = command->control.error_x_q15;
    s_runtime.input_error_y_q15 = command->control.error_y_q15;
    s_runtime.pan_target_us = control_output->pan.target_us;
    s_runtime.tilt_target_us = control_output->tilt.target_us;
    s_runtime.pan_applied_us = actuator_result->applied_pan_us;
    s_runtime.tilt_applied_us = actuator_result->applied_tilt_us;
    s_runtime.pan_applied_q15 =
        PulseToQ15(actuator_result->applied_pan_us,
                   ACTUATOR_PAN_SAFE_MIN_US,
                   ACTUATOR_PAN_CENTER_US,
                   ACTUATOR_PAN_SAFE_MAX_US);
    s_runtime.tilt_applied_q15 =
        PulseToQ15(actuator_result->applied_tilt_us,
                   ACTUATOR_TILT_SAFE_MIN_US,
                   ACTUATOR_TILT_CENTER_US,
                   ACTUATOR_TILT_SAFE_MAX_US);
    s_runtime.pan_dead_zone_active = control_output->pan.dead_zone_active;
    s_runtime.tilt_dead_zone_active = control_output->tilt.dead_zone_active;
    s_runtime.pan_limit_blocked =
        control_output->pan.limit_blocked || actuator_result->pan_saturated;
    s_runtime.tilt_limit_blocked =
        control_output->tilt.limit_blocked || actuator_result->tilt_saturated;
    s_runtime.mailbox_generation = decision->mailbox_generation;
    s_runtime.accepted_generation = decision->accepted_generation;
    s_runtime.last_applied_generation = decision->mailbox_generation;

    if (s_runtime.pan_limit_blocked)
    {
        s_runtime.pan_limit_hit_count++;
    }
    if (s_runtime.tilt_limit_blocked)
    {
        s_runtime.tilt_limit_hit_count++;
    }
    if (control_output->pan.slew_limited)
    {
        s_runtime.pan_slew_limit_count++;
    }
    if (control_output->tilt.slew_limited)
    {
        s_runtime.tilt_slew_limit_count++;
    }

    taskEXIT_CRITICAL();
}

static void RuntimeRecordControllerFault(void)
{
    taskENTER_CRITICAL();
    s_runtime.controller_fault_count++;
    taskEXIT_CRITICAL();
}

static void RuntimeRecordActuatorFailure(void)
{
    taskENTER_CRITICAL();
    s_runtime.actuator_apply_failure_count++;
    taskEXIT_CRITICAL();
}

static int16_t PulseToQ15(uint16_t pulse_us,
                          uint16_t minimum_us,
                          uint16_t center_us,
                          uint16_t maximum_us)
{
    int32_t value;

    if (pulse_us <= minimum_us)
    {
        return (int16_t)-32767;
    }
    if (pulse_us >= maximum_us)
    {
        return (int16_t)32767;
    }
    if (pulse_us == center_us)
    {
        return 0;
    }

    if (pulse_us < center_us)
    {
        value = -((int32_t)(center_us - pulse_us) * 32767L) /
                 (int32_t)(center_us - minimum_us);
    }
    else
    {
        value = ((int32_t)(pulse_us - center_us) * 32767L) /
                (int32_t)(maximum_us - center_us);
    }

    if (value < -32767L)
    {
        value = -32767L;
    }
    else if (value > 32767L)
    {
        value = 32767L;
    }

    return (int16_t)value;
}
