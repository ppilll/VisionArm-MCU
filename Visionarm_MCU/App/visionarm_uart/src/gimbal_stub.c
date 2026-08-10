#include "gimbal_stub.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "control_mailbox.h"
#include "protocol_state.h"

static GimbalStubSnapshot s_output;
static TaskHandle_t s_gimbal_task;
static StaticTask_t s_gimbal_task_tcb;
static StackType_t s_gimbal_task_stack[APP_GIMBAL_TASK_STACK_WORDS];

static void GimbalTask(void *argument);
static void SetOutput(int16_t pan_q15, int16_t tilt_q15, bool active);

bool GimbalStubTask_Create(void)
{
    if (s_gimbal_task != NULL)
    {
        return false;
    }

    (void)memset(&s_output, 0, sizeof(s_output));

    s_gimbal_task = xTaskCreateStatic(GimbalTask,
                                      "GimbalStub",
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
    uint32_t last_generation = 0U;
    ControlCommand command;
    ProtocolStateSnapshot state;
    bool have_command;
    bool apply;

    (void)argument;
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        (void)xTaskDelayUntil(&last_wake,
                              pdMS_TO_TICKS(APP_GIMBAL_TASK_PERIOD_MS));

        ProtocolState_GetSnapshot(&state);
        have_command = ControlMailbox_Read(&command);

        if ((state.link_state != PROTOCOL_LINK_READY) ||
            state.remote_stop_latched ||
            !state.control_valid)
        {
            SetOutput(0, 0, false);
        }

        if (have_command && (command.generation != last_generation))
        {
            last_generation = command.generation;
            apply = command.valid &&
                    (state.link_state == PROTOCOL_LINK_READY) &&
                    !state.remote_stop_latched &&
                    state.control_valid;

            if (apply)
            {
                SetOutput(command.control.error_x_q15,
                          command.control.error_y_q15,
                          true);
            }
            else
            {
                SetOutput(0, 0, false);
            }
        }
    }
}

static void SetOutput(int16_t pan_q15, int16_t tilt_q15, bool active)
{
    taskENTER_CRITICAL();
    s_output.pan_q15 = pan_q15;
    s_output.tilt_q15 = tilt_q15;
    s_output.active = active;
    taskEXIT_CRITICAL();
}
