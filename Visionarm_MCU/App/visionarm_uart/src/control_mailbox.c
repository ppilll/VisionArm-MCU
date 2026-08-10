#include "control_mailbox.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

static ControlCommand s_command;
static uint32_t s_overwrite_count;

void ControlMailbox_Init(void)
{
    (void)memset(&s_command, 0, sizeof(s_command));
    s_overwrite_count = 0U;
}

void ControlMailbox_Publish(uint32_t wire_sequence,
                            const va_uart_control_update_t *control,
                            bool valid)
{
    if (control == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();

    if (s_command.generation != 0U)
    {
        s_overwrite_count++;
    }

    s_command.generation++;
    if (s_command.generation == 0U)
    {
        s_command.generation = 1U;
    }

    s_command.valid = valid;
    s_command.wire_sequence = wire_sequence;
    s_command.control = *control;

    taskEXIT_CRITICAL();
}

void ControlMailbox_Invalidate(void)
{
    taskENTER_CRITICAL();
    s_command.valid = false;
    taskEXIT_CRITICAL();
}

bool ControlMailbox_Read(ControlCommand *command)
{
    bool available;

    if (command == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    *command = s_command;
    available = (s_command.generation != 0U);
    taskEXIT_CRITICAL();

    return available;
}

uint32_t ControlMailbox_GetOverwriteCount(void)
{
    uint32_t count;

    taskENTER_CRITICAL();
    count = s_overwrite_count;
    taskEXIT_CRITICAL();

    return count;
}
