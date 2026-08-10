#ifndef VISIONARM_CONTROL_MAILBOX_H
#define VISIONARM_CONTROL_MAILBOX_H

#include <stdbool.h>
#include <stdint.h>

#include "visionarm_uart_c/messages.h"

typedef struct
{
    bool valid;
    uint32_t generation;
    uint32_t wire_sequence;
    va_uart_control_update_t control;
} ControlCommand;

void ControlMailbox_Init(void);
void ControlMailbox_Publish(uint32_t wire_sequence,
                            const va_uart_control_update_t *control,
                            bool valid);
void ControlMailbox_Invalidate(void);
bool ControlMailbox_Read(ControlCommand *command);
uint32_t ControlMailbox_GetOverwriteCount(void);

#endif /* VISIONARM_CONTROL_MAILBOX_H */
