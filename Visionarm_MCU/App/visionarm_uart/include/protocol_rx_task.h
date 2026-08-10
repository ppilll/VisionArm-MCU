#ifndef VISIONARM_PROTOCOL_RX_TASK_H
#define VISIONARM_PROTOCOL_RX_TASK_H

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

bool ProtocolRxTask_Create(void);
void ProtocolRxTask_NotifyFromISR(BaseType_t *higher_priority_task_woken);

#endif /* VISIONARM_PROTOCOL_RX_TASK_H */
