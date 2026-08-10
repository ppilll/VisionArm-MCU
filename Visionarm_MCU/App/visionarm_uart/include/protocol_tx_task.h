#ifndef VISIONARM_PROTOCOL_TX_TASK_H
#define VISIONARM_PROTOCOL_TX_TASK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

bool ProtocolTxTask_Create(void);
bool ProtocolTxTask_EnqueueResponse(uint8_t message_type,
                                    const uint8_t *payload,
                                    size_t payload_size,
                                    TickType_t ticks_to_wait);
void ProtocolTxTask_NotifyTxCompleteFromISR(BaseType_t *higher_priority_task_woken);

#endif /* VISIONARM_PROTOCOL_TX_TASK_H */
