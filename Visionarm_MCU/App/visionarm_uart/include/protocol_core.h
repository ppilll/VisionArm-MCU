#ifndef VISIONARM_PROTOCOL_CORE_H
#define VISIONARM_PROTOCOL_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "visionarm_uart_c/messages.h"
#include "visionarm_uart_c/protocol.h"

typedef enum
{
    PROTOCOL_LINK_WAIT_HELLO = 0,
    PROTOCOL_LINK_READY = 1,
    PROTOCOL_LINK_LOST = 2
} ProtocolLinkState;

typedef struct
{
    ProtocolLinkState link_state;
    bool remote_stop_latched;
    bool control_valid;
    uint32_t mcu_boot_id;
    uint32_t last_rx_wire_sequence;
    uint32_t last_control_wire_sequence;
    uint32_t sequence_gap_count;
} ProtocolStateSnapshot;

typedef struct
{
    bool valid;
    uint32_t generation;
    uint32_t wire_sequence;
    va_uart_control_update_t control;
} ControlCommand;

void ProtocolCore_Init(void);
void ProtocolCore_OnFrame(const va_uart_frame_t *frame);
void ProtocolCore_Tick(void);

void ProtocolCore_GetState(ProtocolStateSnapshot *snapshot);
bool ProtocolCore_ReadControl(ControlCommand *command);
uint32_t ProtocolCore_GetMcuBootId(void);
uint32_t ProtocolCore_GetMailboxOverwriteCount(void);

#endif /* VISIONARM_PROTOCOL_CORE_H */
