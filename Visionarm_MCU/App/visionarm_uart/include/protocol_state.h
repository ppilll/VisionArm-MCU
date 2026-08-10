#ifndef VISIONARM_PROTOCOL_STATE_H
#define VISIONARM_PROTOCOL_STATE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    PROTOCOL_LINK_WAIT_HELLO = 0,
    PROTOCOL_LINK_READY = 1,
    PROTOCOL_LINK_LOST = 2
} ProtocolLinkState;

typedef enum
{
    PROTOCOL_SEQUENCE_NEW = 0,
    PROTOCOL_SEQUENCE_DUPLICATE,
    PROTOCOL_SEQUENCE_OLD
} ProtocolSequenceDecision;

typedef struct
{
    ProtocolSequenceDecision decision;
    uint32_t gap_count;
} ProtocolSequenceResult;

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

void ProtocolState_Init(void);
void ProtocolState_AcceptHello(uint32_t peer_boot_id, uint32_t hello_wire_sequence);
void ProtocolState_RejectHello(void);
void ProtocolState_PeerMismatch(void);
void ProtocolState_LinkTimeout(void);

bool ProtocolState_IsReadyForPeer(uint32_t peer_boot_id);
ProtocolSequenceResult ProtocolState_ObserveSequence(uint32_t wire_sequence);

void ProtocolState_SetControlValid(uint32_t wire_sequence);
void ProtocolState_InvalidateControl(void);
void ProtocolState_SetRemoteStop(bool latched);

uint32_t ProtocolState_GetMcuBootId(void);
void ProtocolState_GetSnapshot(ProtocolStateSnapshot *snapshot);

#endif /* VISIONARM_PROTOCOL_STATE_H */
