#include "protocol_state.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"

#define STM32F1_UID_WORD0_ADDRESS ((uintptr_t)0x1FFFF7E8UL)
#define STM32F1_UID_WORD1_ADDRESS ((uintptr_t)0x1FFFF7ECUL)
#define STM32F1_UID_WORD2_ADDRESS ((uintptr_t)0x1FFFF7F0UL)

typedef struct
{
    ProtocolLinkState link_state;
    bool remote_stop_latched;
    bool control_valid;
    bool have_peer;
    bool sequence_valid;
    uint32_t mcu_boot_id;
    uint32_t peer_boot_id;
    uint32_t last_rx_wire_sequence;
    uint32_t last_control_wire_sequence;
    uint32_t sequence_gap_count;
} ProtocolRuntimeState;

static ProtocolRuntimeState s_state;

static uint32_t GenerateBootId(void);
static uint32_t Mix32(uint32_t value);

void ProtocolState_Init(void)
{
    (void)memset(&s_state, 0, sizeof(s_state));
    s_state.link_state = PROTOCOL_LINK_WAIT_HELLO;
    s_state.mcu_boot_id = GenerateBootId();
}

void ProtocolState_AcceptHello(uint32_t peer_boot_id,
                               uint32_t hello_wire_sequence)
{
    s_state.have_peer = true;
    s_state.peer_boot_id = peer_boot_id;
    s_state.sequence_valid = true;
    s_state.last_rx_wire_sequence = hello_wire_sequence;
    s_state.control_valid = false;
    s_state.link_state = PROTOCOL_LINK_READY;
}

void ProtocolState_RejectHello(void)
{
    s_state.link_state = PROTOCOL_LINK_WAIT_HELLO;
    s_state.have_peer = false;
    s_state.sequence_valid = false;
    s_state.control_valid = false;
}

void ProtocolState_PeerMismatch(void)
{
    ProtocolState_RejectHello();
}

void ProtocolState_LinkTimeout(void)
{
    s_state.link_state = PROTOCOL_LINK_LOST;
    s_state.have_peer = false;
    s_state.sequence_valid = false;
    s_state.control_valid = false;
}

bool ProtocolState_IsReadyForPeer(uint32_t peer_boot_id)
{
    return (s_state.link_state == PROTOCOL_LINK_READY) &&
           s_state.have_peer &&
           (s_state.peer_boot_id == peer_boot_id);
}

ProtocolSequenceResult ProtocolState_ObserveSequence(uint32_t wire_sequence)
{
    ProtocolSequenceResult result = {PROTOCOL_SEQUENCE_NEW, 0U};
    uint32_t difference;

    if (!s_state.sequence_valid)
    {
        s_state.sequence_valid = true;
        s_state.last_rx_wire_sequence = wire_sequence;
        return result;
    }

    difference = (uint32_t)(wire_sequence - s_state.last_rx_wire_sequence);

    if (difference == 0U)
    {
        result.decision = PROTOCOL_SEQUENCE_DUPLICATE;
        return result;
    }

    if (difference < 0x80000000UL)
    {
        result.gap_count = difference - 1U;
        s_state.sequence_gap_count += result.gap_count;
        s_state.last_rx_wire_sequence = wire_sequence;
        return result;
    }

    result.decision = PROTOCOL_SEQUENCE_OLD;
    return result;
}

void ProtocolState_SetControlValid(uint32_t wire_sequence)
{
    s_state.control_valid = true;
    s_state.last_control_wire_sequence = wire_sequence;
}

void ProtocolState_InvalidateControl(void)
{
    s_state.control_valid = false;
}

void ProtocolState_SetRemoteStop(bool latched)
{
    s_state.remote_stop_latched = latched;
    if (latched)
    {
        s_state.control_valid = false;
    }
}

uint32_t ProtocolState_GetMcuBootId(void)
{
    return s_state.mcu_boot_id;
}

void ProtocolState_GetSnapshot(ProtocolStateSnapshot *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();

    snapshot->link_state = s_state.link_state;
    snapshot->remote_stop_latched = s_state.remote_stop_latched;
    snapshot->control_valid = s_state.control_valid;
    snapshot->mcu_boot_id = s_state.mcu_boot_id;
    snapshot->last_rx_wire_sequence = s_state.last_rx_wire_sequence;
    snapshot->last_control_wire_sequence = s_state.last_control_wire_sequence;
    snapshot->sequence_gap_count = s_state.sequence_gap_count;

    taskEXIT_CRITICAL();
}

static uint32_t GenerateBootId(void)
{
    const volatile uint32_t *uid0 =
        (const volatile uint32_t *)STM32F1_UID_WORD0_ADDRESS;
    const volatile uint32_t *uid1 =
        (const volatile uint32_t *)STM32F1_UID_WORD1_ADDRESS;
    const volatile uint32_t *uid2 =
        (const volatile uint32_t *)STM32F1_UID_WORD2_ADDRESS;
    uint32_t seed;

    seed = *uid0;
    seed ^= (*uid1 << 7U) | (*uid1 >> 25U);
    seed ^= (*uid2 << 17U) | (*uid2 >> 15U);
    seed ^= HAL_GetTick();
    seed ^= DWT->CYCCNT;
    seed = Mix32(seed);

    return (seed != 0U) ? seed : 0x5641524DUL; /* "VARM" */
}

static uint32_t Mix32(uint32_t value)
{
    value ^= value >> 16U;
    value *= 0x7FEB352DUL;
    value ^= value >> 15U;
    value *= 0x846CA68BUL;
    value ^= value >> 16U;
    return value;
}
