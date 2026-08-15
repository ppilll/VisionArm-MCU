#ifndef VISIONARM_PROTOCOL_TRANSPORT_H
#define VISIONARM_PROTOCOL_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct
{
    uint32_t valid_frame_count;
    uint32_t crc_error_count;
    uint32_t length_error_count;
    uint32_t version_error_count;
    uint32_t unknown_type_count;
} ProtocolRxStats;

/* Initialize USART2/RS-485, RX ring, and frame parser. */
bool ProtocolTransport_Init(void);

/* Create the statically allocated ProtocolTx and ProtocolRx tasks. */
bool ProtocolTransport_CreateTasks(void);

/* Queue one MCU response; message-type priority is handled internally. */
bool ProtocolTransport_QueueResponse(uint8_t message_type,
                                     const uint8_t *payload,
                                     size_t payload_size);

void ProtocolTransport_GetRxStats(ProtocolRxStats *stats);
uint32_t ProtocolTransport_GetRxOverflowCount(void);

/* Called only from the USART2 vector in Core/Src/interrupts.c. */
void ProtocolTransport_UartIrqHandler(void);

#endif /* VISIONARM_PROTOCOL_TRANSPORT_H */
