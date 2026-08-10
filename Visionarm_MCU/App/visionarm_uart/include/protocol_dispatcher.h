#ifndef VISIONARM_PROTOCOL_DISPATCHER_H
#define VISIONARM_PROTOCOL_DISPATCHER_H

#include "protocol_message.h"

void ProtocolDispatcher_Init(void);
void ProtocolDispatcher_OnMessage(const ProtocolMessage *message);

#endif /* VISIONARM_PROTOCOL_DISPATCHER_H */
