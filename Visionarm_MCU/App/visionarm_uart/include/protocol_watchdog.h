#ifndef VISIONARM_PROTOCOL_WATCHDOG_H
#define VISIONARM_PROTOCOL_WATCHDOG_H

void ProtocolWatchdog_Init(void);
void ProtocolWatchdog_OnHelloAccepted(void);
void ProtocolWatchdog_RefreshLink(void);
void ProtocolWatchdog_RefreshControl(void);
void ProtocolWatchdog_DisarmControl(void);
void ProtocolWatchdog_DisarmAll(void);
void ProtocolWatchdog_Check(void);

#endif /* VISIONARM_PROTOCOL_WATCHDOG_H */
