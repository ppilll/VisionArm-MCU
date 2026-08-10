#include "app_init.h"

#include "control_mailbox.h"
#include "gimbal_stub.h"
#include "protocol_dispatcher.h"
#include "protocol_engine.h"
#include "protocol_rx_task.h"
#include "protocol_state.h"
#include "protocol_tx_task.h"
#include "protocol_watchdog.h"
#include "uart_rx_ring.h"

bool VisionArmApp_Init(void)
{
    UartRxRing_Init();
    ProtocolState_Init();
    ProtocolWatchdog_Init();
    ControlMailbox_Init();
    ProtocolDispatcher_Init();
    ProtocolEngine_Init();

    /* TX must exist before RX can dispatch request-triggered responses. */
    if (!ProtocolTxTask_Create())
    {
        return false;
    }

    if (!ProtocolRxTask_Create())
    {
        return false;
    }

    if (!GimbalStubTask_Create())
    {
        return false;
    }

    return true;
}
