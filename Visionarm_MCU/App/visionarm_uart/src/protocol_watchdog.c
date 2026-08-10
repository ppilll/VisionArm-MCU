#include "protocol_watchdog.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "control_mailbox.h"
#include "protocol_state.h"

static bool s_link_armed;
static bool s_control_armed;
static TickType_t s_last_link_tick;
static TickType_t s_last_control_tick;

void ProtocolWatchdog_Init(void)
{
    s_link_armed = false;
    s_control_armed = false;
    s_last_link_tick = 0;
    s_last_control_tick = 0;
}

void ProtocolWatchdog_OnHelloAccepted(void)
{
    s_link_armed = true;
    s_last_link_tick = xTaskGetTickCount();
    s_control_armed = false;
}

void ProtocolWatchdog_RefreshLink(void)
{
    s_link_armed = true;
    s_last_link_tick = xTaskGetTickCount();
}

void ProtocolWatchdog_RefreshControl(void)
{
    s_control_armed = true;
    s_last_control_tick = xTaskGetTickCount();
}

void ProtocolWatchdog_DisarmControl(void)
{
    s_control_armed = false;
}

void ProtocolWatchdog_DisarmAll(void)
{
    s_link_armed = false;
    s_control_armed = false;
}

void ProtocolWatchdog_Check(void)
{
    TickType_t now = xTaskGetTickCount();

    if (s_link_armed &&
        ((TickType_t)(now - s_last_link_tick) >=
         pdMS_TO_TICKS(APP_LINK_WATCHDOG_TIMEOUT_MS)))
    {
        s_link_armed = false;
        s_control_armed = false;
        ProtocolState_LinkTimeout();
        ControlMailbox_Invalidate();
        return;
    }

    if (s_control_armed &&
        ((TickType_t)(now - s_last_control_tick) >=
         pdMS_TO_TICKS(APP_CONTROL_WATCHDOG_TIMEOUT_MS)))
    {
        s_control_armed = false;
        ProtocolState_InvalidateControl();
        ControlMailbox_Invalidate();
    }
}
