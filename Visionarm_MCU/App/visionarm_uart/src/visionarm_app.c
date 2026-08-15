#include "visionarm_app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"

#include "actuator.h"
#include "gimbal_task.h"
#include "protocol_core.h"
#include "protocol_transport.h"

static void FatalStop(void);

bool VisionArmApp_Init(void)
{
    /* Physical outputs are made safe before communication/tasks start. */
    if (!Actuator_InitSafe())
    {
        return false;
    }

    /* Preserve V5 startup order: UART/DWT is ready before boot-id creation. */
    if (!ProtocolTransport_Init())
    {
        return false;
    }

    ProtocolCore_Init();

    /* TX must exist before RX can dispatch request-triggered responses. */
    if (!ProtocolTransport_CreateTasks())
    {
        return false;
    }

    if (!GimbalTask_Create())
    {
        return false;
    }

    return true;
}

void VisionArmApp_EmergencyStop(void)
{
    /* Actuator safe primitive does not depend on scheduler/HAL state. */
    Actuator_ForceSafeOutput();

    /* RS-485 DE low: receive direction, also safe before UART init. */
    RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
    GPIOD->BRR = (1UL << 7U);
}

void AppFatal_Assert(const char *file, int line)
{
    (void)file;
    (void)line;
    FatalStop();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;
    FatalStop();
}

static void FatalStop(void)
{
    __disable_irq();
    VisionArmApp_EmergencyStop();
    __DSB();
    __ISB();

    for (;;)
    {
        __NOP();
    }
}
