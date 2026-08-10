#include "rs485_uart.h"
#include "stm32f1xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

extern void xPortSysTickHandler(void);

void SysTick_Handler(void)
{
    HAL_IncTick();

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xPortSysTickHandler();
    }
}

void USART2_IRQHandler(void)
{
    Rs485Uart_IrqHandler();
}
