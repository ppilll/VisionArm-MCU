#include "app_fatal.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"

volatile AppFatalState g_app_fatal_state;

static void FatalStop(void);

void AppFatal_Assert(const char *file, int line)
{
    g_app_fatal_state.reason = APP_FATAL_ASSERT;
    g_app_fatal_state.assert_file = file;
    g_app_fatal_state.assert_line = line;
    FatalStop();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task_name;
    g_app_fatal_state.reason = APP_FATAL_STACK_OVERFLOW;
    g_app_fatal_state.task_handle = (uint32_t)(uintptr_t)task;
    FatalStop();
}

static void FatalStop(void)
{
    __disable_irq();

    /* Force RS-485 DE low without relying on RTOS/HAL state. */
    RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
    GPIOD->BRR = (1UL << 7U);

    __DSB();
    __ISB();

    for (;;)
    {
        __NOP();
    }
}
