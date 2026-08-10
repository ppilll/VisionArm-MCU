#include "FreeRTOS.h"
#include "task.h"

void vApplicationGetIdleTaskMemory(StaticTask_t **idle_tcb,
                                   StackType_t **idle_stack,
                                   uint32_t *idle_stack_size)
{
    static StaticTask_t s_idle_tcb;
    static StackType_t s_idle_stack[configMINIMAL_STACK_SIZE];

    *idle_tcb = &s_idle_tcb;
    *idle_stack = s_idle_stack;
    *idle_stack_size = configMINIMAL_STACK_SIZE;
}
