#include "stm32f1xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_init.h"
#include "pwm_bench_test.h"
#include "rs485_uart.h"
#include "timer_pwm.h"

static void SystemClock_Config(void);
static void Error_Handler(void);

int main(void)
{
    HAL_Init();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    SystemClock_Config();

    /*
     * Step A/B/C boundary:
     * configure TIM3 and PA6/PA7, but keep both PWM outputs disabled and low.
     */
    if (!TimerPwm_InitSafe())
    {
        Error_Handler();
    }

    if (!Rs485Uart_Init() || !VisionArmApp_Init())
    {
        Error_Handler();
    }

    /*
     * Step-C bench-only waveform generator.
     * Do not connect the servo signal wires while this task is enabled.
     */
    if (!PwmBenchTest_Create())
    {
        Error_Handler();
    }

    vTaskStartScheduler();
    Error_Handler();
    return 0;
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.HSIState = RCC_HSI_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
        Error_Handler();
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 |
                    RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }

    SystemCoreClockUpdate();
}

static void Error_Handler(void)
{
    __disable_irq();

    /* Final actuator-safe software action. */
    TimerPwm_ForceSafeOutput();

    /* Force RS-485 DE low without depending on UART/HAL initialization state. */
    RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
    GPIOD->BRR = (1UL << 7U);

    __DSB();
    __ISB();

    for (;;)
    {
        __NOP();
    }
}
