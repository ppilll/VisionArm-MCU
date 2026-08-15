#include "stm32f1xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "actuator_driver.h"
#include "app_init.h"
#include "gimbal_config.h"
#include "rs485_uart.h"
#include "timer_pwm.h"

#if GIMBAL_STEP_G_SELF_TEST_ENABLED
#include "actuator_driver_test.h"
#endif

static void SystemClock_Config(void);
static void Error_Handler(void);

int main(void)
{
    HAL_Init();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    SystemClock_Config();

    /*
     * Step-G product boundary:
     * application code initializes ActuatorDriver, never TIM3 directly.
     * InitSafe leaves both physical servo outputs disabled and LOW.
     */
    if (!ActuatorDriver_InitSafe())
    {
        Error_Handler();
    }

    if (!Rs485Uart_Init() || !VisionArmApp_Init())
    {
        Error_Handler();
    }

#if GIMBAL_STEP_G_SELF_TEST_ENABLED
    /*
     * One-shot Step-G physical validation only.
     * The final Step-G baseline sets the switch to 0 so no autonomous
     * actuator motion occurs before Step H installs the SafetyGate owner.
     */
    if (!ActuatorDriverTest_Create())
    {
        Error_Handler();
    }
#endif

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

    /* Fatal/emergency path deliberately bypasses higher-level state. */
    TimerPwm_ForceSafeOutput();

    RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
    GPIOD->BRR = (1UL << 7U);

    __DSB();
    __ISB();

    for (;;)
    {
        __NOP();
    }
}
