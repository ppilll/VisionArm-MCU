#include "stm32f1xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_init.h"
#include "rs485_uart.h"
#include "tilt_calibration_test.h"
#include "timer_pwm.h"

static void SystemClock_Config(void);
static void Error_Handler(void);

int main(void)
{
    HAL_Init();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    SystemClock_Config();

    if (!TimerPwm_InitSafe())
    {
        Error_Handler();
    }

    if (!Rs485Uart_Init() || !VisionArmApp_Init())
    {
        Error_Handler();
    }

    /*
     * V6 Step F: Tilt single-axis calibration.
     *
     * - The Step-C-proven dual-channel TimerPwm driver is unchanged.
     * - PA7 / TIM3_CH2 is the Tilt calibration command.
     * - PA6 / TIM3_CH1 stays at 1500 us only as an analyzer reference while
     *   PWM is active.
     * - The physical Pan servo SIGNAL must be disconnected during Step F.
     * - PwmBenchTest_Create() and ServoCalibrationTest_Create() are NOT called.
     */
    if (!TiltCalibrationTest_Create())
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
