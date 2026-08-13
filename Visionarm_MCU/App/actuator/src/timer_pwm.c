#include "timer_pwm.h"

#include "stm32f1xx_hal.h"

#define TIMER_PWM_PAN_GPIO_PORT          GPIOA
#define TIMER_PWM_PAN_GPIO_PIN           GPIO_PIN_6
#define TIMER_PWM_TILT_GPIO_PORT         GPIOA
#define TIMER_PWM_TILT_GPIO_PIN          GPIO_PIN_7

#define TIMER_PWM_PAN_CHANNEL            TIM_CHANNEL_1
#define TIMER_PWM_TILT_CHANNEL           TIM_CHANNEL_2

static TIM_HandleTypeDef s_tim3;
static TimerPwmSnapshot s_snapshot;

static uint32_t TimerPwm_GetTim3ClockHz(void);
static bool TimerPwm_IsPulseValid(uint16_t pulse_us);
static void TimerPwm_ConfigurePinsSafeLow(void);
static void TimerPwm_ConfigurePinsAlternateFunction(void);
static void TimerPwm_RecordDisabled(void);

bool TimerPwm_InitSafe(void)
{
    TIM_OC_InitTypeDef oc = {0};
    uint32_t timer_clock_hz;
    uint32_t prescaler_divisor;

    TimerPwm_ForceSafeOutput();
    s_tim3 = (TIM_HandleTypeDef){0};
    s_snapshot = (TimerPwmSnapshot){0};

    timer_clock_hz = TimerPwm_GetTim3ClockHz();
    if ((timer_clock_hz == 0U) ||
        ((timer_clock_hz % TIMER_PWM_COUNTER_HZ) != 0U))
    {
        TimerPwm_ForceSafeOutput();
        return false;
    }

    prescaler_divisor = timer_clock_hz / TIMER_PWM_COUNTER_HZ;
    if ((prescaler_divisor == 0U) || (prescaler_divisor > 65536U))
    {
        TimerPwm_ForceSafeOutput();
        return false;
    }

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* Explicitly select the default TIM3 mapping: CH1=PA6, CH2=PA7. */
    CLEAR_BIT(AFIO->MAPR, AFIO_MAPR_TIM3_REMAP);

    TimerPwm_ConfigurePinsSafeLow();

    s_tim3.Instance = TIM3;
    s_tim3.Init.Prescaler = prescaler_divisor - 1U;
    s_tim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_tim3.Init.Period = TIMER_PWM_PERIOD_US - 1U;
    s_tim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_tim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&s_tim3) != HAL_OK)
    {
        TimerPwm_ForceSafeOutput();
        return false;
    }

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = TIMER_PWM_SERVO_CENTER_US;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&s_tim3,
                                  &oc,
                                  TIMER_PWM_PAN_CHANNEL) != HAL_OK)
    {
        TimerPwm_ForceSafeOutput();
        return false;
    }

    if (HAL_TIM_PWM_ConfigChannel(&s_tim3,
                                  &oc,
                                  TIMER_PWM_TILT_CHANNEL) != HAL_OK)
    {
        TimerPwm_ForceSafeOutput();
        return false;
    }

    __HAL_TIM_SET_COUNTER(&s_tim3, 0U);
    if (HAL_TIM_GenerateEvent(&s_tim3, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
    {
        TimerPwm_ForceSafeOutput();
        return false;
    }

    s_snapshot.pclk1_hz = HAL_RCC_GetPCLK1Freq();
    s_snapshot.tim3_clock_hz = timer_clock_hz;
    s_snapshot.prescaler = (uint16_t)s_tim3.Init.Prescaler;
    s_snapshot.auto_reload = (uint16_t)s_tim3.Init.Period;
    s_snapshot.pan_pulse_us = TIMER_PWM_SERVO_CENTER_US;
    s_snapshot.tilt_pulse_us = TIMER_PWM_SERVO_CENTER_US;
    s_snapshot.initialized = true;
    s_snapshot.enabled = false;

    return true;
}

bool TimerPwm_Enable(uint16_t pan_pulse_us, uint16_t tilt_pulse_us)
{
    if (!s_snapshot.initialized ||
        !TimerPwm_IsPulseValid(pan_pulse_us) ||
        !TimerPwm_IsPulseValid(tilt_pulse_us))
    {
        TimerPwm_Disable();
        return false;
    }

    /* Start from a known phase with known preload values. */
    __HAL_TIM_DISABLE(&s_tim3);
    __HAL_TIM_SET_COMPARE(&s_tim3, TIMER_PWM_PAN_CHANNEL, pan_pulse_us);
    __HAL_TIM_SET_COMPARE(&s_tim3, TIMER_PWM_TILT_CHANNEL, tilt_pulse_us);
    __HAL_TIM_SET_COUNTER(&s_tim3, 0U);
    if (HAL_TIM_GenerateEvent(&s_tim3, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
    {
        TimerPwm_Disable();
        return false;
    }

    TimerPwm_ConfigurePinsAlternateFunction();

    /*
     * Enable both compare outputs before starting the shared counter so CH1 and
     * CH2 begin from the same timer phase. HAL has already validated/configured
     * the timer/channel registers above.
     */
    SET_BIT(TIM3->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E);
    SET_BIT(TIM3->CR1, TIM_CR1_CEN);

    s_snapshot.pan_pulse_us = pan_pulse_us;
    s_snapshot.tilt_pulse_us = tilt_pulse_us;
    s_snapshot.enabled = true;
    return true;
}

bool TimerPwm_SetBothPulseUs(uint16_t pan_pulse_us, uint16_t tilt_pulse_us)
{
    if (!s_snapshot.initialized ||
        !s_snapshot.enabled ||
        !TimerPwm_IsPulseValid(pan_pulse_us) ||
        !TimerPwm_IsPulseValid(tilt_pulse_us))
    {
        return false;
    }

    /*
     * Block update-event register transfers while both preload CCRs are being
     * written. The next normal timer update transfers both values together.
     */
    SET_BIT(TIM3->CR1, TIM_CR1_UDIS);
    __HAL_TIM_SET_COMPARE(&s_tim3, TIMER_PWM_PAN_CHANNEL, pan_pulse_us);
    __HAL_TIM_SET_COMPARE(&s_tim3, TIMER_PWM_TILT_CHANNEL, tilt_pulse_us);
    CLEAR_BIT(TIM3->CR1, TIM_CR1_UDIS);

    s_snapshot.pan_pulse_us = pan_pulse_us;
    s_snapshot.tilt_pulse_us = tilt_pulse_us;
    return true;
}

bool TimerPwm_SetPanPulseUs(uint16_t pulse_us)
{
    if (!s_snapshot.initialized ||
        !s_snapshot.enabled ||
        !TimerPwm_IsPulseValid(pulse_us))
    {
        return false;
    }

    __HAL_TIM_SET_COMPARE(&s_tim3, TIMER_PWM_PAN_CHANNEL, pulse_us);
    s_snapshot.pan_pulse_us = pulse_us;
    return true;
}

bool TimerPwm_SetTiltPulseUs(uint16_t pulse_us)
{
    if (!s_snapshot.initialized ||
        !s_snapshot.enabled ||
        !TimerPwm_IsPulseValid(pulse_us))
    {
        return false;
    }

    __HAL_TIM_SET_COMPARE(&s_tim3, TIMER_PWM_TILT_CHANNEL, pulse_us);
    s_snapshot.tilt_pulse_us = pulse_us;
    return true;
}

void TimerPwm_Disable(void)
{
    TimerPwm_ForceSafeOutput();
    TimerPwm_RecordDisabled();
}

void TimerPwm_ForceSafeOutput(void)
{
    uint32_t crl;

    /*
     * Make this path independent of HAL/task state. GPIOA reset state is not
     * considered a guaranteed actuator-safe state, so force PA6/PA7 low.
     */
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
    CLEAR_BIT(TIM3->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E);
    CLEAR_BIT(TIM3->CR1, TIM_CR1_CEN);

    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);
    GPIOA->BRR = (uint32_t)(GPIO_PIN_6 | GPIO_PIN_7);

    /* STM32F1 CRL nibble: CNF=00 (GP push-pull), MODE=10 (2 MHz). */
    crl = GPIOA->CRL;
    crl &= ~((0xFUL << 24U) | (0xFUL << 28U));
    crl |=  ((0x2UL << 24U) | (0x2UL << 28U));
    GPIOA->CRL = crl;

    GPIOA->BRR = (uint32_t)(GPIO_PIN_6 | GPIO_PIN_7);
}

void TimerPwm_GetSnapshot(TimerPwmSnapshot *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    *snapshot = s_snapshot;
}

static uint32_t TimerPwm_GetTim3ClockHz(void)
{
    uint32_t pclk1_hz;
    uint32_t ppre1;

    pclk1_hz = HAL_RCC_GetPCLK1Freq();
    ppre1 = READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1);

    if (ppre1 == RCC_CFGR_PPRE1_DIV1)
    {
        return pclk1_hz;
    }

    return pclk1_hz * 2U;
}

static bool TimerPwm_IsPulseValid(uint16_t pulse_us)
{
    return (pulse_us >= TIMER_PWM_SERVO_MIN_US) &&
           (pulse_us <= TIMER_PWM_SERVO_MAX_US);
}

static void TimerPwm_ConfigurePinsSafeLow(void)
{
    GPIO_InitTypeDef gpio = {0};

    HAL_GPIO_WritePin(GPIOA,
                      TIMER_PWM_PAN_GPIO_PIN | TIMER_PWM_TILT_GPIO_PIN,
                      GPIO_PIN_RESET);

    gpio.Pin = TIMER_PWM_PAN_GPIO_PIN | TIMER_PWM_TILT_GPIO_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_GPIO_WritePin(GPIOA,
                      TIMER_PWM_PAN_GPIO_PIN | TIMER_PWM_TILT_GPIO_PIN,
                      GPIO_PIN_RESET);
}

static void TimerPwm_ConfigurePinsAlternateFunction(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = TIMER_PWM_PAN_GPIO_PIN | TIMER_PWM_TILT_GPIO_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void TimerPwm_RecordDisabled(void)
{
    s_snapshot.enabled = false;
}
