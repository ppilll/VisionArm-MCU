#include "actuator.h"

#include <stddef.h>

#include "stm32f1xx_hal.h"

#define PAN_GPIO_PORT                     GPIOA
#define PAN_GPIO_PIN                      GPIO_PIN_6
#define TILT_GPIO_PORT                    GPIOA
#define TILT_GPIO_PIN                     GPIO_PIN_7
#define PAN_TIMER_CHANNEL                 TIM_CHANNEL_1
#define TILT_TIMER_CHANNEL                TIM_CHANNEL_2

static TIM_HandleTypeDef s_tim3;
static bool s_pwm_initialized;
static bool s_pwm_enabled;
static ActuatorSnapshot s_snapshot;

static bool PwmInitSafe(void);
static bool PwmEnable(uint16_t pan_us, uint16_t tilt_us);
static bool PwmSetBoth(uint16_t pan_us, uint16_t tilt_us);
static bool PulseIsElectricalValid(uint16_t pulse_us);
static uint32_t GetTim3ClockHz(void);
static void ConfigurePinsSafeLow(void);
static void ConfigurePinsAlternateFunction(void);
static uint16_t ClampPulse(int32_t requested,
                           uint16_t minimum,
                           uint16_t maximum,
                           bool *clamped);

bool Actuator_InitSafe(void)
{
    s_snapshot = (ActuatorSnapshot){0};

    if (!PwmInitSafe())
    {
        Actuator_ForceSafeOutput();
        s_snapshot.fault_count++;
        return false;
    }

    s_snapshot.initialized = true;
    s_snapshot.enabled = false;
    s_snapshot.applied_pan_us = ACTUATOR_PAN_CENTER_US;
    s_snapshot.applied_tilt_us = ACTUATOR_TILT_CENTER_US;
    return true;
}

bool Actuator_Enable(void)
{
    if (!s_snapshot.initialized)
    {
        return false;
    }

    if (!PwmEnable(ACTUATOR_PAN_CENTER_US,
                   ACTUATOR_TILT_CENTER_US))
    {
        Actuator_Disable();
        s_snapshot.fault_count++;
        return false;
    }

    s_snapshot.enabled = true;
    s_snapshot.applied_pan_us = ACTUATOR_PAN_CENTER_US;
    s_snapshot.applied_tilt_us = ACTUATOR_TILT_CENTER_US;
    s_snapshot.enable_count++;
    return true;
}

bool Actuator_Apply(const ActuatorCommand *command,
                    ActuatorApplyResult *result)
{
    ActuatorApplyResult local_result = {0};

    if (command != NULL)
    {
        local_result.requested_pan_us = command->pan_position_us;
        local_result.requested_tilt_us = command->tilt_position_us;
    }

    if ((command == NULL) ||
        !s_snapshot.initialized ||
        !s_snapshot.enabled)
    {
        if (result != NULL)
        {
            *result = local_result;
        }
        return false;
    }

    local_result.applied_pan_us =
        ClampPulse(command->pan_position_us,
                   ACTUATOR_PAN_SAFE_MIN_US,
                   ACTUATOR_PAN_SAFE_MAX_US,
                   &local_result.pan_saturated);

    local_result.applied_tilt_us =
        ClampPulse(command->tilt_position_us,
                   ACTUATOR_TILT_SAFE_MIN_US,
                   ACTUATOR_TILT_SAFE_MAX_US,
                   &local_result.tilt_saturated);

    if (!PwmSetBoth(local_result.applied_pan_us,
                    local_result.applied_tilt_us))
    {
        Actuator_Disable();
        s_snapshot.fault_count++;

        if (result != NULL)
        {
            *result = local_result;
        }
        return false;
    }

    s_snapshot.applied_pan_us = local_result.applied_pan_us;
    s_snapshot.applied_tilt_us = local_result.applied_tilt_us;
    s_snapshot.command_count++;

    if (local_result.pan_saturated)
    {
        s_snapshot.pan_saturation_count++;
    }

    if (local_result.tilt_saturated)
    {
        s_snapshot.tilt_saturation_count++;
    }

    if (result != NULL)
    {
        *result = local_result;
    }

    return true;
}

void Actuator_Disable(void)
{
    bool was_enabled = s_snapshot.enabled;

    Actuator_ForceSafeOutput();
    s_pwm_enabled = false;
    s_snapshot.enabled = false;

    if (was_enabled)
    {
        s_snapshot.disable_count++;
    }
}

void Actuator_ForceSafeOutput(void)
{
    uint32_t crl;

    /*
     * Emergency-safe path intentionally bypasses HAL and RTOS state.
     * Disable both TIM3 compare outputs and the counter, then force PA6/PA7
     * to GPIO push-pull LOW.
     */
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
    CLEAR_BIT(TIM3->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E);
    CLEAR_BIT(TIM3->CR1, TIM_CR1_CEN);

    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);
    GPIOA->BRR = (uint32_t)(PAN_GPIO_PIN | TILT_GPIO_PIN);

    /* STM32F1 CRL nibble: CNF=00, MODE=10 -> GP push-pull, 2 MHz. */
    crl = GPIOA->CRL;
    crl &= ~((0xFUL << 24U) | (0xFUL << 28U));
    crl |=  ((0x2UL << 24U) | (0x2UL << 28U));
    GPIOA->CRL = crl;

    GPIOA->BRR = (uint32_t)(PAN_GPIO_PIN | TILT_GPIO_PIN);
}

void Actuator_GetSnapshot(ActuatorSnapshot *snapshot)
{
    if (snapshot != NULL)
    {
        *snapshot = s_snapshot;
    }
}

static bool PwmInitSafe(void)
{
    TIM_OC_InitTypeDef oc = {0};
    uint32_t timer_clock_hz;
    uint32_t prescaler_divisor;

    Actuator_ForceSafeOutput();
    s_tim3 = (TIM_HandleTypeDef){0};
    s_pwm_initialized = false;
    s_pwm_enabled = false;

    timer_clock_hz = GetTim3ClockHz();
    if ((timer_clock_hz == 0U) ||
        ((timer_clock_hz % ACTUATOR_PWM_COUNTER_HZ) != 0U))
    {
        Actuator_ForceSafeOutput();
        return false;
    }

    prescaler_divisor = timer_clock_hz / ACTUATOR_PWM_COUNTER_HZ;
    if ((prescaler_divisor == 0U) || (prescaler_divisor > 65536U))
    {
        Actuator_ForceSafeOutput();
        return false;
    }

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* Default TIM3 mapping: CH1=PA6, CH2=PA7. */
    CLEAR_BIT(AFIO->MAPR, AFIO_MAPR_TIM3_REMAP);
    ConfigurePinsSafeLow();

    s_tim3.Instance = TIM3;
    s_tim3.Init.Prescaler = prescaler_divisor - 1U;
    s_tim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_tim3.Init.Period = ACTUATOR_PWM_PERIOD_US - 1U;
    s_tim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_tim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&s_tim3) != HAL_OK)
    {
        Actuator_ForceSafeOutput();
        return false;
    }

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = ACTUATOR_SERVO_CENTER_US;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&s_tim3, &oc, PAN_TIMER_CHANNEL) != HAL_OK)
    {
        Actuator_ForceSafeOutput();
        return false;
    }

    if (HAL_TIM_PWM_ConfigChannel(&s_tim3, &oc, TILT_TIMER_CHANNEL) != HAL_OK)
    {
        Actuator_ForceSafeOutput();
        return false;
    }

    __HAL_TIM_SET_COUNTER(&s_tim3, 0U);
    if (HAL_TIM_GenerateEvent(&s_tim3, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
    {
        Actuator_ForceSafeOutput();
        return false;
    }

    s_pwm_initialized = true;
    return true;
}

static bool PwmEnable(uint16_t pan_us, uint16_t tilt_us)
{
    if (!s_pwm_initialized ||
        !PulseIsElectricalValid(pan_us) ||
        !PulseIsElectricalValid(tilt_us))
    {
        Actuator_ForceSafeOutput();
        s_pwm_enabled = false;
        return false;
    }

    __HAL_TIM_DISABLE(&s_tim3);
    __HAL_TIM_SET_COMPARE(&s_tim3, PAN_TIMER_CHANNEL, pan_us);
    __HAL_TIM_SET_COMPARE(&s_tim3, TILT_TIMER_CHANNEL, tilt_us);
    __HAL_TIM_SET_COUNTER(&s_tim3, 0U);

    if (HAL_TIM_GenerateEvent(&s_tim3, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
    {
        Actuator_ForceSafeOutput();
        s_pwm_enabled = false;
        return false;
    }

    ConfigurePinsAlternateFunction();
    SET_BIT(TIM3->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E);
    SET_BIT(TIM3->CR1, TIM_CR1_CEN);
    s_pwm_enabled = true;
    return true;
}

static bool PwmSetBoth(uint16_t pan_us, uint16_t tilt_us)
{
    if (!s_pwm_initialized ||
        !s_pwm_enabled ||
        !PulseIsElectricalValid(pan_us) ||
        !PulseIsElectricalValid(tilt_us))
    {
        return false;
    }

    /* Write both preload CCRs without allowing a transfer between writes. */
    SET_BIT(TIM3->CR1, TIM_CR1_UDIS);
    __HAL_TIM_SET_COMPARE(&s_tim3, PAN_TIMER_CHANNEL, pan_us);
    __HAL_TIM_SET_COMPARE(&s_tim3, TILT_TIMER_CHANNEL, tilt_us);
    CLEAR_BIT(TIM3->CR1, TIM_CR1_UDIS);
    return true;
}

static bool PulseIsElectricalValid(uint16_t pulse_us)
{
    return (pulse_us >= ACTUATOR_SERVO_ELECTRICAL_MIN_US) &&
           (pulse_us <= ACTUATOR_SERVO_ELECTRICAL_MAX_US);
}

static uint32_t GetTim3ClockHz(void)
{
    uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();
    uint32_t ppre1 = READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1);

    return (ppre1 == RCC_CFGR_PPRE1_DIV1) ? pclk1_hz : (pclk1_hz * 2U);
}

static void ConfigurePinsSafeLow(void)
{
    GPIO_InitTypeDef gpio = {0};

    HAL_GPIO_WritePin(GPIOA, PAN_GPIO_PIN | TILT_GPIO_PIN, GPIO_PIN_RESET);

    gpio.Pin = PAN_GPIO_PIN | TILT_GPIO_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_GPIO_WritePin(GPIOA, PAN_GPIO_PIN | TILT_GPIO_PIN, GPIO_PIN_RESET);
}

static void ConfigurePinsAlternateFunction(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = PAN_GPIO_PIN | TILT_GPIO_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static uint16_t ClampPulse(int32_t requested,
                           uint16_t minimum,
                           uint16_t maximum,
                           bool *clamped)
{
    if (clamped != NULL)
    {
        *clamped = false;
    }

    if (requested < (int32_t)minimum)
    {
        if (clamped != NULL)
        {
            *clamped = true;
        }
        return minimum;
    }

    if (requested > (int32_t)maximum)
    {
        if (clamped != NULL)
        {
            *clamped = true;
        }
        return maximum;
    }

    return (uint16_t)requested;
}
