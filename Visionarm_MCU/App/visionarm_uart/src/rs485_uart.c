#include "rs485_uart.h"

#include "app_config.h"
#include "stm32f1xx_hal.h"

#define RS485_DIRECTION_GPIO_PORT GPIOD
#define RS485_DIRECTION_GPIO_PIN  GPIO_PIN_7

#define RS485_UART_RX_ERROR_SR_MASK \
    (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)

static UART_HandleTypeDef s_huart2;
static volatile bool s_initialized = false;
static volatile bool s_tx_busy = false;

static void Rs485Uart_DwtInit(void);
static void Rs485Uart_DelayUs(uint32_t delay_us);
static uint32_t Rs485Uart_SrErrorsToHalErrors(uint32_t status_register);

bool Rs485Uart_Init(void)
{
    HAL_StatusTypeDef status;

    if (s_initialized)
    {
        Rs485Uart_EnterReceive();
        return true;
    }

    s_huart2.Instance = USART2;
    s_huart2.Init.BaudRate = APP_UART_BAUDRATE;
    s_huart2.Init.WordLength = UART_WORDLENGTH_8B;
    s_huart2.Init.StopBits = UART_STOPBITS_1;
    s_huart2.Init.Parity = UART_PARITY_NONE;
    s_huart2.Init.Mode = UART_MODE_TX_RX;
    s_huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
#if defined(USART_CR1_OVER8)
    s_huart2.Init.OverSampling = UART_OVERSAMPLING_16;
#endif

    status = HAL_UART_Init(&s_huart2);
    if (status != HAL_OK)
    {
        Rs485Uart_EnterReceive();
        return false;
    }

    Rs485Uart_DwtInit();

    s_tx_busy = false;
    s_initialized = true;

    /*
     * Safe physical default only.
     * RXNE/ERR interrupts are enabled later by ProtocolRxTask.
     */
    Rs485Uart_EnterReceive();
    return true;
}

void Rs485Uart_StartReceive(void)
{
    if (!s_initialized)
    {
        return;
    }

    Rs485Uart_EnterReceive();

    /*
     * PE/FE/NE/ORE are cleared by the STM32F1 SR-read then DR-read sequence.
     * The HAL macro performs that sequence.
     */
    __HAL_UART_CLEAR_OREFLAG(&s_huart2);

    __HAL_UART_ENABLE_IT(&s_huart2, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&s_huart2, UART_IT_ERR);
}

bool Rs485Uart_StartTransmit(const uint8_t *data, size_t length)
{
    HAL_StatusTypeDef status;

    if ((!s_initialized) || (data == NULL) || (length == 0U) ||
        (length > (size_t)UINT16_MAX) || s_tx_busy)
    {
        return false;
    }

    s_tx_busy = true;
    Rs485Uart_EnterTransmit();
    Rs485Uart_DelayUs(APP_RS485_PRE_TX_GUARD_US);

    status = HAL_UART_Transmit_IT(&s_huart2,
                                  (uint8_t *)(uintptr_t)data,
                                  (uint16_t)length);
    if (status != HAL_OK)
    {
        s_tx_busy = false;
        Rs485Uart_EnterReceive();
        return false;
    }

    return true;
}

void Rs485Uart_AbortTransmit(void)
{
    if (!s_initialized)
    {
        return;
    }

    (void)HAL_UART_AbortTransmit(&s_huart2);
    s_tx_busy = false;
    Rs485Uart_EnterReceive();
}

void Rs485Uart_EnterReceive(void)
{
    HAL_GPIO_WritePin(RS485_DIRECTION_GPIO_PORT,
                      RS485_DIRECTION_GPIO_PIN,
                      GPIO_PIN_RESET);
}

void Rs485Uart_EnterTransmit(void)
{
    HAL_GPIO_WritePin(RS485_DIRECTION_GPIO_PORT,
                      RS485_DIRECTION_GPIO_PIN,
                      GPIO_PIN_SET);
}

void Rs485Uart_IrqHandler(void)
{
    uint32_t status_register;
    uint32_t control_register_1;
    uint32_t control_register_3;
    uint32_t received_data;
    uint32_t hal_errors;
    bool rx_interrupt_pending;
    bool error_interrupt_pending;

    /*
     * The application owns RXNE directly.
     *
     * We cannot enable RXNEIE and then pass RXNE to HAL_UART_IRQHandler()
     * without a HAL receive operation being armed, because the HAL RX path
     * expects pRxBuffPtr/RxXferCount to describe an active transfer.
     *
     * Therefore:
     *   1. sample SR/CR1/CR3,
     *   2. consume DR ourselves for RX/error,
     *   3. pass the remaining TXE/TC work to HAL.
     */
    status_register = READ_REG(s_huart2.Instance->SR);
    control_register_1 = READ_REG(s_huart2.Instance->CR1);
    control_register_3 = READ_REG(s_huart2.Instance->CR3);

    rx_interrupt_pending =
        (((status_register & USART_SR_RXNE) != 0U) &&
         ((control_register_1 & USART_CR1_RXNEIE) != 0U));

    error_interrupt_pending =
        (((status_register & RS485_UART_RX_ERROR_SR_MASK) != 0U) &&
         ((((control_register_3 & USART_CR3_EIE) != 0U)) ||
           ((control_register_1 & USART_CR1_RXNEIE) != 0U)));

    if (rx_interrupt_pending || error_interrupt_pending)
    {
        /*
         * Reading DR after SR clears RXNE and PE/FE/NE/ORE as specified for
         * STM32F1 USART.
         */
        received_data = READ_REG(s_huart2.Instance->DR);

        hal_errors = Rs485Uart_SrErrorsToHalErrors(status_register);

        if (hal_errors != HAL_UART_ERROR_NONE)
        {
            Rs485Uart_OnErrorFromISR(hal_errors);
        }
        else if (rx_interrupt_pending)
        {
            Rs485Uart_OnRxByteFromISR((uint8_t)(received_data & 0xFFU));
        }
        else
        {
            /* DR read was required only to clear a stale condition. */
        }
    }

    /*
     * RXNE/error flags handled above are now clear. HAL remains the owner of
     * the HAL TXE -> TC transmit state machine.
     */
    HAL_UART_IRQHandler(&s_huart2);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART2))
    {
        return;
    }

    /*
     * In STM32CubeF1 IT mode this callback is reached from the USART TC path,
     * not merely when the data register becomes empty.
     */
    Rs485Uart_DelayUs(APP_RS485_POST_TX_GUARD_US);
    Rs485Uart_EnterReceive();

    s_tx_busy = false;

    /*
     * Notify the RTOS TX owner only after the bus has been released to RX.
     */
    Rs485Uart_OnTxCompleteFromISR();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t error_flags;

    if ((huart == NULL) || (huart->Instance != USART2))
    {
        return;
    }

    /*
     * Normal RX errors are consumed directly in Rs485Uart_IrqHandler().
     * Keep this callback as a defensive path for errors reported by HAL.
     */
    error_flags = HAL_UART_GetError(huart);

    if (huart->gState != HAL_UART_STATE_BUSY_TX)
    {
        Rs485Uart_EnterReceive();
    }

    Rs485Uart_OnErrorFromISR(error_flags);
}

__weak void Rs485Uart_OnRxByteFromISR(uint8_t byte)
{
    (void)byte;
}

__weak void Rs485Uart_OnTxCompleteFromISR(void)
{
}

__weak void Rs485Uart_OnErrorFromISR(uint32_t hal_error_flags)
{
    (void)hal_error_flags;
}

static uint32_t Rs485Uart_SrErrorsToHalErrors(uint32_t status_register)
{
    uint32_t hal_errors = HAL_UART_ERROR_NONE;

    if ((status_register & USART_SR_PE) != 0U)
    {
        hal_errors |= HAL_UART_ERROR_PE;
    }

    if ((status_register & USART_SR_NE) != 0U)
    {
        hal_errors |= HAL_UART_ERROR_NE;
    }

    if ((status_register & USART_SR_FE) != 0U)
    {
        hal_errors |= HAL_UART_ERROR_FE;
    }

    if ((status_register & USART_SR_ORE) != 0U)
    {
        hal_errors |= HAL_UART_ERROR_ORE;
    }

    return hal_errors;
}

static void Rs485Uart_DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void Rs485Uart_DelayUs(uint32_t delay_us)
{
    uint32_t cycles_per_us;
    uint32_t target_cycles;
    uint32_t start_cycles;

    if (delay_us == 0U)
    {
        return;
    }

    cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U)
    {
        return;
    }

    if (delay_us > (UINT32_MAX / cycles_per_us))
    {
        delay_us = UINT32_MAX / cycles_per_us;
    }

    target_cycles = delay_us * cycles_per_us;
    start_cycles = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start_cycles) < target_cycles)
    {
        __NOP();
    }
}
