/*
 *  HAL MSP — peripheral-specific clock + GPIO AF + NVIC setup.
 *  Called by HAL whenever a peripheral is initialised.
 *
 *  ODrive S1 branch: the previous PWM/encoder MSP entries are removed
 *  because the BLD300B is gone — all motor control now goes over CAN.
 *  Only FDCAN, I2C (IMU/LCD), USART (debug), and simple GPIO UI remain.
 */

#include "stm32g4xx_hal.h"
#include "board.h"

extern void Error_Handler(void);   /* defined in main.c */

/* =====================================================================
 *  FDCAN MSP — ODrive CAN bus on PA11 (RX) / PA12 (TX)
 *
 *  FDCAN1 kernel clock on STM32G4 defaults to PCLK1 (= 170 MHz on this
 *  board), which is what the bit-timing constants in board.h assume.
 *  If the clock source is reconfigured elsewhere (e.g. PLL1Q via
 *  RCC PeriphCLKConfig), the bit timing must be recomputed.
 * =====================================================================*/
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance != ODRIVE_CAN_INSTANCE) return;

    /* Select the FDCAN kernel clock source.  This is NOT optional: the
     * RCC FDCANSEL mux resets to HSE, and this board has no HSE (HSI-only
     * clock tree), so without this the peripheral has no kernel clock and
     * the CAN core can't generate or sample a single bit — HAL_FDCAN_Init
     * still succeeds (APB registers only) but the bus is dead. */
    RCC_PeriphCLKInitTypeDef periph = {0};
    periph.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    periph.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) {
        Error_Handler();
    }

    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = ODRIVE_CAN_GPIO_AF;

    g.Pin = ODRIVE_CAN_TX_PIN;  HAL_GPIO_Init(ODRIVE_CAN_TX_PORT, &g);
    g.Pin = ODRIVE_CAN_RX_PIN;  HAL_GPIO_Init(ODRIVE_CAN_RX_PORT, &g);

    HAL_NVIC_SetPriority(ODRIVE_CAN_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ODRIVE_CAN_IRQn);
}

/* =====================================================================
 *  I2C MSP — ISM330DHCX IMU + 16x2 LCD share I2C1
 * =====================================================================*/
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != IMU_I2C_INSTANCE) return;

    /* I2C1 is shared by the ISM330DHCX IMU and the 16x2 LCD. Internal
     * pull-ups are enabled here - remove them if the breakout already
     * has pull-ups. */

    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_OD;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = IMU_I2C_GPIO_AF;

    g.Pin = IMU_I2C_SCL_PIN; HAL_GPIO_Init(IMU_I2C_SCL_PORT, &g);
    g.Pin = IMU_I2C_SDA_PIN; HAL_GPIO_Init(IMU_I2C_SDA_PORT, &g);
}

/* =====================================================================
 *  USART MSP — debug VCP on USART2
 * =====================================================================*/
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance != DEBUG_UART_INSTANCE) return;

    /* USART2 -> ST-Link virtual COM port (same USB cable as the debugger). */

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = DEBUG_UART_AF;

    g.Pin = DEBUG_UART_TX_PIN;  HAL_GPIO_Init(DEBUG_UART_TX_PORT, &g);
    g.Pin = DEBUG_UART_RX_PIN;  HAL_GPIO_Init(DEBUG_UART_RX_PORT, &g);

    HAL_NVIC_SetPriority(DEBUG_UART_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DEBUG_UART_IRQn);
}
