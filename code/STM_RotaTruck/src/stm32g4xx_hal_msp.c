/*
 *  HAL MSP — peripheral-specific clock + GPIO AF + NVIC setup.
 *  Called by HAL whenever a peripheral is initialised.
 *
 *  ODrive S1 branch: the previous PWM/encoder MSP entries are removed
 *  because the BLD300B is gone — all motor control now goes over CAN.
 *  Only I2C (IMU/LCD), USART (debug), and simple GPIO UI remain
 *  (the CAN MSP lives in odrive_can.c).
 */

#include "stm32g4xx_hal.h"
#include "board.h"

extern void Error_Handler(void);   /* defined in main.c */

/* CAN MSP (CAN2 on PB12/PB13) is HAL_CAN_MspInit in odrive_can.c. */

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
