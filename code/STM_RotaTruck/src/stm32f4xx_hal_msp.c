/*
 *  HAL MSP — peripheral clock + GPIO AF + NVIC setup for the F407 DISC1.
 *  Called by HAL whenever a peripheral is initialised.  Shared by every
 *  build env (replaces the Nucleo-G4 stm32g4xx_hal_msp.c).
 *
 *  The CAN MSP (CAN2 on PB12/PB13) is HAL_CAN_MspInit in odrive_can.c.
 */

#include "stm32f4xx_hal.h"
#include "board.h"

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

/* =====================================================================
 *  I2C MSP — ISM330DHCX IMU + 16x2 LCD share I2C1 (PB8 SCL / PB9 SDA)
 * =====================================================================*/
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != IMU_I2C_INSTANCE) return;
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Internal pull-ups on as well as the breakouts' own. */
    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_OD;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = IMU_I2C_GPIO_AF;
    g.Pin = IMU_I2C_SCL_PIN; HAL_GPIO_Init(IMU_I2C_SCL_PORT, &g);
    g.Pin = IMU_I2C_SDA_PIN; HAL_GPIO_Init(IMU_I2C_SDA_PORT, &g);

    /* Enable the clock only after the pins are on AF, then pulse the
     * peripheral reset - avoids the F4 I2C "BUSY stuck after init" erratum. */
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != IMU_I2C_INSTANCE) return;
    __HAL_RCC_I2C1_CLK_DISABLE();
}

/* =====================================================================
 *  USART MSP — USART2 TX = PA2, RX = PA3, 115200 8N1.
 *  NOTE: on the DISC1 this is NOT the ST-Link virtual COM port (that
 *  isn't wired to the F407); it needs a 3.3 V USB-serial adapter.  The
 *  NVIC line is enabled here but only fires once HAL_UART_Receive_IT()
 *  is called (command.c); the bring-up envs poll instead.
 * =====================================================================*/
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance != DEBUG_UART_INSTANCE) return;
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = DEBUG_UART_AF;
    g.Pin = DEBUG_UART_TX_PIN; HAL_GPIO_Init(DEBUG_UART_TX_PORT, &g);
    g.Pin = DEBUG_UART_RX_PIN; HAL_GPIO_Init(DEBUG_UART_RX_PORT, &g);

    HAL_NVIC_SetPriority(DEBUG_UART_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DEBUG_UART_IRQn);
}
