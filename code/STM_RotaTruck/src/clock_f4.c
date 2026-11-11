/*
 *  clock_f4.c — system clock for the STM32F407G-DISC1.
 *
 *  168 MHz from the 8 MHz HSE crystal (X2 on the DISC1):
 *    8 / M8 * N336 / P2 = 168 MHz SYSCLK, Q7 = 48 MHz (USB).
 *    AHB = 168 MHz, APB1 = 42 MHz (I2C1, USART2, CAN2), APB2 = 84 MHz.
 *
 *  The CAN bit timing in board.h assumes APB1 = 42 MHz; odriveCanInit()
 *  refuses to start if it isn't.  Shared by every build env.
 */

#include "stm32f4xx_hal.h"

void SystemClock_Config(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitTypeDef o = {0};
    o.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    o.HSEState       = RCC_HSE_ON;
    o.PLL.PLLState   = RCC_PLL_ON;
    o.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    o.PLL.PLLM       = 8;
    o.PLL.PLLN       = 336;
    o.PLL.PLLP       = RCC_PLLP_DIV2;
    o.PLL.PLLQ       = 7;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) { while (1) { } }

    RCC_ClkInitTypeDef c = {0};
    c.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV4;
    c.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_5) != HAL_OK) { while (1) { } }
}
