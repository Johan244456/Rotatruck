/*
 *  Interrupt handlers for STM RotaTruck — ODrive S1 branch.
 *
 *  Active IRQs:
 *    - SysTick      : HAL_Delay / HAL_GetTick timebase
 *    - FDCAN1_IT0   : ODrive CAN RX (heartbeats + feedback)
 *    - USART2       : debug VCP RX bytes for the command parser
 *
 *  Fault handlers (HardFault/NMI/etc.) are not defined here — the
 *  CMSIS/HAL weak defaults apply (infinite loop, no output).
 *
 *  Buttons are polled, not interrupt-driven (PC13 USER in button.c,
 *  payload +/- in mode_input.c).
 *  The TIM3 encoder-overflow handler from Phase 1 is gone — encoders
 *  are read off the ODrive over CAN now.
 */

#include "stm32g4xx_hal.h"
#include "odrive_can.h"
#include "command.h"

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* FDCAN1 main interrupt line — RX FIFO 0 plus most other status bits. */
void FDCAN1_IT0_IRQHandler(void)
{
    odriveCanRxISR();
}

/* USART2 = ST-Link virtual COM port = command channel. */
void USART2_IRQHandler(void)
{
    commandUartISR();
}
