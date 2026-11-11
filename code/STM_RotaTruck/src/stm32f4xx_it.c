/*
 *  Interrupt handlers for STM RotaTruck — ODrive S1 branch.
 *
 *  Active IRQs:
 *    - SysTick      : HAL_Delay / HAL_GetTick timebase
 *    - CAN2_RX0     : ODrive CAN RX (heartbeats + feedback)
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

#include "stm32f4xx_hal.h"
#include "odrive_can.h"
#include "command.h"

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* CAN2 RX FIFO 0 interrupt — ODrive heartbeats + feedback. */
void CAN2_RX0_IRQHandler(void)
{
    odriveCanRxISR();
}

/* USART2 = ST-Link virtual COM port = command channel. */
void USART2_IRQHandler(void)
{
    commandUartISR();
}

/* No output channel survives a hard fault; spin so the debugger can see it. */
void HardFault_Handler(void)
{
    while (1) { }
}
