#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialise USART2 + start RX interrupt.
 * Call once from main() after motorControlSetup(). */
void commandSetup(void);

/* Drain the RX line buffer if a complete line is queued and execute it. */
void commandPoll(void);

/* USART2 interrupt handler — call from USART2_IRQHandler in
 * stm32f4xx_it.c. */
void commandUartISR(void);

/* Convenience print helpers used by motor_control.c (and elsewhere). */
void uart_print(const char *s);
void uart_printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* COMMAND_H */
