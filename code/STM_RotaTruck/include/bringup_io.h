#ifndef BRINGUP_IO_H
#define BRINGUP_IO_H

/* =====================================================================
 *  bringup_io — console I/O for the standalone bring-up envs
 *  (imu_test, loadcell_test).  The full firmware gets uart_print /
 *  uart_printf from command.c instead and does not build this file.
 *
 *  Output goes to BOTH:
 *    - SEGGER RTT over the ST-Link (PlatformIO serial monitor via
 *      monitor/filter_rtt.py, or tools/rtt_monitor.sh) — the DISC1's
 *      ST-Link virtual COM port is NOT wired to the F407;
 *    - USART2 TX = PA2 (115200 8N1) for a 3.3 V USB-serial adapter.
 *  Input: RTT first, then USART2 RX = PA3 (polled).
 *
 *  Also provides SysTick_Handler and HardFault_Handler for these envs.
 * =====================================================================*/

#include <stdbool.h>
#include <stddef.h>

/* RTT + USART2.  Call once after SystemClock_Config(). */
void bringupIoInit(void);

/* Non-blocking single character, -1 if none. */
int  bringupGetc(void);

/* Non-blocking line reader: collects printable characters until CR or LF.
 * Returns true once with a complete, non-empty line in buf (NUL-terminated);
 * characters past len-1 are dropped. */
bool bringupGetLine(char *buf, size_t len);

/* uart_print / uart_printf are declared in command.h (same signatures). */

#endif /* BRINGUP_IO_H */
