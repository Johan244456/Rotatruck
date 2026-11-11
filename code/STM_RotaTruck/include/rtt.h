#ifndef RTT_H
#define RTT_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Minimal SEGGER-RTT-compatible terminal (channel 0) over the debug probe.
 *
 * The STM32F407G-DISC1's ST-Link virtual COM port is not wired to the
 * F407, so text written here is how the firmware reaches the PC terminal
 * over the same mini-USB cable used for flashing: OpenOCD finds the
 * "SEGGER RTT" control block in RAM and serves channel 0 on a TCP port
 * (see tools/rtt_monitor.sh).
 *
 * Never blocks: with no host attached, output that does not fit in the
 * 4 KiB buffer is dropped (whole messages, never half a line). */

void rttInit(void);
void rttWrite(const char *s, size_t len);

/* Next byte typed in the RTT terminal, or -1 if none. */
int  rttGetc(void);

#ifdef __cplusplus
}
#endif

#endif /* RTT_H */
