#ifndef AUTOSEQ_H
#define AUTOSEQ_H

/* =====================================================================
 *  autoseq.h — autonomous arm-and-calibrate sequencer.
 *
 *  In AUTONOMOUS mode (`R A`), every time the cart settles into the
 *  DRIVING pose the firmware runs the start-up sequence by itself —
 *  clear ODrive errors, tare the force estimate, zero the pitch baseline,
 *  enable gravity + friction compensation, and arm the assist — so the
 *  operator just tilts the cart and pushes.  In MANUAL mode the
 *  drive-entry calibration still runs on each DRIVE entry; only the
 *  assist auto-arm is skipped, leaving the operator to arm with the
 *  D8 / B1 button.
 *
 *  The standing/driving brake behaviour and the manual payload input
 *  (`W <kg>`) are unchanged in both modes.
 * =====================================================================*/

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { RUN_MODE_MANUAL = 0, RUN_MODE_AUTO = 1 } RunMode;

void    autoseqInit(void);
void    autoseqUpdate(void);            /* tick from the main loop */
void    autoseqSetMode(RunMode mode);   /* R A / R M */
RunMode autoseqGetMode(void);

#ifdef __cplusplus
}
#endif

#endif /* AUTOSEQ_H */