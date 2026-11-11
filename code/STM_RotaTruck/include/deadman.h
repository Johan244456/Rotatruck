#ifndef DEADMAN_H
#define DEADMAN_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 *  deadman — operator-presence (handle) safety switch on PE7.
 *  Spec FR1 (assist only with operator presence) and FR3 (safe state when
 *  the operator releases the handle).
 *
 *  Pressed  -> the assist may drive the wheels as normal.
 *  Released -> assist_control.c ramps both wheels down to 0 at
 *              DEADMAN_DECEL_REV_PER_S2 (a gradual, controlled stop, not a
 *              freewheel and not a slam), then holds them at 0 until the
 *              switch is pressed again.
 *
 *  Fail-safe: the switch is read with a pull resistor towards the RELEASED
 *  level, so a broken wire or unplugged switch reads as released (stop).
 *  Polled + debounced (release is accepted faster than press).
 * =====================================================================*/

void deadmanSetup(void);
void deadmanPoll(void);          /* every main-loop pass */
bool deadmanPressed(void);       /* debounced; always true if DEADMAN_ENABLE is 0 */

#ifdef __cplusplus
}
#endif

#endif /* DEADMAN_H */
