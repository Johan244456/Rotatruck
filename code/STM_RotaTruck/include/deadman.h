#ifndef DEADMAN_H
#define DEADMAN_H

#include <stdbool.h>
#include <stdint.h>

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

/* Gestures (board.h DEADMAN_HOLD_MS / DEADMAN_CLICK_*). */
bool     deadmanHeld(void);                 /* pressed for >= DEADMAN_HOLD_MS          */
uint32_t deadmanPressStartMs(void);         /* HAL tick of the current / last press    */
bool     deadmanTakeTripleClick(uint32_t *first_press_ms);   /* once per triple click  */
uint8_t  deadmanTakeShortClicks(void);      /* 1-2 quick clicks that ended, once (0 = none) */

#ifdef __cplusplus
}
#endif

#endif /* DEADMAN_H */
