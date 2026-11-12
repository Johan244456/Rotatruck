#ifndef PAYLOAD_H
#define PAYLOAD_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 *  payload — decides which payload mass the assist uses (observer mass +
 *  friction model), from automatic load detection or the manual buttons.
 *
 *  LATCH RULE (load detection -> assist):
 *    1. The assist is switched ON (D8 / B1 button, or AS 1) while the
 *       truck is in STANDING.  This "arms" the latch for that session.
 *    2. While standing, the last VALID live load-detect reading (status OK
 *       or OVERLOAD) is remembered: LOAD_ASSIST_SCALE x measured, capped at rated.
 *    3. On the STANDING -> DRIVE (tipped back) transition, that final
 *       reading is LATCHED as the assist payload and stays fixed for the
 *       push.  Every later STANDING -> DRIVE in the same armed session
 *       re-latches from that standing period.
 *
 *  FALLBACK (manual):
 *    - Assist switched ON outside STANDING  -> no latch, manual payload.
 *    - No valid reading during the standing period (NO_CAL, NO_DATA,
 *      IMU_FAULT, never settled ...)          -> manual payload.
 *    - The +/- buttons (PB6 / PC7, 5 kg steps) or `W <kg>` set the manual
 *      payload and take over immediately (also overriding a latch, starting
 *      from the value in use).  The next armed STANDING -> DRIVE latches
 *      load detection again.
 *  The manual payload starts at PAYLOAD_DEFAULT_KG on power-up (not saved).
 *
 *  F407 (board.h PAYLOAD_ACCEPT_ON_HANDLE 1) replaces the LATCH RULE: a
 *  HANDLE SWITCH press while the load-detect pose is STANDING accepts the
 *  last settled standing reading (<= PAYLOAD_ACCEPT_MAX_AGE_MS old) as
 *  LOAD_ASSIST_SCALE (0.7) x measured; otherwise it is refused with the
 *  reason and the payload in use is kept.
 * =====================================================================*/

void  payloadInit(void);

/* Main loop, after loadcellUpdate() and assistUpdate().  Watches the assist
 * enable and pose edges, latches, and pushes changes to the observer. */
void  payloadUpdate(void);

/* Payload the assist is using now, and whether it is the manual value. */
float payloadGetKg(void);          /* ASSIST payload = payloadAssistScale() x initialised load */
float payloadAssistScale(void);    /* AUTO (measured) 0.7 = LOAD_ASSIST_SCALE, MANUAL 1.0 = MANUAL_ASSIST_SCALE */
float payloadGetLoadKg(void);      /* the initialised load (accepted / default / W / buttons) */
bool  payloadIsManual(void);

/* Manual payload: +/- buttons (delta) and `W <kg>` (absolute).  Both switch
 * the source to MANUAL straight away. */
void  payloadAdjustManual(float delta_kg);
void  payloadSetManual(float kg);

/* `W A`: back to the latched load-detect value of this session, if there is
 * one.  Returns false (and stays manual) if nothing has been latched. */
bool  payloadUseLatched(void);

/* `W`: source, value, latch state and load-detect status on the console. */
void  payloadPrintStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* PAYLOAD_H */
