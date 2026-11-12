/*
 *  payload.c — assist payload source: latched load detection or manual.
 *  See payload.h for the latch rule.
 */

#include "payload.h"
#include "loadcell.h"
#include "assist_control.h"
#include "admittance.h"
#include "observer.h"
#include "deadman.h"
#include "board.h"
#include "command.h"
#include "stm32f4xx_hal.h"

#include <math.h>

typedef enum { SRC_MANUAL = 0, SRC_LATCHED } payload_src_t;

static payload_src_t s_src        = SRC_MANUAL;
/* Both hold the INITIALISED LOAD (kg on the trolley); the assist gets
 * LOAD_ASSIST_SCALE (0.7) x a MEASURED load - never more than the real load -
 * and MANUAL_ASSIST_SCALE (1.0) x a manual one (payloadGetKg). */
static float         s_manual_kg  = PAYLOAD_DEFAULT_KG;   /* load: buttons / W, board.h default, not saved */

/* latch state */
static bool  s_armed_at_standing  = false;  /* assist switched ON while STANDING   */
static bool  s_have_latch         = false;  /* a latch happened this session        */
static float s_latched_kg         = 0.0f;
static float s_latched_measured   = 0.0f;   /* measured kg behind the latch (log)   */

/* last valid live reading in the current standing period */
static bool     s_stand_ok          = false;
static float    s_stand_ok_kg       = 0.0f;   /* LOAD_ASSIST_SCALE x measured      */
static float    s_stand_ok_measured = 0.0f;
static uint32_t s_stand_ok_ms       = 0;      /* when it was taken                 */
static bool     s_prev_standing     = false;  /* load-detect pose, previous pass   */
/* auto-read safety: steady-reading window and the manual override */
static float    s_auto_ref_kg       = 0.0f;   /* reading the steady window started at */
static uint32_t s_auto_since_ms     = 0;      /* 0 = no window running              */
static bool     s_manual_lock       = false;  /* `W <kg>`: auto-read paused           */
static bool     s_prev_handle       = true;   /* must be seen released first       */

/* edge detection */
static bool      s_prev_enabled   = false;
static TruckPose s_prev_pose      = POSE_STANDING;

static void push_to_observer(void);

static float clamp_manual(float kg)
{
    if (kg < 0.0f)              kg = 0.0f;
    if (kg > UI_PAYLOAD_MAX_KG) kg = UI_PAYLOAD_MAX_KG;
    return kg;
}

float payloadGetLoadKg(void) { return (s_src == SRC_LATCHED) ? s_latched_kg : s_manual_kg; }
/* AUTO (measured) load -> LOAD_ASSIST_SCALE (0.7); MANUAL -> MANUAL_ASSIST_SCALE (1.0). */
float payloadAssistScale(void) { return (s_src == SRC_LATCHED) ? LOAD_ASSIST_SCALE : MANUAL_ASSIST_SCALE; }
float payloadGetKg(void)     { return payloadAssistScale() * payloadGetLoadKg(); }
bool  payloadIsManual(void) { return s_src == SRC_MANUAL; }

void payloadInit(void)
{
    s_src = SRC_MANUAL;
    s_manual_kg = PAYLOAD_DEFAULT_KG;
    uart_printf("PAYLOAD: initialised load %.1f kg at power-up (board.h PAYLOAD_DEFAULT_KG; 'W <kg>' "
                "to change) -> assist payload %.1f kg (%.2f x).\r\n", (double)s_manual_kg,
                (double)payloadGetKg(), (double)payloadAssistScale());
    s_armed_at_standing = s_have_latch = s_stand_ok = false;
    s_prev_enabled = admittanceIsEnabled();
    s_prev_pose    = assistGetPose();
    s_prev_handle  = true;
#if PAYLOAD_ACCEPT_ON_HANDLE
#if PAYLOAD_AUTO_ACCEPT
    uart_printf("PAYLOAD: the load is read automatically whenever the trolley stands exactly upright "
                "and still -> assist gets %.2f x (frozen while tipped back; add load and stand it up "
                "again to update).  Triple-click tipped back = zero.  Hold = arm assist.\r\n",
                (double)LOAD_ASSIST_SCALE);
#else
    uart_printf("PAYLOAD: CLICK the handle once while STANDING (hands off ~1 s first) to accept "
                "the load -> assist gets %.2f x.  Triple-click tipped back = zero.  Hold = arm assist.\r\n",
                (double)LOAD_ASSIST_SCALE);
#endif
#endif
}

#if PAYLOAD_ACCEPT_ON_HANDLE
/* Handle press: accept the last settled standing reading if the IMU says the
 * trolley is (still) at the standing angle.  Otherwise refuse and say why;
 * the payload in use is kept. */
static void accept_on_press(uint32_t now)
{
    const float in_use = payloadGetLoadKg();   /* the initialised load kept on a refusal */
    const char *src    = (s_src == SRC_LATCHED) ? "accepted" : "manual";
    if (!loadcellImuOk()) {
        uart_printf("PAYLOAD: not accepted - no IMU, can't confirm the trolley is standing. "
                    "Keeping load %.1f kg (%s).\r\n", (double)in_use, src);
        return;
    }
    if (!loadcellPoseStanding() || loadcellStandDevDeg() > PAYLOAD_ACCEPT_MAX_DEV_DEG) {
        uart_printf("PAYLOAD: not accepted - not exactly at the standing angle (%.1f deg from standing, "
                    "need <= %.1f; pose %s). Keeping load %.1f kg (%s).\r\n", (double)loadcellStandDevDeg(),
                    (double)PAYLOAD_ACCEPT_MAX_DEV_DEG, loadcellPoseName(), (double)payloadGetLoadKg(), src);
        return;
    }
    /* reading taken before the press started (gripping disturbs the plate) */
    const uint32_t press = deadmanPressStartMs();
    if (!s_stand_ok || (int32_t)(press - s_stand_ok_ms) > (int32_t)PAYLOAD_ACCEPT_MAX_AGE_MS) {
        uart_printf("PAYLOAD: not accepted - no settled reading in the last %.1f s (load status %s). "
                    "Stand it, hands off ~1 s, then click. Keeping load %.1f kg (%s).\r\n",
                    (double)(PAYLOAD_ACCEPT_MAX_AGE_MS / 1000.0f), loadcellStatusName(),
                    (double)in_use, src);
        return;
    }
    float load = s_stand_ok_measured;
    if (load < 0.0f) load = 0.0f;
    if (load > UI_PAYLOAD_MAX_KG) load = UI_PAYLOAD_MAX_KG;
    s_latched_kg       = load;                /* the initialised load (frozen) */
    s_latched_measured = s_stand_ok_measured;
    s_have_latch       = true;
    s_src              = SRC_LATCHED;
    s_manual_lock      = false;               /* a click hands back to the scale */
    uart_printf("PAYLOAD: ACCEPTED - load initialised %.1f kg -> assist payload %.1f kg (%.2f x).\r\n",
                (double)load, (double)payloadGetKg(), (double)LOAD_ASSIST_SCALE);
}
#endif

/* Push to the observer only on a real change: observerSetPayloadKg()
 * re-seeds the momentum estimate. */
static void push_to_observer(void)
{
    float kg = payloadGetKg();
    if (fabsf(kg - observerGetPayloadKg()) > 0.01f) observerSetPayloadKg(kg);
}

void payloadUpdate(void)
{
#if PAYLOAD_ACCEPT_ON_HANDLE
    /* --- standing period (load-detect pose): remember the last settled reading --- */
    const uint32_t now      = HAL_GetTick();
    const bool     standing = loadcellPoseStanding();
    if (standing && !s_prev_standing) s_stand_ok = false;      /* new standing period */
    /* Readings taken AT the standing angle (<= PAYLOAD_ACCEPT_MAX_DEV_DEG) with a valid status
     * (OK or OVERLOAD).  A handle CLICK accepts the most recent one (as before); the AUTO-read
     * additionally needs it STEADY (+/-PAYLOAD_AUTO_STABLE_KG for PAYLOAD_AUTO_STABLE_MS). */
    bool steady = false;
    const loadcell_status_t st_now = loadcellStatus();
    const bool valid_upright = standing && loadcellStandDevDeg() <= PAYLOAD_ACCEPT_MAX_DEV_DEG &&
                               (st_now == LC_ST_OK || st_now == LC_ST_OVERLOAD);
    if (valid_upright) {                         /* click: most recent valid upright reading */
        s_stand_ok          = true;
        s_stand_ok_measured = loadcellMeasuredKg();
        s_stand_ok_kg       = LOAD_ASSIST_SCALE * s_stand_ok_measured;
        s_stand_ok_ms       = now;
    }
    if (valid_upright) {
        const float m = loadcellMeasuredKg();
        if (s_auto_since_ms == 0 || fabsf(m - s_auto_ref_kg) > PAYLOAD_AUTO_STABLE_KG) {
            s_auto_ref_kg = m; s_auto_since_ms = now ? now : 1;      /* (re)start the window */
        } else if ((uint32_t)(now - s_auto_since_ms) >= PAYLOAD_AUTO_STABLE_MS) {
            steady = true;
        }
    } else {
        s_auto_since_ms = 0;
    }
    if (steady) {
        {
#if PAYLOAD_AUTO_ACCEPT
            /* exactly upright + settled: this IS the load (frozen once it tips back) */
            float load = s_stand_ok_measured;
            if (load < 0.0f) load = 0.0f;
            if (load > UI_PAYLOAD_MAX_KG) load = UI_PAYLOAD_MAX_KG;
            if (!s_manual_lock &&
                (s_src != SRC_LATCHED || fabsf(load - s_latched_kg) > PAYLOAD_AUTO_MIN_CHANGE_KG)) {
                const bool first = (s_src != SRC_LATCHED);
                s_latched_kg       = load;
                s_latched_measured = s_stand_ok_measured;
                s_have_latch       = true;
                s_src              = SRC_LATCHED;
                uart_printf("PAYLOAD: load %s %.1f kg (upright, auto) -> assist payload %.1f kg (%.2f x).\r\n",
                            first ? "detected" : "updated", (double)load, (double)payloadGetKg(),
                            (double)LOAD_ASSIST_SCALE);
            }
#endif
        }
    }
    s_prev_standing = standing;

    /* --- handle: ONE click = accept (decided 0.5 s after the click, when no
     *     2nd / 3rd click followed); a hold only arms the assist --- */
    s_prev_handle = deadmanHeld();

    /* --- handle TRIPLE CLICK = manual load-detect zero (tipped back only) --- */
    uint32_t first_click;
    if (deadmanTakeTripleClick(&first_click) && loadcellZeroFromHandle(first_click)) {
        /* zeroed (tipped back, plate empty): the accepted load is no longer
         * known - back to the MANUAL load until the next upright reading / click */
        s_src         = SRC_MANUAL;
        s_manual_lock = false;
        uart_printf("PAYLOAD: scale zeroed - load back to MANUAL %.1f kg -> assist payload %.1f kg (%.2f x).\r\n",
                    (double)s_manual_kg, (double)payloadGetKg(), (double)payloadAssistScale());
        push_to_observer();
    }

    /* --- one click = accept the most recent load; two clicks = hint --- */
    uint8_t n = deadmanTakeShortClicks();
    if (n == 1) {
        accept_on_press(now);
    } else if (n == 2) {
        uart_printf("HANDLE: 2 quick clicks - nothing done. ONE click while STANDING accepts the load; "
                    "triple-click while tipped back zeroes it; hold arms the assist.  (pose now: %s, "
                    "%.0f deg from standing)\r\n", loadcellPoseName(), (double)loadcellStandDevDeg());
    }

    s_prev_enabled = admittanceIsEnabled();
    s_prev_pose    = assistGetPose();
    push_to_observer();
    return;
#endif
    const bool      en   = admittanceIsEnabled();
    const TruckPose pose = assistGetPose();

    /* --- standing period: remember the last valid live reading --- */
    if (pose == POSE_STANDING && s_prev_pose != POSE_STANDING) {
        s_stand_ok = false;                 /* new standing period */
    }
    if (pose == POSE_STANDING) {
        loadcell_status_t st = loadcellStatus();
        if (st == LC_ST_OK || st == LC_ST_OVERLOAD) {
            s_stand_ok          = true;
            s_stand_ok_kg       = loadcellAssistKg();
            s_stand_ok_measured = loadcellMeasuredKg();
        }
    }

    /* --- assist ON edge: armed only if switched on while STANDING --- */
    if (en && !s_prev_enabled) {
        s_armed_at_standing = (pose == POSE_STANDING);
        if (s_armed_at_standing) {
            uart_print("PAYLOAD: assist ON at STANDING - load will be latched when you tip back.\r\n");
        } else {
            s_src = SRC_MANUAL;
            uart_printf("PAYLOAD: assist ON outside STANDING - no load latch, using manual %.1f kg "
                        "(+/- buttons).\r\n", (double)s_manual_kg);
        }
    }
    if (!en) s_armed_at_standing = false;

    /* --- STANDING -> DRIVE with an armed session: latch --- */
    if (en && s_armed_at_standing && s_prev_pose == POSE_STANDING && pose == POSE_DRIVE) {
        if (s_stand_ok) {
            s_latched_kg       = s_stand_ok_measured;   /* load; the assist gets x LOAD_ASSIST_SCALE */
            s_latched_measured = s_stand_ok_measured;
            s_have_latch       = true;
            s_src              = SRC_LATCHED;
            uart_printf("PAYLOAD: latched %.1f kg for the assist (load detection measured %.1f kg).\r\n",
                        (double)s_latched_kg, (double)s_latched_measured);
        } else {
            s_src = SRC_MANUAL;
            uart_printf("PAYLOAD: no valid load reading while standing (load status %s) - "
                        "using manual %.1f kg (+/- buttons).\r\n",
                        loadcellStatusName(), (double)s_manual_kg);
        }
    }

    s_prev_enabled = en;
    s_prev_pose    = pose;
    push_to_observer();
}

void payloadAdjustManual(float delta_kg)
{
    /* Start from the value in use, so a press nudges a latched value. */
    s_manual_kg = clamp_manual(payloadGetLoadKg() + delta_kg);
    s_src = SRC_MANUAL;
    s_manual_lock = true;                      /* operator override: pause the auto-read */
    uart_printf("PAYLOAD: load %.0f kg (manual) -> assist payload %.1f kg (%.2f x)\r\n",
                (double)s_manual_kg, (double)payloadGetKg(), (double)payloadAssistScale());
    push_to_observer();
}

void payloadSetManual(float kg)
{
    s_manual_kg = clamp_manual(kg);
    s_src = SRC_MANUAL;
    s_manual_lock = true;                      /* operator override: pause the auto-read */
    uart_print("PAYLOAD: manual load - automatic reading paused until 'W A', a handle click or a restart.\r\n");
    push_to_observer();
}

bool payloadUseLatched(void)
{
    s_manual_lock = false;                     /* back to the scale */
    if (!s_have_latch) return false;
    s_src = SRC_LATCHED;
    push_to_observer();
    return true;
}

void payloadPrintStatus(void)
{
    uart_printf("Load: %.1f kg (%s) -> assist payload %.1f kg (%.2f x)  |  effective cart mass: %.1f kg\r\n",
                (double)payloadGetLoadKg(), s_src == SRC_LATCHED ? "ACCEPTED from load detection" : "MANUAL",
                (double)payloadGetKg(), (double)payloadAssistScale(), (double)observerGetMassKg());
    if (s_have_latch)
        uart_printf("  last accepted: load %.1f kg (measured %.1f kg)\r\n",
                    (double)s_latched_kg, (double)s_latched_measured);
    else
        uart_print("  last latch: none this session\r\n");
#if PAYLOAD_ACCEPT_ON_HANDLE
    uart_printf("  manual load: %.1f kg   accept: ONE click of the handle at STANDING\r\n",
                (double)s_manual_kg);
    uart_printf("  pose (load detection): %s, %.1f deg from standing\r\n",
                loadcellPoseName(), (double)loadcellStandDevDeg());
#else
    uart_printf("  manual: %.1f kg   armed at STANDING: %s\r\n",
                (double)s_manual_kg, s_armed_at_standing ? "yes" : "no");
#endif
    uart_printf("  load detection now: %s, measured %.1f kg -> %.1f kg (%.2f x)%s\r\n",
                loadcellStatusName(), (double)loadcellMeasuredKg(), (double)loadcellAssistKg(),
                (double)LOAD_ASSIST_SCALE, s_stand_ok ? ", valid reading this standing period" : "");
}
