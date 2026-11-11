/*
 *  payload.c — assist payload source: latched load detection or manual.
 *  See payload.h for the latch rule.
 */

#include "payload.h"
#include "loadcell.h"
#include "assist_control.h"
#include "admittance.h"
#include "observer.h"
#include "board.h"
#include "command.h"
#include "stm32g4xx_hal.h"

#include <math.h>

typedef enum { SRC_MANUAL = 0, SRC_LATCHED } payload_src_t;

static payload_src_t s_src        = SRC_MANUAL;
static float         s_manual_kg  = 0.0f;   /* buttons / W, starts at 0, not saved */

/* latch state */
static bool  s_armed_at_standing  = false;  /* assist switched ON while STANDING   */
static bool  s_have_latch         = false;  /* a latch happened this session        */
static float s_latched_kg         = 0.0f;
static float s_latched_measured   = 0.0f;   /* measured kg behind the latch (log)   */

/* last valid live reading in the current standing period */
static bool  s_stand_ok           = false;
static float s_stand_ok_kg        = 0.0f;   /* 0.9 x measured                      */
static float s_stand_ok_measured  = 0.0f;

/* edge detection */
static bool      s_prev_enabled   = false;
static TruckPose s_prev_pose      = POSE_STANDING;

static float clamp_manual(float kg)
{
    if (kg < 0.0f)              kg = 0.0f;
    if (kg > UI_PAYLOAD_MAX_KG) kg = UI_PAYLOAD_MAX_KG;
    return kg;
}

float payloadGetKg(void)    { return (s_src == SRC_LATCHED) ? s_latched_kg : s_manual_kg; }
bool  payloadIsManual(void) { return s_src == SRC_MANUAL; }

void payloadInit(void)
{
    s_src = SRC_MANUAL;
    s_manual_kg = 0.0f;
    s_armed_at_standing = s_have_latch = s_stand_ok = false;
    s_prev_enabled = admittanceIsEnabled();
    s_prev_pose    = assistGetPose();
}

/* Push to the observer only on a real change: observerSetPayloadKg()
 * re-seeds the momentum estimate. */
static void push_to_observer(void)
{
    float kg = payloadGetKg();
    if (fabsf(kg - observerGetPayloadKg()) > 0.01f) observerSetPayloadKg(kg);
}

void payloadUpdate(void)
{
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
            s_latched_kg       = s_stand_ok_kg;
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
    s_manual_kg = clamp_manual(payloadGetKg() + delta_kg);
    s_src = SRC_MANUAL;
    uart_printf("PAYLOAD: %.0f kg (manual)\r\n", (double)s_manual_kg);
    push_to_observer();
}

void payloadSetManual(float kg)
{
    s_manual_kg = clamp_manual(kg);
    s_src = SRC_MANUAL;
    push_to_observer();
}

bool payloadUseLatched(void)
{
    if (!s_have_latch) return false;
    s_src = SRC_LATCHED;
    push_to_observer();
    return true;
}

void payloadPrintStatus(void)
{
    uart_printf("Payload: %.1f kg (%s)  |  effective cart mass: %.1f kg\r\n",
                (double)payloadGetKg(), s_src == SRC_LATCHED ? "LATCHED from load detection" : "MANUAL",
                (double)observerGetMassKg());
    if (s_have_latch)
        uart_printf("  last latch: %.1f kg (measured %.1f kg)\r\n",
                    (double)s_latched_kg, (double)s_latched_measured);
    else
        uart_print("  last latch: none this session\r\n");
    uart_printf("  manual: %.1f kg   armed at STANDING: %s\r\n",
                (double)s_manual_kg, s_armed_at_standing ? "yes" : "no");
    uart_printf("  load detection now: %s, measured %.1f kg -> %.1f kg (0.9 x)%s\r\n",
                loadcellStatusName(), (double)loadcellMeasuredKg(), (double)loadcellAssistKg(),
                s_stand_ok ? ", valid reading this standing period" : "");
}
