/*
 *  autoseq.c — drive-entry calibrate (+ autonomous arm) sequencer.
 *
 *  Watches the pose.  On each STANDING -> DRIVING transition, once the cart
 *  has settled (not tilting) for AUTOSEQ_SETTLE_MS, it runs the drive-entry
 *  calibration the operator used to type by hand — in BOTH modes:
 *
 *      OD C   motorControlClearErrors()      (clear any latched faults)
 *      OB T   observerTare()                 (zero the resting force)
 *      OB Z   observerZeroPitch()            (pitch baseline, in DRIVE pose)
 *      OB G   observerSetGravityComp(true)
 *      OB F   observerSetFrictionComp(true)
 *
 *  In AUTONOMOUS mode (R A) it additionally arms the assist (AS 1) after the
 *  calibration.  In MANUAL mode (the default) the assist is left for the
 *  operator to toggle with the D8 / B1 button — which also re-enables OB G/F.
 *
 *  The pose machine still owns the motors: braked while STANDING / during
 *  the drive-entry delay, released to the assist once settled in DRIVING.
 *  Re-runs on every drive entry so each start is freshly tared/zeroed.
 *  Payload set with `W <kg>` (e.g. while standing) is untouched and carries
 *  through.
 */

#include "autoseq.h"
#include "assist_tuning.h"
#include "assist_control.h"
#include "observer.h"
#include "admittance.h"
#include "motor_control.h"
#include "command.h"
#include "stm32g4xx_hal.h"

static RunMode   s_mode      = RUN_MODE_MANUAL;
static TruckPose s_last_pose = POSE_STANDING;
static bool      s_pending   = false;   /* drive entered, sequence not yet run */
static uint32_t  s_settle_t  = 0;       /* timestamp the cart last settled */

void autoseqInit(void)
{
    s_mode      = RUN_MODE_MANUAL;
    s_last_pose = POSE_STANDING;
    s_pending   = false;
}

RunMode autoseqGetMode(void) { return s_mode; }

void autoseqSetMode(RunMode mode)
{
    s_mode    = mode;
    s_pending = false;
    if (mode == RUN_MODE_AUTO) {
        /* Force last-pose = STANDING so a cart already tilted into DRIVE
         * re-triggers the sequence right away. */
        s_last_pose = POSE_STANDING;
        uart_print("MODE: AUTONOMOUS — tilt to DRIVE and it self-calibrates + arms.\r\n");
    } else {
        uart_print("MODE: MANUAL — DRIVE auto-calibrates; arm assist with the "
                   "D8 / B1 button.\r\n");
    }
}

/* Drive-entry calibration — runs on EVERY standing->driving transition, in
 * both manual and autonomous mode.  Tares + zeroes the observer in the drive
 * pose and switches ON gravity + friction compensation, so the force estimate
 * is accurate the instant you start pushing. */
static void run_drive_entry_cal(void)
{
    uart_print(GRAVITY_COMP_AUTO_ENABLE
               ? "DRIVE: calibrating (clear errors, tare, zero pitch + slope, "
                 "gravity + friction comp on)...\r\n"
               : "DRIVE: calibrating (clear errors, tare, zero pitch + slope, "
                 "friction comp on; gravity comp manual via OB G)...\r\n");
    motorControlClearErrors();          /* OD C */
    observerTare();                     /* OB T */
    observerZeroPitch();                /* OB Z (also resets the slope estimate) */
    if (GRAVITY_COMP_AUTO_ENABLE) {
        observerSetGravityComp(true);   /* OB G */
    }
    observerSetFrictionComp(true);      /* OB F */
}

void autoseqUpdate(void)
{
    TruckPose pose = assistGetPose();

    /* Arm the sequence on entry to DRIVING; cancel it if we fall back to
     * STANDING before it has run. */
    if (pose == POSE_DRIVE && s_last_pose != POSE_DRIVE) {
        s_pending  = true;
        s_settle_t = HAL_GetTick();
    } else if (pose == POSE_STANDING) {
        s_pending = false;
    }
    s_last_pose = pose;

    if (s_pending) {
        /* Hold off (and keep resetting the timer) until the cart is still. */
        if (assistIsTilting()) {
            s_settle_t = HAL_GetTick();
        } else if ((uint32_t)(HAL_GetTick() - s_settle_t) >= AUTOSEQ_SETTLE_MS) {
            run_drive_entry_cal();          /* always: OD C, OB T/Z/G/F */
            if (s_mode == RUN_MODE_AUTO) {
                admittanceEnable(true);     /* AS 1 — auto-arm in AUTO only */
                uart_printf("AUTO: armed.  payload %.0f kg "
                            "(effective mass %.0f kg).\r\n",
                            (double)observerGetPayloadKg(),
                            (double)observerGetMassKg());
            }
            s_pending = false;
        }
    }
}