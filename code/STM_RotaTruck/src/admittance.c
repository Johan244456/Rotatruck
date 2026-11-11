/*
 *  admittance.c — power-assist loop (direct-torque force amplification).
 *
 *  The motor applies a force proportional to the operator's push:
 *      F_assist = ASSIST_GAIN * (operator force beyond the deadband),
 *                 faded to zero by ASSIST_V_ROLLOFF_MPS; the only braking
 *                 is the overspeed brake above ASSIST_V_HARD_LIMIT_MPS.
 *      tau_per_wheel = F_assist * r / 2   (clamped)
 *  sent via Set_Input_Torque.  The ODrive applies this torque directly,
 *  so you feel it immediately whether or not the wheel is loaded.
 *
 *  No continuous velocity damper (it made the state flicker and feel like
 *  it was fighting you).  The only braking is the overspeed safety:
 *  above ASSIST_V_HARD_LIMIT_MPS the loop brakes (-BRAKE_GAIN * v).
 *
 *  Stability: the observer subtracts the motor's own torque (Iq*Kt), so
 *  the assist's contribution is removed from Fext and the loop does not
 *  self-drive.  The overspeed brake + per-wheel torque cap are the
 *  backstops if the Iq feedback is degraded.
 *
 *  Safety: opt-in (AS 1), disabled at boot, per-wheel torque hard-capped.
 */

#include "admittance.h"
#include "assist_tuning.h"
#include "board.h"
#include "observer.h"
#include "motor_control.h"
#include "assist_control.h"   /* assistDriveActive() — pose gate */
#include "loadcell.h"         /* refuse assist while the LD console is open */
#include "command.h"
#include "stm32g4xx_hal.h"

#include <math.h>

static bool     s_enabled = false;
static uint32_t s_last_ms = 0;
static float    s_F_assist = 0.0f;       /* smoothed commanded force, N */
static const char *s_state_name = "IDLE";

/* Standstill guard: only believe the observer's force when the cart is
 * actually moving (it has no valid force signal at a dead stop). */
static bool     s_moving = false;
static uint32_t s_still_since_ms = 0;    /* last time |v| was >= MOVE_OFF */

static inline float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

void admittanceInit(void)
{
    s_enabled        = false;
    s_F_assist       = 0.0f;
    s_state_name     = "IDLE";
    s_moving         = false;
    s_still_since_ms = 0;
}

bool admittanceEnable(bool enabled)
{
    if (enabled == s_enabled) return true;

    if (enabled) {
        /* The load-detect console is a maintenance mode: its calibration
         * commands block the main loop for seconds, so no assist while it
         * is open (the ODrive would keep the last torque command). */
        if (loadcellConsoleActive()) {
            uart_print("ASSIST: refused - load-detect console open ('LD' to leave).\r\n");
            return false;
        }
        if (!motorControlEnableTorqueMode()) {
            uart_print("ASSIST: cannot enable — no ODrive heartbeat.\r\n");
            return false;
        }
        observerTare();                 /* zero the resting bias on entry */
        s_F_assist       = 0.0f;
        s_moving         = false;       /* require real motion before assisting */
        s_still_since_ms = HAL_GetTick();
        s_last_ms        = HAL_GetTick();
        s_enabled        = true;
        uart_print("ASSIST: ENABLED (torque). Push the wheel to feel it. "
                   "`AS 0` or `S` to stop.\r\n");
    } else {
        setMotorLeftTorque(0.0f);
        setMotorRightTorque(0.0f);
        observerSetMotorTorque(0.0f, 0.0f);
        motorControlDisableClosedLoop();
        s_F_assist   = 0.0f;
        s_enabled    = false;
        s_state_name = "OFF";
        uart_print("ASSIST: disabled (axes IDLE).\r\n");
    }
    return true;
}

bool admittanceIsEnabled(void) { return s_enabled; }

void admittanceToggleWithComp(void)
{
    admittanceEnable(!s_enabled);
    const bool on = s_enabled;          /* actual state (enable can be refused) */
    /* OB G follows the assist only when GRAVITY_COMP_AUTO_ENABLE is set;
     * otherwise it stays as last toggled with `OB G`. */
    if (GRAVITY_COMP_AUTO_ENABLE || !on) observerSetGravityComp(on);
    observerSetFrictionComp(on);        /* OB F follows the assist */
}

void admittanceStep(void)
{
    if (!s_enabled) return;

    uint32_t now = HAL_GetTick();
    /* self-pace: skip until 1/ASSIST_RATE_HZ (~10 ms at 100 Hz) since the last run */
    if ((uint32_t)(now - s_last_ms) < (uint32_t)(1000.0f / ASSIST_RATE_HZ)) return;
    s_last_ms = now;

    /* Pose gate: while STANDING or within the drive-entry brake delay, the
     * pose machine is actively holding the motors (velocity mode) — don't
     * command assist torque against it. */
    if (!assistDriveActive()) {
        observerSetMotorTorque(0.0f, 0.0f);
        s_F_assist   = 0.0f;
        s_state_name = "HOLD";
        return;
    }

    const float Fext  = observerGetTauExtTrans();   /* operator push, N    */
    const float v_act = observerGetVTransMps();      /* measured speed, m/s */

    const bool obs_ok = (observerGetStatus() == OBSERVER_OK)
                     || (observerGetStatus() == OBSERVER_DEGRADED);

    /* Standstill guard (velocity hysteresis): the observer's force estimate is
     * only meaningful while the cart is moving.  At a dead stop the residual
     * is just static friction reacting our own torque (a phantom -tau), and
     * amplifying it makes a loaded cart chatter FWD<->BACK.  Track a "moving"
     * latch: set it once the cart clears MOVE_ON, drop it after it has stayed
     * below MOVE_OFF for the settle time. */
    const float av = fabsf(v_act);
    if (av >= ASSIST_V_MOVE_OFF_MPS) {
        s_still_since_ms = now;                       /* not still — reset clock */
        if (av > ASSIST_V_MOVE_ON_MPS) s_moving = true;
    } else if ((uint32_t)(now - s_still_since_ms) >= ASSIST_STANDSTILL_SETTLE_MS) {
        s_moving = false;
    }

    /* Deadband the operator force. */
    float F_user = 0.0f;
    if (obs_ok && s_moving) {
        if (Fext >  ASSIST_DEADBAND_N) F_user = Fext - ASSIST_DEADBAND_N;
        else if (Fext < -ASSIST_DEADBAND_N) F_user = Fext + ASSIST_DEADBAND_N;
    }

    /* Amplify the deadbanded operator force by ASSIST_GAIN. */
    float F_cmd = ASSIST_GAIN * F_user;

    /* Velocity rolloff: fade the assist to zero by ASSIST_V_ROLLOFF_MPS so
     * the motor can't drive the cart past that speed — the main runaway
     * bound (a phantom force estimate still can't accelerate without end). */
    float roll = 1.0f - fabsf(v_act) / ASSIST_V_ROLLOFF_MPS;
    if (roll < 0.0f) roll = 0.0f;
    F_cmd *= roll;

    /* Overspeed safety: brake instead of assist. */
    bool braking = false;
    if (fabsf(v_act) > ASSIST_V_HARD_LIMIT_MPS) {
        F_cmd = -ASSIST_BRAKE_GAIN_NS_PER_M * v_act;
        braking = true;
    }

    F_cmd = clampf(F_cmd, -ASSIST_FORCE_MAX_N, ASSIST_FORCE_MAX_N);

    /* Smooth the commanded force so the assist can't reverse fast enough to
     * self-oscillate on a low-inertia free wheel. */
    s_F_assist = ASSIST_CMD_LPF_ALPHA * F_cmd
               + (1.0f - ASSIST_CMD_LPF_ALPHA) * s_F_assist;

    /* Distribute to per-wheel torque: F = (tau_L + tau_R)/r -> tau = F*r/2. */
    float tau = s_F_assist * WHEEL_RADIUS_M * 0.5f;
    tau = clampf(tau, -ASSIST_TORQUE_CAP_NM, ASSIST_TORQUE_CAP_NM);

    setMotorLeftTorque(tau);
    setMotorRightTorque(tau);

    /* Tell the observer exactly what torque we're applying so it subtracts
     * the motor's own contribution (no measured-Iq dependency). */
    observerSetMotorTorque(tau, tau);

    if      (braking)             s_state_name = "BRAKING";
    else if (s_F_assist >  0.5f)  s_state_name = "ASSIST FWD";
    else if (s_F_assist < -0.5f)  s_state_name = "ASSIST BACK";
    else                          s_state_name = "IDLE";
}

float admittanceGetAssistForce(void)  { return s_F_assist; }
const char *admittanceStateName(void) { return s_enabled ? s_state_name : "OFF"; }