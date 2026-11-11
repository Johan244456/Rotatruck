#ifndef ASSIST_TUNING_H
#define ASSIST_TUNING_H

/* =====================================================================
 *  assist_tuning.h — tunable parameters for the momentum-observer +
 *  admittance assist stack on the ODrive_S1 branch.
 *
 *  Ported from the State_Observer_Control branch and adapted for:
 *    - ODrive BotWheel hub motors (torque constant from ODrive config)
 *    - the observer's motor wrench from commanded torque (the Iq_measured
 *      over CAN / Get_Iq feedback path is currently disabled)
 *    - the ODrive running its own FOC / inner current loop, so the
 *      STM32 only runs the OUTER observer + admittance + steering loop
 *
 *  Hardware geometry (WHEEL_RADIUS_M etc.) lives in board.h — do not
 *  redefine it here.
 *
 *  Sentinel convention: values still needing bench measurement are set
 *  to TUNING_TODO so tuning_is_todo() can detect them; the observer then
 *  falls back to conservative defaults and flags DEGRADED rather than
 *  flying off a wrong number.
 * =====================================================================*/

#include <math.h>
#include "board.h"          /* WHEEL_RADIUS_M, MOTOR_*_DIR, node IDs */

#define TUNING_TODO            (-9999.0f)

static inline int tuning_is_todo(float v)
{
    return v < -9000.0f || isnanf(v);
}

/* ---------------------------------------------------------------------
 *  Motor electrical model — BotWheel
 *
 *  NOTE: the observer no longer reads Iq for the motor wrench (it uses
 *  the commanded torque).  Kt still sets the torque cap and is the GUI
 *  cross-check.
 *
 *  Kt (torque constant, N·m/A): ODrive computes it as 8.27 / KV and
 *  stores it in  axis0.config.motor.torque_constant.
 *
 *  >>> CONFIRM THIS against your ODrive GUI (Motor -> torque_constant)
 *      before trusting the observer's force magnitude. <<<
 *  The value below is a BotWheel placeholder — set it to whatever your
 *  ODrive actually reports.
 * ---------------------------------------------------------------------*/
/* ODrive BotWheel: KV = 8.7 RPM/V  ->  Kt = 8.27 / 8.7 = 0.951 N·m/A.
 * The firmware's Kt MUST equal the ODrive's config.motor.torque_constant
 * (the observer's motor force = Kt * Iq_measured must use the same constant
 * the ODrive uses to convert torque<->current).  >>> CONFIRM in the GUI. <<< */
#define MOTOR_KT_NM_PER_A           0.951f      /* BotWheel nominal — verify against ODrive config */

/* ---------------------------------------------------------------------
 *  Trolley mechanical — MEASURE on your cart (TODO until then -> fallback)
 * ---------------------------------------------------------------------*/
/* Values seeded from State_Observer_Control (bench-fit on the MakerBotics
 * 6.5" cart, 2026-05-21).  "Somewhat accurate" starting points — re-measure
 * for the BotWheel cart, but far better than the fallbacks. */
#define TROLLEY_MASS_EMPTY_KG       15.0f       /* total empty mass        */
#define TROLLEY_WHEELBASE_M         0.35f       /* L: track width L<->R     */
#define WHEEL_INERTIA_KGM2          0.007f      /* per wheel about its axle */
#define YAW_INERTIA_KGM2            0.7f        /* about vertical axis      */

/* ---------------------------------------------------------------------
 *  Per-wheel friction model: tau_fric(w) = b_v*w + t_c*sign(w)  [N·m]
 *  Values from State_Observer_Control (Coulomb-dominated, ~0.42 N·m/wheel
 *  -> ~10 N chassis friction at speed).  These describe the REAL cart's
 *  friction; on a FREE propped bench wheel that friction isn't present, so
 *  friction-comp is a runtime toggle (OB F) that DEFAULTS OFF.  Turn it ON
 *  only on the loaded cart, or the observer reads phantom forward force.
 * ---------------------------------------------------------------------*/
#define FRICTION_VISCOUS_NM_PER_RADS    0.013f
#define FRICTION_COULOMB_NM             0.42f

/* ---------------------------------------------------------------------
 *  Generalised Momentum Observer (GMO)
 *
 *  Rate: the STM32 reads the IMU over blocking I2C (~1.5 ms/burst), so a
 *  100 Hz outer observer is comfortably sustainable (human-force
 *  bandwidth is < 10 Hz, so 100 Hz sampling is plenty).  K_o is the
 *  channel cutoff in Hz; forward-Euler stability wants K_o*2π*dt < 2.
 *    trans: 2π*8*0.01  = 0.50   yaw: 2π*6*0.01 = 0.38   -> well-damped.
 * ---------------------------------------------------------------------*/
#define OBSERVER_RATE_HZ            100.0f
#define OBSERVER_DT_S               (1.0f / OBSERVER_RATE_HZ)
#define OBSERVER_K_O_TRANS_HZ       8.0f
#define OBSERVER_K_O_YAW_HZ         6.0f
#define CURRENT_LPF_ALPHA           0.20f   /* Iq LPF per tick — reserved; the Iq feedback path is currently disabled (observer uses commanded torque) */
#define OBSERVER_OUT_LPF_ALPHA      0.10f   /* tau_ext output smoothing  */

/* Pitch low-pass for the gravity feed-forward.  Raw imuGetPitchDeg() reads
 * linear (push) acceleration as false tilt; a heavy LPF rejects that
 * vibration while real slope (which changes over ~1 s) passes through.
 *   alpha=0.05 @ 100 Hz -> tau ~ 190 ms. */
#define PITCH_LPF_ALPHA             0.05f

/* ---------------------------------------------------------------------
 *  Slope estimator (feeds the gravity compensation)
 *
 *  The frame IMU's TILT can't separate "handle held higher" from "on a
 *  ramp" — both rotate the frame against gravity identically.  That made
 *  the old tilt-based gravity comp read handle angle as slope (phantom
 *  backward push on flat floor).  The slope is instead estimated from
 *  ACCELERATION: when the cart speeds up / slows down along the road, the
 *  vertical part of that acceleration is a_wheel*sin(slope).  So
 *      sin(slope) = (f_vertical - f_vertical_at_rest) * g / a_wheel
 *  where f_vertical is the IMU specific force projected on true vertical
 *  (frame tilt tracked by a gyro+accel complementary filter) and a_wheel is
 *  d/dt of the wheel speed.  Handle angle drops out: it changes the frame
 *  tilt, which the filter tracks, but not the vertical acceleration.
 *  Updates only while the cart is accelerating and not being tilted, and
 *  HOLDS between pushes (reset to 0 on each drive entry).
 * --------------------------------------------------------------------- */
#define SLOPE_LPF_ALPHA             0.10f   /* LPF on f_vertical and a_wheel (~95 ms @ 100 Hz) */
#define SLOPE_MIN_ACCEL_MPS2        0.25f   /* need at least this |a_wheel| to update the slope */
#define SLOPE_MAX_PITCH_RATE_DPS    5.0f    /* ...and the frame not swinging faster than this
                                               (10 let frame rocking at direction reversals
                                               leak in as +/-2-3 deg - flat-floor test 2026-11-10) */
#define SLOPE_MIN_SPEED_MPS         0.05f   /* ...and actually rolling                          */
#define SLOPE_TAU_S                 0.6f    /* slope estimate time constant while updating      */
#define SLOPE_MAX_DEG               12.0f   /* clamp: steeper than this is not a real ramp here */
#define SLOPE_TILT_CF_K             0.02f   /* accel weight in the frame-tilt filter (~0.5 s)   */
#define SLOPE_CF_MAX_ACCEL_MPS2     0.08f   /* trust accel tilt only when |a_wheel| below this  */

/* Gravity comp deadband on the ESTIMATED slope (estimator noise floor).
 * Inside +/- this the floor is treated as flat; beyond it only the excess
 * counts (soft, no step).  Raise if a flat floor still shows small phantom
 * push/pull; lower if gentle ramps get no help. */
#define GRAVITY_SLOPE_DEADBAND_DEG  2.5f    /* flat-floor test 2026-11-10: estimate stayed within
                                               ~+/-1 deg in steady pushes but hit +/-2-3 deg at
                                               direction reversals */

/* 1 = turn gravity comp ON automatically at each drive entry / assist-on
 * (old behaviour).  0 = leave it OFF unless toggled with `OB G`.  Keep 0
 * until `OB S` shows slope ~0 on flat floor and ~the ramp angle on a ramp. */
#define GRAVITY_COMP_AUTO_ENABLE    1       /* ON since 2026-11-10: slope-based comp passed the
                                               flat-floor test (no phantom push); ramp test pending */
#define STARTUP_OBSERVER_WARMUP_MS  200U    /* observer settling window before its force output is trusted (converted to a tick count at OBSERVER_RATE_HZ) */

/* Defensive clamps on raw inputs / outputs (much wider than any real value). */
#define V_TRANS_SANITY_CLAMP_MPS    5.0f
#define V_YAW_SANITY_CLAMP_RADS     10.0f
#define TAU_EXT_SANITY_CLAMP_N      200.0f

/* =====================================================================
 *  ##############  POSE DETECTION (STANDING / DRIVING) — TUNE HERE  ####
 *
 *  Pose is decided from the IMU pitch = imuGetPitchDeg() (currently
 *  atan2(ay, az)) in degrees — SIGNED tilt from gravity (no calibration,
 *  robust at boot).  On this cart's mounting the angle is MONOTONIC as you
 *  tilt: DRIVING (onto the wheels) reads ~+5 deg and STANDING (upright)
 *  ~-43 deg.  DRIVING is the HIGHER (more positive) pitch, STANDING the
 *  LOWER (more negative).  Two thresholds with a gap between them give
 *  hysteresis.
 *
 *  Read your real values live with `M S`, then set the thresholds between
 *  them.  Tuning:
 *    - DRIVING activating "too late"?  LOWER POSE_DRIVE_ENTER_DEG.
 *    - STANDING activating "too late"? RAISE POSE_STANDING_ENTER_DEG.
 *    - Keep POSE_DRIVE_ENTER_DEG > POSE_STANDING_ENTER_DEG (the gap is the
 *      hysteresis band — several degrees stops flicker).
 *    - If the IMU is remounted, find the new tilt axis with `I S` and change
 *      imuGetPitchDeg() / imuGetPitchRateDps() in imu.c, then retune these.
 * =====================================================================*/
#define POSE_DRIVE_ENTER_DEG         -5.0f   /* pitch ABOVE this -> DRIVING (driving ~+5)   */
#define POSE_STANDING_ENTER_DEG     -15.0f   /* pitch BELOW this -> STANDING (standing ~-43)*/
#define POSE_DEBOUNCE_MS            200U     /* hold a new pose this long before committing */

/* Anti-rollback: on a STANDING -> DRIVING transition, keep the motors
 * BRAKED (actively held at 0) for this long before releasing to the
 * assist — stops the cart rolling back as you tilt it onto the wheels.
 * Raise for a longer safety hold; lower for a snappier handoff. */
#define POSE_DRIVE_BRAKE_DELAY_MS   1000U

/* Tilt protection: while the cart is being TILTED (e.g. lifting it to the
 * standing position), the wheels are held braked and the assist is
 * suppressed, so they can't spin up and draw current (which was tripping
 * the supply's undervoltage and slamming the brake).  "Being tilted" =
 * the pitch-axis gyro rate exceeds POSE_TILT_RATE_DPS; the hold then
 * persists POSE_TILT_SETTLE_MS after the rotation stops so the cart can
 * settle before the assist is allowed back.
 *   - Wheels still creep while tilting -> LOWER POSE_TILT_RATE_DPS.
 *   - Assist cuts out over bumps on rough ground -> RAISE it. */
#define POSE_TILT_RATE_DPS          15.0f   /* gyro pitch rate that counts as "tilting" */
#define POSE_TILT_SETTLE_MS         250U    /* keep holding this long after the tilt stops */

/* Autonomous mode (R A): once the cart has settled in DRIVING (committed,
 * not tilting), wait this long before auto-running the calibration
 * sequence (clear errors, tare, zero pitch, enable gravity+friction comp)
 * and arming the assist — so the IMU/force are steady when captured. */
#define AUTOSEQ_SETTLE_MS           400U

/* Handle-release stop (dead-man switch, deadman.c): when the switch is
 * released while the wheels are driven, both wheels go to velocity control
 * at their CURRENT speed, which is then ramped to 0 at this rate and held.
 *   1.5 rev/s^2 at the motor ~ 0.78 m/s^2 with the 6.5" wheel:
 *   from 1 m/s (~1.9 rev/s) to a stop in ~1.3 s.
 *   - Stop feels too abrupt / regen trips the bus -> LOWER it.
 *   - Rolls on too long after letting go          -> RAISE it. */
#define DEADMAN_DECEL_REV_PER_S2    1.5f

/* =====================================================================
 *  ##############  POWER-ASSIST TORQUE LOOP — TUNE HERE  ##############
 *
 *  This is the live assist controller (admittance.c).  It reads the
 *  operator force estimate Fext (N) from the observer and commands motor
 *  torque so the truck helps you push.  Start GENTLE and raise gains as
 *  you gain confidence.  Each knob, what it does, and how to tune:
 * =====================================================================*/

/* How often the assist loop runs. */
#define ASSIST_RATE_HZ              100.0f

/* === Direct-torque force amplification ===
 * The motor applies a force proportional to your push:
 *       F_motor = GAIN * (your push beyond the deadband)
 * distributed to per-wheel torque.  The ODrive applies this torque
 * DIRECTLY (no velocity loop), so you feel it immediately, with or
 * without a load.  No continuous damper -> no "fighting" while you push;
 * the only braking is the overspeed safety below. */

/* PRIMARY KNOB — assist strength.  Motor force = GAIN * your push.
 * Current value 1.2: the truck adds ~1.2x your push.
 * Crank it up to FEEL it, then tune down.  This is the main dial. */
#define ASSIST_GAIN                 1.2f

/* Push force (N) needed to engage — rejects the observer's resting noise.
 * Raise if the wheel creeps/twitches at rest; lower for a hair trigger. */
#define ASSIST_DEADBAND_N           7.0f

/* Command smoothing (per tick).  Low-passes the commanded force so the
 * assist can't reverse fast enough to self-oscillate (the limit cycle you
 * see on a low-inertia free wheel).  LOWER = smoother/more stable but a
 * touch laggier.  alpha=0.2 @100Hz -> tau ~40 ms. */
#define ASSIST_CMD_LPF_ALPHA        0.2f 

/* Hard caps.  ASSIST_FORCE_MAX_N bounds total assist force; the per-wheel
 * ASSIST_TORQUE_CAP_NM is the real actuator limit you feel — raise it for
 * more grunt (BotWheel can take several N·m). */
#define ASSIST_FORCE_MAX_N          80.0f
#define ASSIST_TORQUE_CAP_NM        5.0f

/* Velocity rolloff: the assist fades linearly to ZERO by this speed, so
 * the motor can never drive the cart faster than ~this — the primary
 * anti-runaway bound and a natural top speed.  Lower = gentler/slower. */
#define ASSIST_V_ROLLOFF_MPS        0.7f

/* Overspeed safety: above this measured speed (m/s) the loop stops
 * assisting and brakes (-BRAKE_GAIN * v) — the hard backstop. */
#define ASSIST_V_HARD_LIMIT_MPS     1.0f
#define ASSIST_BRAKE_GAIN_NS_PER_M  20.0f

/* === Standstill guard — stops the loaded-cart FWD<->BACK chatter ===
 * The momentum observer infers operator force from MOTION (momentum change).
 * At a dead stop there is no motion, so its force estimate there is pure
 * artifact: static friction reacting the motor's OWN torque reads back as a
 * phantom force of -tau, and amplifying that (gain > 1, through the loop
 * lags) makes a loaded cart oscillate FWD<->BACK forever instead of settling.
 *
 * So below MOVE_OFF (held for the settle time) the operator-force estimate is
 * forced to zero -> the assist decays to true IDLE.  It only re-engages once
 * the cart is ACTUALLY moving (operator-initiated) above MOVE_ON.  The cart
 * helps you once it's rolling; it can't fight you when stopped.  Hysteresis
 * (MOVE_ON > MOVE_OFF) stops flicker at the threshold.
 *
 * Note: with a momentum observer there is genuinely no force signal at a dead
 * stop, so breakaway-from-stationary assist is not possible here — that needs
 * a direct force sensor (load cell).  See [[intent-sensing-approach]].
 *   - Cart feels "dead" when you start a slow push -> LOWER MOVE_ON.
 *   - Still twitches/jerks at a stop               -> RAISE MOVE_OFF or settle. */
#define ASSIST_V_MOVE_ON_MPS        0.05f   /* engage assist above this speed   */
#define ASSIST_V_MOVE_OFF_MPS       0.02f   /* below this (held) -> force IDLE   */
#define ASSIST_STANDSTILL_SETTLE_MS 150U    /* must stay below MOVE_OFF this long*/

#endif /* ASSIST_TUNING_H */
