/*
 *  observer.c — Generalised Momentum Observer (GMO), ODrive_S1 branch.
 *
 *  Forward-Euler @ OBSERVER_RATE_HZ.  Per-channel scalar momentum
 *  observer with diagonal inertia.  The residual K_o*(p - p_hat)
 *  converges on any unmodelled wrench; under the model assumptions that
 *  residual IS the operator's applied force/torque.
 *
 *  Ported from State_Observer_Control with the motor-wrench source moved
 *  from a PWM/bus-current ADC to the COMMANDED per-wheel torque fed in via
 *  observerSetMotorTorque():
 *      tau_motor_per_wheel = commanded wheel torque.
 *
 *  Channels:
 *    trans : forward translation, tau_ext units = N   (a force)
 *    yaw   : rotation about vertical, tau_ext units = N·m
 */

#include "observer.h"
#include "assist_tuning.h"
#include "board.h"
#include "odrive_can.h"
#include "imu.h"
#include "stm32f4xx_hal.h"

#include <math.h>

#define DEG_TO_RAD_F    (0.017453293f)
#define G_MS2           (9.81f)
#define TWO_PI_F        (6.28318530f)

/* Conservative fallbacks when an inertia tuning constant is sentinel-TODO. */
#define M_TRANS_FALLBACK_KG     (30.0f)
#define M_YAW_FALLBACK_KGM2     (5.0f)

static inline float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

static inline float sign_f(float x)
{
    if (x >  1.0e-4f) return  1.0f;
    if (x < -1.0e-4f) return -1.0f;
    return 0.0f;
}

/* =====================================================================
 *  Pre-computed coefficients (set in observerInit)
 * =====================================================================*/
static float M_trans = 0.0f;            /* effective translational mass, kg   */
static float M_trans_base = 0.0f;       /* empty-cart effective mass, kg       */
static float M_yaw   = 0.0f;            /* effective yaw inertia, kg·m²        */
static float L_eff_m = 0.0f;            /* effective wheelbase, m              */

/* Manual payload: added to M_trans, and used to scale the friction model
 * (rolling/bearing friction grows ~ with total weight). */
static float payload_kg    = 0.0f;
static float friction_scale = 1.0f;

static float K_o_trans = 0.0f;          /* observer gain, rad/s                */
static float K_o_yaw   = 0.0f;

static float K_motor_trans_N_per_A = 0.0f;   /* Kt / r            (force/amp)  */
static float K_motor_yaw_Nm_per_A  = 0.0f;   /* Kt * L / (2r)     (torque/amp) */

static float fric_refl_trans = 0.0f;    /* 2 / r   */
static float fric_refl_yaw   = 0.0f;    /* L / r   */
static float omega_w_per_v_trans = 0.0f;/* 1 / r   */
static float omega_w_per_v_yaw   = 0.0f;/* L / (2r)*/

static bool degraded_from_tuning = false;
static bool friction_valid       = false;   /* friction constants present  */

/* =====================================================================
 *  Dynamic state
 * =====================================================================*/
/* Commanded per-wheel torque (truck-forward frame, N·m), pushed in by the
 * assist loop via observerSetMotorTorque().  We use the COMMANDED torque
 * for the motor-wrench term instead of measured Iq: the ODrive's CAN Iq is
 * unreliable here (a constant offset at rest, lag on mode-switch, and it
 * froze when the ODrive faulted), which corrupted the tare and caused
 * phantom forces.  The commanded torque is known exactly and is 0 when
 * idle. */
static float cmd_tau_L_nm = 0.0f, cmd_tau_R_nm = 0.0f;
static float p_hat_trans = 0.0f, p_hat_yaw = 0.0f;

static float tau_ext_trans = 0.0f, tau_ext_yaw = 0.0f;
static float v_trans_mps_cached = 0.0f, v_yaw_rads_cached = 0.0f;
static float tau_motor_trans_cached = 0.0f, tau_motor_yaw_cached = 0.0f;
static float pitch_eff_deg_cached = 0.0f;

static float pitch_baseline_deg = 0.0f;
static float pitch_filt_deg     = 0.0f;     /* LPF'd effective pitch */
static float last_pitch_deg     = 0.0f;
static float last_yaw_rate_dps  = 0.0f;

/* Slope estimator state (see assist_tuning.h "Slope estimator"). */
static float slope_deg_est   = 0.0f;    /* estimated road slope, + = uphill ahead */
static float tilt_hat_deg    = 0.0f;    /* frame tilt vs gravity (gyro + accel CF) */
static float gyro_bias_dps   = 0.0f;    /* pitch-rate gyro bias, learnt at rest    */
static float fv_rest_g       = 1.0f;    /* vertical specific force at rest (scale) */
static float fv_filt_g       = 1.0f;    /* LPF'd vertical specific force           */
static float a_wheel_filt    = 0.0f;    /* LPF'd wheel acceleration, m/s^2         */
static float v_trans_prev    = 0.0f;
static bool  slope_seeded    = false;

/* Refinement knobs (runtime-toggleable via the OB command).  Both default
 * OFF for free-bench testing:
 *   gravity: pitch is noise on the bench; a wrong-pose baseline saturates.
 *   friction: the cart's friction model isn't present on a free wheel, so
 *             enabling it reads phantom forward force.
 * Turn them ON (OB G / OB F) only for real-floor/loaded-cart testing. */
static bool  gravity_enabled  = false;      /* OB G toggles */
static bool  friction_runtime = false;      /* OB F toggles */
static float tare_trans = 0.0f;             /* OB T captures resting offset */
static float tare_yaw   = 0.0f;

static observer_status_t status = OBSERVER_NOT_READY;
static uint32_t tick_count    = 0U;
static uint32_t last_tick_ms  = 0U;

/* =====================================================================
 *  Friction model (chassis frame)
 * =====================================================================*/
static float wheel_friction_Nm(float omega_w)
{
    if (!friction_valid || !friction_runtime) return 0.0f;
    return (FRICTION_VISCOUS_NM_PER_RADS * omega_w
         + FRICTION_COULOMB_NM * sign_f(omega_w)) * friction_scale;
}

static float chassis_friction_trans_N(float v_trans)
{
    return wheel_friction_Nm(v_trans * omega_w_per_v_trans) * fric_refl_trans;
}

static float chassis_friction_yaw_Nm(float v_yaw)
{
    float per_wheel = wheel_friction_Nm(fabsf(v_yaw) * omega_w_per_v_yaw);
    return per_wheel * sign_f(v_yaw) * fric_refl_yaw;
}

/* =====================================================================
 *  Slope estimator — kinematic, independent of handle angle.
 *
 *  1. Track the frame tilt vs gravity with a complementary filter: gyro
 *     integration (bias-corrected) + a slow pull toward the accelerometer
 *     tilt, the latter only while the cart is NOT accelerating (otherwise
 *     the push acceleration corrupts the accel tilt).
 *  2. Project the specific force onto true vertical with that tilt:
 *        f_v = ay*sin(tilt) + az*cos(tilt)   (= 1 g at rest on any slope)
 *  3. While accelerating along the road, the vertical part of the
 *     acceleration is a_wheel*sin(slope):
 *        sin(slope) = (f_v - f_v_rest) * g / a_wheel
 *     f_v_rest is learnt at standstill, which also cancels accel scale error.
 *  Runs every observer tick, including warm-up.
 * =====================================================================*/
static void slope_estimator_update(float v_trans)
{
    const float dt = OBSERVER_DT_S;
    const float ay = imuGetAyG();
    const float az = imuGetAzG();
    const float acc_tilt_deg = atan2f(ay, az) / DEG_TO_RAD_F;

    if (!slope_seeded) {                /* first tick: start from the accel tilt */
        tilt_hat_deg = acc_tilt_deg;
        fv_rest_g    = sqrtf(ay * ay + az * az);
        fv_filt_g    = fv_rest_g;
        v_trans_prev = v_trans;
        slope_seeded = true;
        return;
    }

    /* Wheel acceleration (LPF'd — differentiating the CAN velocity is noisy). */
    const float a_wheel = (v_trans - v_trans_prev) / dt;
    v_trans_prev = v_trans;
    a_wheel_filt = SLOPE_LPF_ALPHA * a_wheel + (1.0f - SLOPE_LPF_ALPHA) * a_wheel_filt;

    const float q_raw = imuGetPitchRateDps();
    const bool  still = fabsf(v_trans) < 0.02f && fabsf(a_wheel_filt) < 0.05f
                     && fabsf(q_raw - gyro_bias_dps) < 3.0f;
    if (still) gyro_bias_dps += 0.01f * (q_raw - gyro_bias_dps);
    const float q = q_raw - gyro_bias_dps;

    /* 1. Frame tilt: gyro always; accel correction only when not accelerating. */
    tilt_hat_deg += q * dt;
    if (fabsf(a_wheel_filt) < SLOPE_CF_MAX_ACCEL_MPS2) {
        tilt_hat_deg += SLOPE_TILT_CF_K * (acc_tilt_deg - tilt_hat_deg);
    }

    /* 2. Vertical specific force. */
    const float th = tilt_hat_deg * DEG_TO_RAD_F;
    const float fv = ay * sinf(th) + az * cosf(th);
    fv_filt_g = SLOPE_LPF_ALPHA * fv + (1.0f - SLOPE_LPF_ALPHA) * fv_filt_g;
    if (still) fv_rest_g += 0.02f * (fv_filt_g - fv_rest_g);

    /* 3. Slope from the vertical share of the acceleration. */
    const float abs_a = fabsf(a_wheel_filt);
    if (abs_a > SLOPE_MIN_ACCEL_MPS2 && fabsf(q) < SLOPE_MAX_PITCH_RATE_DPS
        && fabsf(v_trans) > SLOPE_MIN_SPEED_MPS) {
        const float s_max = sinf(SLOPE_MAX_DEG * DEG_TO_RAD_F);
        float s = (fv_filt_g - fv_rest_g) * G_MS2 / a_wheel_filt;
        s = clampf(s, -s_max, s_max);
        const float meas_deg = asinf(s) / DEG_TO_RAD_F;
        /* More acceleration = better signal-to-noise = faster update. */
        float w = (dt / SLOPE_TAU_S) * clampf(abs_a / 0.5f, 0.0f, 1.0f);
        slope_deg_est += w * (meas_deg - slope_deg_est);
        slope_deg_est = clampf(slope_deg_est, -SLOPE_MAX_DEG, SLOPE_MAX_DEG);
    }
}

/* =====================================================================
 *  Input sampling — ODrive CAN cache (encoder velocity) and the IMU.
 * =====================================================================*/
static void sample_inputs(float *v_left_rads, float *v_right_rads,
                          float *pitch_deg, float *yaw_rate_dps)
{
    const odrive_state_t *L = odriveGetState(ODRIVE_NODE_ID_LEFT);
    const odrive_state_t *R = odriveGetState(ODRIVE_NODE_ID_RIGHT);

    /* Velocity: turns/s -> rad/s (x 2*pi), then per-motor direction sign. */
    *v_left_rads  = (L ? L->vel_estimate_turns_per_s : 0.0f) * TWO_PI_F * MOTOR_LEFT_DIR;
    *v_right_rads = (R ? R->vel_estimate_turns_per_s : 0.0f) * TWO_PI_F * MOTOR_RIGHT_DIR;

    /* IMU: refresh and derive pitch (slope) + yaw rate.  Hold last good
     * values on a read failure (the I2C recovery handles real lockups). */
    if (imuRead()) {
        last_pitch_deg    = imuGetPitchDeg();
        last_yaw_rate_dps = imuGetGzDps();
    }
    *pitch_deg    = last_pitch_deg - pitch_baseline_deg;   /* effective slope */
    *yaw_rate_dps = last_yaw_rate_dps;
}

/* =====================================================================
 *  Init
 * =====================================================================*/
void observerInit(void)
{
    const bool jw_todo = tuning_is_todo(WHEEL_INERTIA_KGM2);
    const bool jz_todo = tuning_is_todo(YAW_INERTIA_KGM2);
    const bool fv_todo = tuning_is_todo(FRICTION_VISCOUS_NM_PER_RADS);
    const bool fc_todo = tuning_is_todo(FRICTION_COULOMB_NM);
    const bool kt_todo = tuning_is_todo(MOTOR_KT_NM_PER_A);

    degraded_from_tuning = jw_todo || jz_todo || kt_todo;
    friction_valid       = !(fv_todo || fc_todo);

    const float r  = WHEEL_RADIUS_M;
    const float r2 = r * r;
    L_eff_m        = TROLLEY_WHEELBASE_M;

    /* Effective translational mass = chassis + reflected wheel rotors,
     * plus any manually-entered payload (W command). */
    M_trans_base = jw_todo ? M_TRANS_FALLBACK_KG
                           : (TROLLEY_MASS_EMPTY_KG + 2.0f * WHEEL_INERTIA_KGM2 / r2);
    M_trans = M_trans_base + payload_kg;
    friction_scale = (TROLLEY_MASS_EMPTY_KG + payload_kg) / TROLLEY_MASS_EMPTY_KG;

    /* Effective yaw inertia = chassis yaw + reflected wheel rotors at L/2. */
    M_yaw = (jz_todo || jw_todo) ? M_YAW_FALLBACK_KGM2
              : (YAW_INERTIA_KGM2 + (L_eff_m * L_eff_m * 0.5f) * WHEEL_INERTIA_KGM2 / r2);

    K_o_trans = TWO_PI_F * OBSERVER_K_O_TRANS_HZ;
    K_o_yaw   = TWO_PI_F * OBSERVER_K_O_YAW_HZ;

    /* Motor wrench coefficients (Iq -> chassis force/torque). */
    K_motor_trans_N_per_A = MOTOR_KT_NM_PER_A / r;
    K_motor_yaw_Nm_per_A  = (MOTOR_KT_NM_PER_A * L_eff_m) / (2.0f * r);

    fric_refl_trans     = 2.0f / r;
    fric_refl_yaw       = L_eff_m / r;
    omega_w_per_v_trans = 1.0f / r;
    omega_w_per_v_yaw   = L_eff_m / (2.0f * r);

    cmd_tau_L_nm = cmd_tau_R_nm = 0.0f;
    p_hat_trans = p_hat_yaw = 0.0f;
    tau_ext_trans = tau_ext_yaw = 0.0f;
    v_trans_mps_cached = v_yaw_rads_cached = 0.0f;
    tau_motor_trans_cached = tau_motor_yaw_cached = 0.0f;
    pitch_eff_deg_cached = 0.0f;
    tick_count = 0U;
    last_tick_ms = HAL_GetTick();

    observerZeroPitch();

    status = degraded_from_tuning ? OBSERVER_DEGRADED : OBSERVER_NOT_READY;
}

void observerZeroPitch(void)
{
    if (imuRead()) {
        pitch_baseline_deg = imuGetPitchDeg();
        last_pitch_deg     = pitch_baseline_deg;
    }
    /* Fresh slope estimate on each drive entry: re-learnt within the first
     * push (the cart has to accelerate to move anyway). */
    slope_deg_est = 0.0f;
    slope_seeded  = false;
}

float observerGetSlopeDeg(void) { return slope_deg_est; }

/* =====================================================================
 *  Tick (self-paced)
 * =====================================================================*/
void observerUpdate(void)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_tick_ms) < (uint32_t)(OBSERVER_DT_S * 1000.0f)) return;
    last_tick_ms = now;
    tick_count++;

    /* 1. Sample inputs. */
    float v_left_rads, v_right_rads, pitch_deg, yaw_rate_dps;
    sample_inputs(&v_left_rads, &v_right_rads, &pitch_deg, &yaw_rate_dps);

    /* 2. Chassis-frame velocities. */
    float v_trans = (v_left_rads + v_right_rads) * 0.5f * WHEEL_RADIUS_M;
    float v_yaw   = yaw_rate_dps * DEG_TO_RAD_F;

    v_trans = clampf(v_trans, -V_TRANS_SANITY_CLAMP_MPS, V_TRANS_SANITY_CLAMP_MPS);
    v_yaw   = clampf(v_yaw,   -V_YAW_SANITY_CLAMP_RADS,  V_YAW_SANITY_CLAMP_RADS);
    v_trans_mps_cached  = v_trans;
    v_yaw_rads_cached   = v_yaw;

    /* Road-slope estimate for the gravity feed-forward (handle-angle proof). */
    slope_estimator_update(v_trans);

    /* Low-pass the pitch so push-acceleration vibration doesn't masquerade
     * as tilt in the gravity feed-forward. */
    pitch_filt_deg = PITCH_LPF_ALPHA * pitch_deg
                   + (1.0f - PITCH_LPF_ALPHA) * pitch_filt_deg;
    pitch_eff_deg_cached = pitch_filt_deg;

    /* 3. Motor-side wrench from the COMMANDED wheel torque (chassis frame).
     *    tau_motor_trans = (tau_L + tau_R)/r ;  tau_motor_yaw = (tau_R -
     *    tau_L)*L/(2r).  Exact, lag-free, and 0 when the assist is idle. */
    const float tau_motor_trans = (cmd_tau_L_nm + cmd_tau_R_nm) / WHEEL_RADIUS_M;
    const float tau_motor_yaw   = (cmd_tau_R_nm - cmd_tau_L_nm) * L_eff_m
                                / (2.0f * WHEEL_RADIUS_M);
    tau_motor_trans_cached = tau_motor_trans;
    tau_motor_yaw_cached   = tau_motor_yaw;

    /* 4. Warmup gate — let p_hat settle onto reality before believing the
     *    residual. */
    if (tick_count < (uint32_t)(STARTUP_OBSERVER_WARMUP_MS / (OBSERVER_DT_S * 1000.0f))) {
        const float res_t = K_o_trans * (M_trans * v_trans - p_hat_trans);
        const float res_y = K_o_yaw   * (M_yaw   * v_yaw   - p_hat_yaw);
        p_hat_trans += res_t * OBSERVER_DT_S;
        p_hat_yaw   += res_y * OBSERVER_DT_S;
        tau_ext_trans = 0.0f;
        tau_ext_yaw   = 0.0f;
        status = OBSERVER_NOT_READY;
        return;
    }

    /* 5. Gravity along the travel axis, from the ESTIMATED ROAD SLOPE (not
     *    the frame tilt, which mostly reflects how high the handle is held).
     *    + slope = uphill ahead -> gravity resists forward motion.  Soft
     *    deadband covers the estimator's noise floor. */
    float slope_deg = 0.0f;
    if      (slope_deg_est >  GRAVITY_SLOPE_DEADBAND_DEG) slope_deg = slope_deg_est - GRAVITY_SLOPE_DEADBAND_DEG;
    else if (slope_deg_est < -GRAVITY_SLOPE_DEADBAND_DEG) slope_deg = slope_deg_est + GRAVITY_SLOPE_DEADBAND_DEG;
    const float gravity_N = gravity_enabled
        ? (M_trans * G_MS2 * sinf(slope_deg * DEG_TO_RAD_F))
        : 0.0f;

    /* 6. Friction (zero if disabled). */
    const float f_fric_trans = chassis_friction_trans_N(v_trans);
    const float f_fric_yaw   = chassis_friction_yaw_Nm(v_yaw);

    /* 7. GMO update (forward Euler):
     *    p_dot_hat = tau_motor - friction - gravity + K_o*(M*v - p_hat)
     *    residual  = K_o*(M*v - p_hat)  ==  estimated external wrench. */
    const float residual_t = K_o_trans * (M_trans * v_trans - p_hat_trans);
    const float residual_y = K_o_yaw   * (M_yaw   * v_yaw   - p_hat_yaw);

    const float p_dot_t = tau_motor_trans - f_fric_trans - gravity_N + residual_t;
    const float p_dot_y = tau_motor_yaw   - f_fric_yaw                + residual_y;

    p_hat_trans += p_dot_t * OBSERVER_DT_S;
    p_hat_yaw   += p_dot_y * OBSERVER_DT_S;

    /* 8. Output: sanity clamp + LPF (internal p_hat stays at full bandwidth). */
    const float clamped_t = clampf(residual_t, -TAU_EXT_SANITY_CLAMP_N, TAU_EXT_SANITY_CLAMP_N);
    const float clamped_y = clampf(residual_y, -TAU_EXT_SANITY_CLAMP_N, TAU_EXT_SANITY_CLAMP_N);
    const float a_out = OBSERVER_OUT_LPF_ALPHA;
    tau_ext_trans = a_out * clamped_t + (1.0f - a_out) * tau_ext_trans;
    tau_ext_yaw   = a_out * clamped_y + (1.0f - a_out) * tau_ext_yaw;

    status = degraded_from_tuning ? OBSERVER_DEGRADED : OBSERVER_OK;
}

/* =====================================================================
 *  Re-seed + getters
 * =====================================================================*/
void observerReseedMomentum(void)
{
    p_hat_trans   = M_trans * v_trans_mps_cached;
    p_hat_yaw     = M_yaw   * v_yaw_rads_cached;
    tau_ext_trans = 0.0f;
    tau_ext_yaw   = 0.0f;
}

/* Capture the present (resting) force estimate as a zero offset — removes
 * the steady bias from model mismatch / idle motor current.  Call with no
 * operator force applied. */
void observerTare(void)
{
    tare_trans = tau_ext_trans;
    tare_yaw   = tau_ext_yaw;
}

void observerSetGravityComp(bool en) { gravity_enabled = en; }
bool observerGravityEnabled(void)    { return gravity_enabled; }

void observerSetFrictionComp(bool en) { friction_runtime = en && friction_valid; }
bool observerFrictionEnabled(void)    { return friction_runtime; }

void observerSetMotorTorque(float tau_left_nm, float tau_right_nm)
{
    cmd_tau_L_nm = tau_left_nm;
    cmd_tau_R_nm = tau_right_nm;
}

void observerSetPayloadKg(float kg)
{
    if (kg < 0.0f) kg = 0.0f;
    payload_kg = kg;
    M_trans = M_trans_base + payload_kg;
    /* Rolling/bearing friction grows roughly with total weight. */
    friction_scale = (TROLLEY_MASS_EMPTY_KG + payload_kg) / TROLLEY_MASS_EMPTY_KG;
    /* Re-seed so the larger mass doesn't produce a one-shot residual spike. */
    p_hat_trans = M_trans * v_trans_mps_cached;
}

float observerGetPayloadKg(void)    { return payload_kg; }
float observerGetMassKg(void)       { return M_trans; }

float observerGetTauExtTrans(void) { return tau_ext_trans - tare_trans; }
float observerGetTauExtYaw(void)   { return tau_ext_yaw   - tare_yaw;   }

observer_status_t observerGetStatus(void)   { return status; }
float    observerGetVTransMps(void)         { return v_trans_mps_cached; }
float    observerGetVYawRads(void)          { return v_yaw_rads_cached; }
float    observerGetTauMotorTrans(void)     { return tau_motor_trans_cached; }
float    observerGetTauMotorYaw(void)       { return tau_motor_yaw_cached; }
float    observerGetPitchDeg(void)          { return pitch_eff_deg_cached; }
uint32_t observerGetTickCount(void)         { return tick_count; }
