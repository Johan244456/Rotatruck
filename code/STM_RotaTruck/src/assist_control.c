/*
 *  assist_control.c — pose detector + pose-driven motor brake/release.
 *
 *  Detects the truck pose from the IMU pitch and, when the assist system
 *  is ON (AS 1), controls the motors accordingly:
 *
 *    STANDING                 -> motors BRAKED (actively held at 0 so the
 *                                cart can't roll).
 *    STANDING -> DRIVING       -> stay BRAKED for POSE_DRIVE_BRAKE_DELAY_MS
 *                                (anti-rollback), then release.
 *    DRIVING (after delay)     -> RELEASED to TORQUE control; the assist
 *                                loop (admittance.c) drives the wheels.
 *
 *  When the system is OFF (AS 0) the motors are IDLE and the pose is only
 *  detected + printed.  All thresholds/delays are in assist_tuning.h.
 *
 *  Pose angle = atan2(ax, az) in degrees — SIGNED tilt from gravity.
 *  STANDING ~-40 deg, DRIVING ~+10 deg.
 */

#include "assist_control.h"
#include "assist_tuning.h"
#include "board.h"
#include "imu.h"
#include "motor_control.h"
#include "admittance.h"
#include "observer.h"
#include "deadman.h"
#include "command.h"
#include "stm32g4xx_hal.h"

#include <math.h>

#define ASSIST_TICK_MS          20U      /* 50 Hz pose-detection rate */

/* =====================================================================
 *  State
 * =====================================================================*/
static TruckPose pose             = POSE_STANDING;
static TruckPose pose_candidate   = POSE_STANDING;
static uint32_t  pose_candidate_t = 0;
static uint32_t  last_tick_ms     = 0;
static uint32_t  drive_entered_ms = 0;   /* when DRIVING was committed */
static bool      imu_present_at_boot = false;

/* Tilt protection: true while the cart is being rotated (held a little
 * after motion stops).  Brakes the wheels + suppresses assist so they
 * can't spin during a tilt. */
static bool      s_tilting          = false;
static uint32_t  tilt_hold_until_ms = 0;

/* Motor mode the pose machine is currently commanding (so we only issue a
 * mode change on a transition, not every tick).
 *   MM_STOP_RAMP: handle (dead-man) switch released - velocity control from
 *   the current wheel speed, ramped to 0 at DEADMAN_DECEL_REV_PER_S2, then
 *   held at 0. */
typedef enum { MM_IDLE, MM_BRAKE, MM_DRIVE, MM_STOP_RAMP } motor_mode_t;
static motor_mode_t motor_mode = MM_IDLE;

/* stop-ramp state (truck frame, rev/s) */
static float    s_ramp_vL = 0.0f, s_ramp_vR = 0.0f;
static uint32_t s_ramp_last_ms = 0;
static bool     s_ramp_done = false;

static float toward_zero(float v, float step)
{
    if (v >  step) return v - step;
    if (v < -step) return v + step;
    return 0.0f;
}

/* One tick of the handle-release ramp: walk both setpoints towards 0. */
static void ramp_step(void)
{
    uint32_t now = HAL_GetTick();
    float dt = (float)(uint32_t)(now - s_ramp_last_ms) / 1000.0f;
    s_ramp_last_ms = now;
    if (dt > 0.1f) dt = 0.1f;               /* a stalled loop must not jump the ramp */
    float dv = DEADMAN_DECEL_REV_PER_S2 * dt;
    s_ramp_vL = toward_zero(s_ramp_vL, dv);
    s_ramp_vR = toward_zero(s_ramp_vR, dv);
    setMotorLeftVelocity(s_ramp_vL);
    setMotorRightVelocity(s_ramp_vR);
    if (!s_ramp_done && s_ramp_vL == 0.0f && s_ramp_vR == 0.0f) {
        s_ramp_done = true;
        uart_print("Pose: stopped - holding at 0 (press the handle switch to continue).\r\n");
    }
}

/* =====================================================================
 *  Helpers
 * =====================================================================*/
static float current_pitch_deg(void)
{
    /* SIGNED pitch (no abs): on this mounting DRIVING reads ~+5 deg and
     * STANDING ~-43 deg, so the angle is MONOTONIC as the cart tilts.  Using
     * abs() would fold the two opposite signs and dip through 0 mid-tilt.
     * Sensor-axis mapping lives in imuGetPitchDeg(). */
    return imuGetPitchDeg();
}

static void print_pose(TruckPose p)
{
    uart_print((p == POSE_DRIVE) ? "DRIVING mode\r\n" : "STANDING mode\r\n");
}

/* Switch the ODrive motor mode.  Idempotent: only the velocity-brake path
 * re-issues each tick (to keep the held-zero setpoint fresh). */
static void set_motor_mode(motor_mode_t m)
{
    if (m == motor_mode) {
        if (m == MM_BRAKE)     setMotorBothVelocity(0.0f);   /* keep the hold fresh */
        if (m == MM_STOP_RAMP) ramp_step();                  /* ramp / hold at 0    */
        return;
    }
    const motor_mode_t prev = motor_mode;
    motor_mode = m;
    switch (m) {
    case MM_STOP_RAMP:
        /* Take over in velocity control AT the current speed, then ramp. */
        motorControlEnterVelocityHold(&s_ramp_vL, &s_ramp_vR);
        s_ramp_last_ms = HAL_GetTick();
        s_ramp_done    = false;
        uart_printf("Pose: handle released - ramping to a stop (L %+.2f, R %+.2f rev/s, "
                    "%.1f rev/s^2).\r\n", (double)s_ramp_vL, (double)s_ramp_vR,
                    (double)DEADMAN_DECEL_REV_PER_S2);
        return;
    default:
        break;
    }
    if (prev == MM_STOP_RAMP && m == MM_DRIVE) {
        /* Resuming after a handle release: restart the force estimate from
         * the present motion so the stop itself doesn't read as a push. */
        observerReseedMomentum();
    }
    switch (m) {
    case MM_IDLE:
        motorControlDisableClosedLoop();
        break;
    case MM_BRAKE:
        motorControlEnableClosedLoop();   /* velocity mode... */
        setMotorBothVelocity(0.0f);       /* ...held at 0 = active brake */
        uart_print("Pose: motors BRAKED (held).\r\n");
        break;
    case MM_DRIVE:
        motorControlEnableTorqueMode();   /* torque mode for the assist */
        uart_print("Pose: motors RELEASED (assist active).\r\n");
        break;
    case MM_STOP_RAMP:                    /* handled above */
        break;
    }
}

/* =====================================================================
 *  Public API
 * =====================================================================*/
void assistSetup(void)
{
    imu_present_at_boot = imuIsPresent();
    pose             = POSE_STANDING;
    pose_candidate   = POSE_STANDING;
    pose_candidate_t = HAL_GetTick();
    last_tick_ms     = HAL_GetTick();
    drive_entered_ms = 0;
    motor_mode       = MM_IDLE;
    motorControlDisableClosedLoop();

    if (imu_present_at_boot) {
        print_pose(pose);                 /* announce starting state */
    } else {
        uart_print("Pose: no IMU - detection disabled (dev mode).\r\n");
    }
}

void assistUpdate(void)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_tick_ms) < ASSIST_TICK_MS) return;
    last_tick_ms = now;

    if (!imu_present_at_boot && imuIsPresent()) {
        /* The IMU came up after boot (imu.c retries its setup): leave dev
         * mode and use normal pose control from now on. */
        imu_present_at_boot = true;
        pose = pose_candidate = POSE_STANDING;
        pose_candidate_t = now;
        uart_print("Pose: IMU now available - pose control enabled.\r\n");
        print_pose(pose);
    }
    if (!imu_present_at_boot) {
        /* Dev mode (no IMU): no pose control, but the handle switch still
         * applies while the assist is on - released = ramp to a stop. */
        if (admittanceIsEnabled()) set_motor_mode(deadmanPressed() ? MM_DRIVE : MM_STOP_RAMP);
        else                       motor_mode = MM_IDLE;  /* admittanceEnable(false) idled the axes */
        return;
    }

    /* --- Pose detection (IMU refreshed by observerUpdate; read cache) --- */
    float pitch = current_pitch_deg();

    TruckPose detected;
    if (pitch > POSE_DRIVE_ENTER_DEG) {           /* higher pitch -> DRIVING  */
        detected = POSE_DRIVE;
    } else if (pitch < POSE_STANDING_ENTER_DEG) { /* lower pitch  -> STANDING */
        detected = POSE_STANDING;
    } else {
        detected = pose;                  /* hysteresis band -> hold */
    }

    if (detected != pose) {
        if (detected != pose_candidate) {
            pose_candidate   = detected;
            pose_candidate_t = now;
        } else if ((uint32_t)(now - pose_candidate_t) >= POSE_DEBOUNCE_MS) {
            pose = detected;
            if (pose == POSE_DRIVE) drive_entered_ms = now;
            print_pose(pose);
        }
    } else {
        pose_candidate   = pose;
        pose_candidate_t = now;
    }

    /* --- Tilt protection: pitch-axis gyro rate (clean, instant) --- */
    if (fabsf(imuGetPitchRateDps()) > POSE_TILT_RATE_DPS) {
        tilt_hold_until_ms = now + POSE_TILT_SETTLE_MS;
    }
    s_tilting = (int32_t)(tilt_hold_until_ms - now) > 0;

    /* --- Pose-driven motor control (only when the assist system is ON) --- */
    if (!admittanceIsEnabled()) {
        set_motor_mode(MM_IDLE);
        return;
    }

    /* Brake (hold the wheels at 0) when STANDING, while being TILTED, or
     * during the post-STANDING anti-rollback delay.  Only release to the
     * assist when settled in DRIVING. */
    bool brake = (pose == POSE_STANDING) || s_tilting;
    if (!brake && pose == POSE_DRIVE) {
        uint32_t elapsed = (uint32_t)(now - drive_entered_ms);
        if (elapsed < POSE_DRIVE_BRAKE_DELAY_MS) brake = true;  /* anti-rollback */
    }
    motor_mode_t want = brake ? MM_BRAKE : MM_DRIVE;

    /* Handle (dead-man) switch released: whatever is being driven ramps down
     * to 0 and is held there.  Already braked (STANDING / tilt / anti-
     * rollback) stays braked - it is not moving.  Pressing the switch again
     * hands back to the normal pose logic (assist resumes in DRIVE). */
    if (!deadmanPressed()) {
        want = (motor_mode == MM_BRAKE) ? MM_BRAKE : MM_STOP_RAMP;
    }
    set_motor_mode(want);
}

/* True when the assist loop is allowed to drive: system ON, handle switch
 * pressed, pose DRIVING, and the anti-rollback brake delay has elapsed.  In
 * dev mode (no IMU) only the handle switch gates it. */
bool assistDriveActive(void)
{
    if (!deadmanPressed())          return false;   /* operator not holding the handle */
    if (motor_mode == MM_STOP_RAMP) return false;   /* still finishing a release stop  */
    if (!imu_present_at_boot)       return true;
    if (!admittanceIsEnabled())     return false;
    if (s_tilting)                  return false;   /* held while being tilted */
    if (pose != POSE_DRIVE)         return false;
    return (uint32_t)(HAL_GetTick() - drive_entered_ms) >= POSE_DRIVE_BRAKE_DELAY_MS;
}

bool assistIsTilting(void) { return s_tilting; }

TruckPose   assistGetPose(void)     { return pose; }
const char *assistGetPoseName(void) { return (pose == POSE_DRIVE) ? "DRIVING" : "STANDING"; }

/* Signed pitch (deg) the pose detector uses — for finding thresholds. */
float assistGetPitchDeg(void)       { return imuIsPresent() ? current_pitch_deg() : 0.0f; }

/* Manual UART motor commands (L/R/B) are not gated by pose. */
bool assistIsEnabled(void) { return true; }

/* `IC` command — recompute the IMU forward-axis gravity offset. */
void assistCalibrateNow(void) { imuCalibrate(60); }

/* =====================================================================
 *  Legacy brake API (K / M UART commands).  The pose machine owns the
 *  motors when the assist is ON; these stay for manual override + status.
 * =====================================================================*/
static BrakeMode brake_mode = BRAKE_AUTO;

void assistForceBrakeOn(void)
{
    brake_mode = BRAKE_FORCE_ON;
    set_motor_mode(MM_BRAKE);
}

void assistForceBrakeOff(void)
{
    brake_mode = BRAKE_FORCE_OFF;
    set_motor_mode(MM_DRIVE);
}

void assistAutoBrake(void) { brake_mode = BRAKE_AUTO; }

BrakeMode assistGetBrakeMode(void) { return brake_mode; }
bool      assistIsBraked(void)     { return motor_mode == MM_BRAKE; }

/* Anti-rollback handled by the pose machine — nothing to notify. */
void assistNotifyMotorCommand(void) { }