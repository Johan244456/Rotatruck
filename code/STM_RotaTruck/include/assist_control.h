#ifndef ASSIST_CONTROL_H
#define ASSIST_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Truck pose state machine.
 *
 * Drives the pose-driven brake/release for the motors and the entry
 * point for the push-to-go / pull-to-stop assist loop.
 *
 *   STANDING : truck parked on its back rest, IMU X axis pointing up.
 *              Wheels are actively braked while the assist is on.
 *   DRIVE    : truck tilted forward onto its wheels, IMU X axis near
 *              horizontal.  Wheels are released to the assist after the
 *              anti-rollback delay. */
typedef enum {
    POSE_STANDING = 0,
    POSE_DRIVE    = 1,
} TruckPose;

/* Initialise the state machine.
 * Call once from main() after motorControlSetup(), commandSetup(),
 * and imuBegin(). */
void assistSetup(void);

/* Run one tick.  Internally paced at ASSIST_TICK_MS using HAL_GetTick().
 * Does pose detection, tilt protection, and pose-driven brake/release of
 * the motors.
 * Safe to call as often as the main loop wants. */
void assistUpdate(void);

/* Current pose. */
TruckPose   assistGetPose(void);
const char *assistGetPoseName(void);

/* True when the assist torque loop is allowed to drive the wheels:
 * the assist system is ON, the pose is DRIVING, and the anti-rollback
 * brake delay (POSE_DRIVE_BRAKE_DELAY_MS) has elapsed.  While false the
 * pose machine is holding the motors braked (STANDING / delay) or the
 * system is off.  In dev mode (no IMU) this is always true.
 * admittance.c gates on this. */
bool assistDriveActive(void);

/* Signed pitch (deg, atan2(ax,az)) the pose detector uses — stream it
 * with `M S` to find the right POSE_*_ENTER_DEG thresholds. */
float assistGetPitchDeg(void);

/* True while the cart is being tilted (wheels held braked, assist
 * suppressed) — see the tilt-protection params in assist_tuning.h. */
bool  assistIsTilting(void);

/* Always true on this branch — manual L/R/B motor commands are not
 * pose-gated; pose only gates the assist torque loop, via
 * assistDriveActive(). */
bool assistIsEnabled(void);

/* Force a re-calibration of the IMU forward-axis offset using whatever
 * pose the truck is in right now.  Reachable via the 'IC' command. */
void assistCalibrateNow(void);

/* --- Brake control ------------------------------------------------- */

/* Brake source modes.  The actual brake state is driven by the assist
 * state machine in AUTO mode (pose machine + anti-rollback delay,
 * POSE_DRIVE_BRAKE_DELAY_MS); user commands switch into FORCE_ON or
 * FORCE_OFF, which override AUTO until the next pose change. */
typedef enum {
    BRAKE_AUTO       = 0,
    BRAKE_FORCE_ON   = 1,
    BRAKE_FORCE_OFF  = 2,
} BrakeMode;

void       assistForceBrakeOn(void);
void       assistForceBrakeOff(void);
void       assistAutoBrake(void);          /* return to AUTO */
BrakeMode  assistGetBrakeMode(void);
bool       assistIsBraked(void);            /* what is actually driven */

/* No-op on this branch.  Anti-rollback is owned by the pose machine
 * (POSE_DRIVE_BRAKE_DELAY_MS), not collapsed by a torque/velocity
 * setpoint command. */
void       assistNotifyMotorCommand(void);

#ifdef __cplusplus
}
#endif

#endif /* ASSIST_CONTROL_H */
