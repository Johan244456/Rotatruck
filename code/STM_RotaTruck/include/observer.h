#ifndef OBSERVER_H
#define OBSERVER_H

/* =====================================================================
 *  observer.h — Generalised Momentum Observer (GMO) for the RotaTruck,
 *  ODrive_S1 branch.
 *
 *  Estimates the EXTERNAL (operator-applied) wrench on the chassis in two
 *  generalised coordinates:
 *      trans : forward push/pull   -> tau_ext_trans in Newtons (a force)
 *      yaw   : twist of the handle -> tau_ext_yaw   in N·m
 *
 *  This is the port of the State_Observer_Control momentum observer with
 *  one change at the bottom layer: the motor wrench is computed from the
 *  COMMANDED per-wheel torque fed in via observerSetMotorTorque(), not a
 *  PWM/bus-current ADC and not measured Iq:
 *
 *      tau_motor_per_wheel = commanded wheel torque  [N·m]
 *      F_motor_trans       = (tau_L + tau_R) / r     [N]
 *      M_motor_yaw         = (tau_R - tau_L)*L/(2r)  [N·m]
 *
 *  Commanded torque is used instead of measured Iq because the ODrive's
 *  CAN Iq had a resting offset, lagged on mode-switch, and froze on fault.
 *
 *  The observer is READ-ONLY — it commands no motor torque.  It samples
 *  the ODrive encoder velocity and the IMU, and exposes the estimated
 *  operator force/torque for the admittance layer (next phase) and for the
 *  `OB` UART command (verification).
 *
 *  Sign convention (truck-forward frame, consistent with MOTOR_*_DIR):
 *      tau_ext_trans > 0  =>  operator pushing forward
 *      tau_ext_yaw   > 0  =>  operator twisting one way (verify on bench)
 * =====================================================================*/

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OBSERVER_OK = 0,
    OBSERVER_NOT_READY,   /* warmup window; outputs forced to zero        */
    OBSERVER_DEGRADED,    /* a tuning constant is sentinel-TODO -> fallback */
} observer_status_t;

/* Initialise the observer: compute coefficients from tuning + geometry,
 * zero the estimator state, capture the IMU pitch baseline (assumes the
 * truck is level at this moment).  Call once after imuBegin(). */
void observerInit(void);

/* Run the observer at OBSERVER_RATE_HZ.  Internally paced off HAL_GetTick,
 * so it is safe to call every main-loop iteration.  Samples the ODrive
 * encoder velocity cache + the IMU and advances the momentum estimate. */
void observerUpdate(void);

/* Re-capture the IMU pitch baseline (the mounting/level offset that the
 * gravity feed-forward subtracts).  Hold the truck level/at-rest first. */
void observerZeroPitch(void);

/* Capture the present (resting) force estimate as a zero offset, removing
 * the steady idle bias.  Call with no operator force applied. */
void observerTare(void);

/* Enable/disable the gravity feed-forward.  Turn OFF for flat-bench
 * testing where pitch is pure measurement noise; ON for real slopes. */
void observerSetGravityComp(bool enabled);
bool observerGravityEnabled(void);

/* Enable/disable friction compensation.  OFF for a free bench wheel (the
 * cart's friction model would read as phantom force); ON for the loaded
 * cart.  No-op if the friction constants are sentinel-TODO. */
void observerSetFrictionComp(bool enabled);
bool observerFrictionEnabled(void);

/* Re-seed the momentum estimate from the present measured momentum so the
 * residual restarts at zero — call on entry to a state that consumes
 * tau_ext (admittance layer, next phase). */
void observerReseedMomentum(void);

/* Estimated external (operator) wrench. */
float observerGetTauExtTrans(void);   /* N    */
float observerGetTauExtYaw(void);     /* N·m  */

/* Feed the observer the per-wheel torque the assist loop is COMMANDING
 * (truck-forward frame, N·m).  The observer uses this — not the ODrive's
 * measured Iq — for the motor-wrench term, because commanded torque is
 * exact, lag-free, and exactly zero when idle (measured Iq had a resting
 * offset, lagged on mode-switch, and froze on ODrive fault, all of which
 * corrupted the force estimate).  Call every assist tick; pass 0,0 when
 * the assist is not driving. */
void observerSetMotorTorque(float tau_left_nm, float tau_right_nm);

/* Manual payload entry (kg).  Added to the empty-cart mass the observer
 * uses (M_trans) and used to scale the friction model by total/empty
 * weight, so the assist adapts to the load.  0 = empty cart.  This is the
 * manual stand-in until automatic load estimation lands. */
void  observerSetPayloadKg(float kg);
float observerGetPayloadKg(void);    /* entered payload, kg            */
float observerGetMassKg(void);       /* current effective M_trans, kg  */

/* Diagnostics (for the `OB` command + future telemetry). */
observer_status_t observerGetStatus(void);
float    observerGetVTransMps(void);      /* filtered forward velocity   */
float    observerGetVYawRads(void);       /* yaw rate                    */
float    observerGetTauMotorTrans(void);  /* motor force, N              */
float    observerGetTauMotorYaw(void);    /* motor torque, N·m           */
float    observerGetPitchDeg(void);       /* frame pitch vs drive-entry baseline (handle angle + slope) */
float    observerGetSlopeDeg(void);       /* ESTIMATED road slope, + = uphill ahead (feeds gravity comp) */
uint32_t observerGetTickCount(void);

#ifdef __cplusplus
}
#endif

#endif /* OBSERVER_H */
