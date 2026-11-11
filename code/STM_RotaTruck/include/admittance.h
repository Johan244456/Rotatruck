#ifndef ADMITTANCE_H
#define ADMITTANCE_H

/* =====================================================================
 *  admittance.h — power-assist torque loop (ODrive_S1 branch).
 *
 *  Direct-torque force amplification: reads the operator's push force
 *  Fext (N) from the momentum observer and commands motor torque so the
 *  truck helps you push — "push to go, pull to slow".  The ODrive runs
 *  its own FOC/current loop; we only set the per-wheel torque setpoint.
 *
 *      F_assist = ASSIST_GAIN * (operator force beyond the deadband),
 *                 faded to zero by ASSIST_V_ROLLOFF_MPS; the only braking
 *                 is the overspeed brake above ASSIST_V_HARD_LIMIT_MPS.
 *      tau_per_wheel = F_assist * r / 2   (clamped)
 *
 *  All gains live in assist_tuning.h (the POWER-ASSIST block).  This is
 *  OPT-IN and disabled at boot — enable with the `AS 1` UART command.
 * =====================================================================*/

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Init (disabled).  Call once after observerInit(). */
void  admittanceInit(void);

/* Enable / disable the assist.  Enabling arms the live ODrive axes into
 * TORQUE control and tares the observer's resting bias; disabling zeroes
 * torque and drops the axes to IDLE.  Returns false if no ODrive is alive. */
bool  admittanceEnable(bool enabled);
bool  admittanceIsEnabled(void);

/* Button helper (D8 / B1): toggle the assist AND the observer's gravity +
 * friction compensation together, so one press gives the full "assist on"
 * feel and one press the raw cart.  Compensation is matched to the ACTUAL
 * resulting assist state (stays off if the enable was refused — no ODrive). */
void  admittanceToggleWithComp(void);

/* Run the torque loop.  Self-paced at ASSIST_RATE_HZ; call every main-loop
 * iteration.  No-op while disabled. */
void  admittanceStep(void);

/* Telemetry (for the `AS` command). */
float admittanceGetAssistForce(void);   /* commanded assist force, N        */
const char *admittanceStateName(void);  /* OFF/IDLE/ASSIST FWD/BACK/BRAKING */

#ifdef __cplusplus
}
#endif

#endif /* ADMITTANCE_H */