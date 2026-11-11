#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 *  motor_control.h — thin wrapper over odrive_can for the UART command
 *  layer.  This module is what the rest of the firmware calls when it
 *  wants a wheel to move.  Under the hood every operation becomes a
 *  CAN frame to one of the ODrive S1 controllers.
 *
 *  Sign convention:
 *    +velocity = truck-forward (handle pulled forward by the operator).
 *    +torque   = same, when used directly.
 *  If a wheel turns the wrong way on first bring-up, flip its node-ID
 *  sign convention in the ODrive Web GUI (Motor → spin direction)
 *  rather than negating here.
 * =====================================================================*/

/* Init: bring up CAN2 and enable the ODrive RX path.  Does NOT take
 * the motors out of IDLE — call motorControlEnableClosedLoop() once you
 * actually want them to move. */
void motorControlSetup(void);

/* Switch both ODrives to / from CLOSED_LOOP_CONTROL.  When entering
 * closed-loop, also sets the controller mode to VELOCITY/PASSTHROUGH
 * which is the default mode used by the UART velocity commands.
 * Returns true if at least one axis is alive and armed; false only when
 * neither ODrive is responding. */
bool motorControlEnableClosedLoop(void);
void motorControlDisableClosedLoop(void);

/* Arm the live axes into TORQUE control (passthrough) at zero torque.
 * Required before setMotorLeftTorque()/setMotorRightTorque() have effect
 * (Set_Input_Torque is ignored unless the ODrive is in TORQUE_CONTROL).
 * Returns true if at least one axis was alive and armed. */
bool motorControlEnableTorqueMode(void);

/* Switch the live axes to VELOCITY control holding the speed they have
 * right now (no jolt), for a controlled ramp-down.  Returns those speeds in
 * the truck frame (rev/s, + = forward) so the caller can ramp them to 0
 * with setMotorLeftVelocity()/setMotorRightVelocity().  False if neither
 * ODrive is alive. */
bool motorControlEnterVelocityHold(float *v_left_rps, float *v_right_rps);

/* Velocity setpoint in revolutions per second at the motor shaft.
 * Negative = reverse.  Both axes use the same value for the BOTH variant. */
void setMotorLeftVelocity(float rev_per_s);
void setMotorRightVelocity(float rev_per_s);
void setMotorBothVelocity(float rev_per_s);

/* Torque setpoint in N·m at the motor shaft. */
void setMotorLeftTorque(float nm);
void setMotorRightTorque(float nm);

/* Soft stop — commands zero velocity on both axes but leaves them armed
 * (closed-loop hold).  For a real de-energise, call
 * motorControlDisableClosedLoop().  Always allowed. */
void stopMotors(void);

/* Clear latched ODrive errors on both axes.  Use after a fault. */
void motorControlClearErrors(void);

/* Time-based velocity move — drive at <rev_per_s> for <seconds>, then stop.
 * Blocking; uses HAL_Delay internally.  Phase 1 carried over for bench
 * testing.  Aborts immediately if the pose state machine drops out of
 * DRIVE (see assist_control.h). */
void moveMotorLeftForTime(float rev_per_s, float seconds);
void moveMotorRightForTime(float rev_per_s, float seconds);
void moveBothForTime(float rev_per_s, float seconds);

/* Print a one-line wheel status snapshot to the UART:
 *   L pos=<turns> vel=<rps> iq=<A>  |  R pos=<turns> vel=<rps> iq=<A>
 * Pulled from the latest cached telemetry; no CAN traffic generated. */
void printMotorStatus(void);

/* Print full ODrive status: axis state, errors, bus voltage/current.
 * Used by the `OD ?` UART command. */
void printOdriveStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
