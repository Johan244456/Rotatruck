/*
 *  motor_control.c — wrapper over odrive_can.  Keeps the per-wheel
 *  function names from Phase 1 so command.c looks similar, but every
 *  call now becomes a CAN frame to an ODrive S1 instead of a PWM/F-R
 *  GPIO write to a BLD300B.
 */

#include "motor_control.h"
#include "board.h"
#include "odrive_can.h"
#include "assist_control.h"
#include "command.h"
#include "stm32f4xx_hal.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define HEARTBEAT_TIMEOUT_MS    500U

/* =====================================================================
 *  Init
 * =====================================================================*/
void motorControlSetup(void)
{
    if (!odriveCanInit()) {
        uart_print("ODrive CAN init FAILED — check transceiver 5V power + PB12/PB13 wiring.\r\n");
        return;
    }
    uart_print("ODrive CAN init OK — waiting for first heartbeat...\r\n");
}

/* =====================================================================
 *  Closed-loop arming
 * =====================================================================*/
/* Arm a single axis: velocity/passthrough mode, default limits, parked at
 * zero, then CLOSED_LOOP_CONTROL.  Caller guarantees the node is alive. */
static void arm_axis(uint8_t node_id)
{
    /* Velocity mode with simple passthrough — the safest default; the
     * higher control loops will switch this to TORQUE for admittance
     * mode later if needed. */
    odriveSetControllerMode(node_id,
                            ODRIVE_CONTROL_MODE_VELOCITY,
                            ODRIVE_INPUT_MODE_PASSTHROUGH);
    /* Sensible default limits. */
    odriveSetLimits(node_id, MAX_VELOCITY_REV_PER_S, MAX_TORQUE_NM);
    /* Park velocity at zero so entering closed-loop doesn't slam. */
    odriveSetVelocity(node_id, 0.0f, 0.0f);
    odriveSetAxisState(node_id, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
}

bool motorControlEnableClosedLoop(void)
{
    /* Liveness check first.  No point commanding state changes if the
     * ODrives aren't even talking to us.
     *
     * Bring-up note: we arm whichever axes are actually present rather
     * than requiring both.  This lets you bench-test a single wheel (the
     * docs/ODRIVE_INTEGRATION.md §8 wheel-by-wheel procedure) before the
     * second ODrive is on the bus.  On the assembled truck both will be
     * alive and both arm.  Full "fault one axis -> E-stop the other"
     * handling is still a TODO (see §10 safety list). */
    bool l_alive = odriveIsAlive(ODRIVE_NODE_ID_LEFT,  HEARTBEAT_TIMEOUT_MS);
    bool r_alive = odriveIsAlive(ODRIVE_NODE_ID_RIGHT, HEARTBEAT_TIMEOUT_MS);

    if (!l_alive && !r_alive) {
        uart_print("ODrive: no heartbeat from EITHER axis. Check power, "
                   "Node ID (0=L/1=R), CAN wiring, 250k baud.\r\n");
        return false;
    }

    if (l_alive) {
        arm_axis(ODRIVE_NODE_ID_LEFT);
    } else {
        uart_print("ODrive LEFT (node 0): no heartbeat — arming RIGHT only.\r\n");
    }
    if (r_alive) {
        arm_axis(ODRIVE_NODE_ID_RIGHT);
    } else {
        uart_print("ODrive RIGHT (node 1): no heartbeat — arming LEFT only.\r\n");
    }
    return true;
}

void motorControlDisableClosedLoop(void)
{
    odriveSetVelocity(ODRIVE_NODE_ID_LEFT,  0.0f, 0.0f);
    odriveSetVelocity(ODRIVE_NODE_ID_RIGHT, 0.0f, 0.0f);
    odriveSetAxisState(ODRIVE_NODE_ID_LEFT,  ODRIVE_AXIS_STATE_IDLE);
    odriveSetAxisState(ODRIVE_NODE_ID_RIGHT, ODRIVE_AXIS_STATE_IDLE);
}

/* Arm one axis directly into TORQUE control (passthrough), parked at zero
 * torque.  This is the prerequisite for setMotorXTorque() to actually do
 * anything — Set_Input_Torque is ignored unless the ODrive is in
 * TORQUE_CONTROL mode. */
static void arm_axis_torque(uint8_t node_id)
{
    odriveSetControllerMode(node_id,
                            ODRIVE_CONTROL_MODE_TORQUE,
                            ODRIVE_INPUT_MODE_PASSTHROUGH);
    odriveSetLimits(node_id, MAX_VELOCITY_REV_PER_S, MAX_TORQUE_NM);
    odriveSetTorque(node_id, 0.0f);
    odriveSetAxisState(node_id, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
}

bool motorControlEnableTorqueMode(void)
{
    bool l_alive = odriveIsAlive(ODRIVE_NODE_ID_LEFT,  HEARTBEAT_TIMEOUT_MS);
    bool r_alive = odriveIsAlive(ODRIVE_NODE_ID_RIGHT, HEARTBEAT_TIMEOUT_MS);
    if (!l_alive && !r_alive) {
        uart_print("Torque mode: no ODrive heartbeat on either axis.\r\n");
        return false;
    }
    if (l_alive) arm_axis_torque(ODRIVE_NODE_ID_LEFT);
    if (r_alive) arm_axis_torque(ODRIVE_NODE_ID_RIGHT);
    return true;
}

/* Switch one live axis to VELOCITY / PASSTHROUGH holding <v_raw> (ODrive
 * frame).  The setpoint is sent BEFORE the mode change so the controller
 * takes over at the speed the wheel already has - no jolt. */
static void hold_axis_velocity(uint8_t node_id, float v_raw)
{
    odriveSetVelocity(node_id, v_raw, 0.0f);
    odriveSetControllerMode(node_id,
                            ODRIVE_CONTROL_MODE_VELOCITY,
                            ODRIVE_INPUT_MODE_PASSTHROUGH);
    odriveSetLimits(node_id, MAX_VELOCITY_REV_PER_S, MAX_TORQUE_NM);
    odriveSetAxisState(node_id, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
}

bool motorControlEnterVelocityHold(float *v_left_rps, float *v_right_rps)
{
    bool l_alive = odriveIsAlive(ODRIVE_NODE_ID_LEFT,  HEARTBEAT_TIMEOUT_MS);
    bool r_alive = odriveIsAlive(ODRIVE_NODE_ID_RIGHT, HEARTBEAT_TIMEOUT_MS);
    const odrive_state_t *L = odriveGetState(ODRIVE_NODE_ID_LEFT);
    const odrive_state_t *R = odriveGetState(ODRIVE_NODE_ID_RIGHT);
    /* Current speed in the truck frame (0 for an axis that is not alive). */
    *v_left_rps  = (l_alive && L) ? L->vel_estimate_turns_per_s * MOTOR_LEFT_DIR  : 0.0f;
    *v_right_rps = (r_alive && R) ? R->vel_estimate_turns_per_s * MOTOR_RIGHT_DIR : 0.0f;
    if (l_alive) hold_axis_velocity(ODRIVE_NODE_ID_LEFT,  *v_left_rps  * MOTOR_LEFT_DIR);
    if (r_alive) hold_axis_velocity(ODRIVE_NODE_ID_RIGHT, *v_right_rps * MOTOR_RIGHT_DIR);
    return l_alive || r_alive;
}

/* =====================================================================
 *  Setpoint commands
 * =====================================================================*/
void setMotorLeftVelocity(float rev_per_s)
{
    /* Tell the assist state machine a real (nonzero) drive command went
     * out, so it keeps the pose latched in DRIVE. */
    if (rev_per_s !=  0.0f) assistNotifyMotorCommand();
    odriveSetVelocity(ODRIVE_NODE_ID_LEFT, rev_per_s * MOTOR_LEFT_DIR, 0.0f);
}

void setMotorRightVelocity(float rev_per_s)
{
    if (rev_per_s != 0.0f) assistNotifyMotorCommand();
    odriveSetVelocity(ODRIVE_NODE_ID_RIGHT, rev_per_s * MOTOR_RIGHT_DIR, 0.0f);
}

void setMotorBothVelocity(float rev_per_s)
{
    setMotorLeftVelocity(rev_per_s);
    setMotorRightVelocity(rev_per_s);
}

void setMotorLeftTorque(float nm)
{
    if (nm != 0.0f) assistNotifyMotorCommand();
    odriveSetTorque(ODRIVE_NODE_ID_LEFT, nm * MOTOR_LEFT_DIR);
}

void setMotorRightTorque(float nm)
{
    if (nm != 0.0f) assistNotifyMotorCommand();
    odriveSetTorque(ODRIVE_NODE_ID_RIGHT, nm * MOTOR_RIGHT_DIR);
}

void stopMotors(void)
{
    /* Soft stop: command zero velocity on both axes but STAY in
     * CLOSED_LOOP_CONTROL.  In velocity mode a setpoint of 0 makes the
     * motor actively hold standstill, so the wheels stop but remain armed
     * and ready to drive again — no need to re-`E 1`.
     *
     * To actually de-energise the motors (axis IDLE, wheels freewheel),
     * use `E 0` or `K 1`, which call motorControlDisableClosedLoop().
     * The pose state machine still disarms on its own via apply_brake()
     * when the truck returns to STANDING, so safety behaviour is intact. */
    setMotorBothVelocity(0.0f);
}

void motorControlClearErrors(void)
{
    odriveClearErrors(ODRIVE_NODE_ID_LEFT);
    odriveClearErrors(ODRIVE_NODE_ID_RIGHT);
    uart_print("OK: ODrive errors cleared on both axes.\r\n");
}

/* =====================================================================
 *  Blocking time-based move
 *
 *  Mirrors the Phase 1 'B T <secs> <pwm>' command shape, but the value
 *  is now in rev/s instead of PWM percent.
 * =====================================================================*/
static bool keep_running_or_abort(void)
{
    assistUpdate();
    if (!assistIsEnabled()) {
        stopMotors();
        uart_print("Move aborted: pose left DRIVE.\r\n");
        return false;
    }
    return true;
}

void moveMotorLeftForTime(float rev_per_s, float seconds)
{
    uart_printf("L vel %.2f rev/s for %.2fs\r\n",
                (double)rev_per_s, (double)seconds);
    setMotorLeftVelocity(rev_per_s);
    uint32_t dur = (uint32_t)(seconds * 1000.0f);
    uint32_t t0  = HAL_GetTick();
    while ((HAL_GetTick() - t0) < dur) {
        if (!keep_running_or_abort()) return;
        HAL_Delay(10);
    }
    setMotorLeftVelocity(0.0f);
    printMotorStatus();
}

void moveMotorRightForTime(float rev_per_s, float seconds)
{
    uart_printf("R vel %.2f rev/s for %.2fs\r\n",
                (double)rev_per_s, (double)seconds);
    setMotorRightVelocity(rev_per_s);
    uint32_t dur = (uint32_t)(seconds * 1000.0f);
    uint32_t t0  = HAL_GetTick();
    while ((HAL_GetTick() - t0) < dur) {
        if (!keep_running_or_abort()) return;
        HAL_Delay(10);
    }
    setMotorRightVelocity(0.0f);
    printMotorStatus();
}

void moveBothForTime(float rev_per_s, float seconds)
{
    uart_printf("B vel %.2f rev/s for %.2fs\r\n",
                (double)rev_per_s, (double)seconds);
    setMotorBothVelocity(rev_per_s);
    uint32_t dur = (uint32_t)(seconds * 1000.0f);
    uint32_t t0  = HAL_GetTick();
    while ((HAL_GetTick() - t0) < dur) {
        if (!keep_running_or_abort()) return;
        HAL_Delay(10);
    }
    setMotorBothVelocity(0.0f);
    printMotorStatus();
}

/* =====================================================================
 *  Status prints
 * =====================================================================*/
void printMotorStatus(void)
{
    const odrive_state_t *L = odriveGetState(ODRIVE_NODE_ID_LEFT);
    const odrive_state_t *R = odriveGetState(ODRIVE_NODE_ID_RIGHT);
    if (!L || !R) {
        uart_print("ODrive states not tracked — check board.h node IDs.\r\n");
        return;
    }
    /* Apply the per-motor direction sign so telemetry reads in the same
     * "+ = truck-forward" frame as the commands (board.h MOTOR_*_DIR). */
    uart_printf("L pos=%.3f vel=%.3f iq=%+0.2fA | R pos=%.3f vel=%.3f iq=%+0.2fA\r\n",
                (double)(L->pos_estimate_turns       * MOTOR_LEFT_DIR),
                (double)(L->vel_estimate_turns_per_s * MOTOR_LEFT_DIR),
                (double)(L->iq_measured_a            * MOTOR_LEFT_DIR),
                (double)(R->pos_estimate_turns       * MOTOR_RIGHT_DIR),
                (double)(R->vel_estimate_turns_per_s * MOTOR_RIGHT_DIR),
                (double)(R->iq_measured_a            * MOTOR_RIGHT_DIR));
}

void printOdriveStatus(void)
{
    const odrive_state_t *L = odriveGetState(ODRIVE_NODE_ID_LEFT);
    const odrive_state_t *R = odriveGetState(ODRIVE_NODE_ID_RIGHT);
    if (!L || !R) {
        uart_print("ODrive states not tracked — check board.h node IDs.\r\n");
        return;
    }
    /* Request a fresh bus-VI sample for both axes; reply lands async.
     * The reply arrives later via the RX ISR, so the Vbus/Ibus printed
     * below are from the PREVIOUS request — the first call after boot
     * shows 0. */
    odriveRequestBusVI(ODRIVE_NODE_ID_LEFT);
    odriveRequestBusVI(ODRIVE_NODE_ID_RIGHT);

    uart_printf("LEFT (node %u): state=%u proc=%u err=0x%08lx hb=%lums "
                "Vbus=%.2fV Ibus=%+0.2fA\r\n",
                (unsigned)ODRIVE_NODE_ID_LEFT,
                (unsigned)L->axis_state, (unsigned)L->procedure_result,
                (unsigned long)L->axis_error,
                (unsigned long)(HAL_GetTick() - L->last_heartbeat_ms),
                (double)L->bus_voltage_v, (double)L->bus_current_a);
    uart_printf("RIGHT (node %u): state=%u proc=%u err=0x%08lx hb=%lums "
                "Vbus=%.2fV Ibus=%+0.2fA\r\n",
                (unsigned)ODRIVE_NODE_ID_RIGHT,
                (unsigned)R->axis_state, (unsigned)R->procedure_result,
                (unsigned long)R->axis_error,
                (unsigned long)(HAL_GetTick() - R->last_heartbeat_ms),
                (double)R->bus_voltage_v, (double)R->bus_current_a);
}
