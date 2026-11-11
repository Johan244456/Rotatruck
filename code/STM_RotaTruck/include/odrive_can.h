#ifndef ODRIVE_CAN_H
#define ODRIVE_CAN_H

/* =====================================================================
 *  odrive_can.h — minimal ODrive S1 CAN driver for STM32G4 FDCAN.
 *
 *  Protocol reference:
 *    https://docs.odriverobotics.com/v/latest/manual/can-protocol.html
 *
 *  CAN message ID format (11-bit standard, "Cyphal-like"):
 *      arbitration_id = (node_id << 5) | cmd_id
 *      node_id : 0 - 63   (top 6 bits)
 *      cmd_id  : 0 - 31   (bottom 5 bits)
 *
 *  This module implements the subset of commands needed for the assist
 *  application: setState, setControllerMode, setVelocity, setTorque,
 *  clearErrors, and feedback-message ingestion (heartbeat, encoder,
 *  current).  See docs/ODRIVE_INTEGRATION.md for usage notes.
 *
 *  Hardware:
 *    FDCAN1 on PA11/PA12 via external transceiver (SN65HVD230 or similar).
 *    Bit timing in board.h.  Default baud is 250 kbit/s.
 * =====================================================================*/

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------
 *  Public types — ODrive enums (taken verbatim from the ODrive headers
 *  so the values stay in sync with the controller side).
 * ---------------------------------------------------------------------*/

typedef enum {
    ODRIVE_AXIS_STATE_UNDEFINED                  = 0,
    ODRIVE_AXIS_STATE_IDLE                       = 1,
    ODRIVE_AXIS_STATE_STARTUP_SEQUENCE           = 2,
    ODRIVE_AXIS_STATE_FULL_CALIBRATION_SEQUENCE  = 3,
    ODRIVE_AXIS_STATE_MOTOR_CALIBRATION          = 4,
    /* Values match ODrive exactly, including the gap (5 =
     * SENSORLESS_CONTROL, intentionally unused here). */
    ODRIVE_AXIS_STATE_ENCODER_INDEX_SEARCH       = 6,
    ODRIVE_AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
    ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL        = 8,
    ODRIVE_AXIS_STATE_LOCKIN_SPIN                = 9,
    ODRIVE_AXIS_STATE_ENCODER_DIR_FIND           = 10,
    ODRIVE_AXIS_STATE_HOMING                     = 11,
} odrive_axis_state_t;

typedef enum {
    ODRIVE_CONTROL_MODE_VOLTAGE  = 0,
    ODRIVE_CONTROL_MODE_TORQUE   = 1,
    ODRIVE_CONTROL_MODE_VELOCITY = 2,
    ODRIVE_CONTROL_MODE_POSITION = 3,
} odrive_control_mode_t;

typedef enum {
    ODRIVE_INPUT_MODE_INACTIVE     = 0,
    ODRIVE_INPUT_MODE_PASSTHROUGH  = 1,
    ODRIVE_INPUT_MODE_VEL_RAMP     = 2,
    ODRIVE_INPUT_MODE_POS_FILTER   = 3,
    ODRIVE_INPUT_MODE_MIX_CHANNELS = 4,
    ODRIVE_INPUT_MODE_TRAP_TRAJ    = 5,
    ODRIVE_INPUT_MODE_TORQUE_RAMP  = 6,
    ODRIVE_INPUT_MODE_MIRROR       = 7,
} odrive_input_mode_t;

/* Cached telemetry per node, populated from incoming feedback frames. */
typedef struct {
    /* Heartbeat (cmd 0x01) */
    uint32_t            axis_error;          /* ODrive fault bitfield; 0 = no error */
    odrive_axis_state_t axis_state;
    uint8_t             procedure_result;
    uint8_t             trajectory_done;
    uint32_t            last_heartbeat_ms;   /* HAL_GetTick stamp */
    bool                seen;                /* false until the first heartbeat ever — distinguishes never-seen from gone-stale */

    /* Encoder estimates (cmd 0x09) */
    float               pos_estimate_turns;
    float               vel_estimate_turns_per_s;
    uint32_t            last_encoder_ms;

    /* Iq (cmd 0x14) */
    float               iq_setpoint_a;
    float               iq_measured_a;
    uint32_t            last_iq_ms;

    /* Bus voltage / current (cmd 0x17) */
    float               bus_voltage_v;
    float               bus_current_a;
    uint32_t            last_busvi_ms;
} odrive_state_t;

/* ---------------------------------------------------------------------
 *  Lifecycle
 * ---------------------------------------------------------------------*/

/* Initialise FDCAN1 and start listening for ODrive frames.
 * Call once after SystemClock_Config().  Returns true on success. */
bool  odriveCanInit(void);

/* Number of node IDs we track.  Currently 2 (left + right wheel). */
#define ODRIVE_NUM_NODES        2

/* Read the cached state for a node.  node_id must be one we tracked
 * (i.e. ODRIVE_NODE_ID_LEFT or ODRIVE_NODE_ID_RIGHT in board.h).
 * Returns NULL for unknown node_id. */
const odrive_state_t *odriveGetState(uint8_t node_id);

/* True if at least one heartbeat has been received from `node_id`
 * within the last `timeout_ms`.  Useful for liveness checks. */
bool  odriveIsAlive(uint8_t node_id, uint32_t timeout_ms);

/* ---------------------------------------------------------------------
 *  Outgoing commands — all fire-and-forget (no ACK from ODrive).
 *  Return true if the frame was queued for transmission, false on
 *  bus-busy / TX FIFO full.
 * ---------------------------------------------------------------------*/

/* Set axis state (e.g. ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL).  CMD 0x07. */
bool  odriveSetAxisState(uint8_t node_id, odrive_axis_state_t state);

/* Set both control + input mode in one frame.  CMD 0x0B. */
bool  odriveSetControllerMode(uint8_t node_id,
                              odrive_control_mode_t ctrl,
                              odrive_input_mode_t   input);

/* Velocity setpoint with optional torque feedforward.  CMD 0x0D.
 * Velocity is in turns/sec at the motor shaft. */
bool  odriveSetVelocity(uint8_t node_id, float velocity_turns_per_s,
                        float torque_feedforward_nm);

/* Torque setpoint, N·m at the motor shaft.  CMD 0x0E. */
bool  odriveSetTorque(uint8_t node_id, float torque_nm);

/* Position setpoint with optional velocity + torque feedforward.  CMD 0x0C. */
bool  odriveSetPosition(uint8_t node_id, float position_turns,
                        float velocity_ff_turns_per_s,
                        float torque_ff_nm);

/* Per-axis limits.  CMD 0x0F. */
bool  odriveSetLimits(uint8_t node_id, float vel_limit_turns_per_s,
                      float current_limit_a);

/* Clear latched errors.  CMD 0x18. */
bool  odriveClearErrors(uint8_t node_id);

/* Request bus voltage + current from the ODrive (remote-transmit).
 * CMD 0x17.  Response populates cached fields via the RX path; poll
 * `odriveGetState()` to read the values. */
bool  odriveRequestBusVI(uint8_t node_id);

/* Estop — sends the ODrive Estop command (0x02), which latches an estop
 * error — heavier than a plain IDLE and needs odriveClearErrors() to
 * recover. */
bool  odriveEstop(uint8_t node_id);

/* Convenience: stop one or both axes.  Sets velocity to 0 and switches
 * to IDLE.  Safe to call from any context. */
void  odriveStopAll(void);

/* ---------------------------------------------------------------------
 *  ISR hook — wire from stm32g4xx_it.c::FDCAN1_IT0_IRQHandler.
 * ---------------------------------------------------------------------*/
void  odriveCanRxISR(void);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_CAN_H */
