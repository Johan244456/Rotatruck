/*
 *  odrive_can.c — minimal STM32G4 FDCAN driver for the ODrive S1.
 *
 *  All ODrive command + feedback IDs match the protocol documented at
 *  https://docs.odriverobotics.com/v/latest/manual/can-protocol.html
 *  (as of firmware 0.6.12).
 */

#include "odrive_can.h"
#include "board.h"
#include "stm32g4xx_hal.h"

#include <string.h>

/* ---------------------------------------------------------------------
 *  ODrive command IDs (cmd_id portion of the 11-bit CAN identifier)
 * ---------------------------------------------------------------------*/
#define ODRV_CMD_HEARTBEAT              0x001
#define ODRV_CMD_ESTOP                  0x002
#define ODRV_CMD_GET_ERROR              0x003
#define ODRV_CMD_SET_AXIS_STATE         0x007
#define ODRV_CMD_GET_ENCODER_ESTIMATES  0x009
#define ODRV_CMD_SET_CONTROLLER_MODE    0x00B
#define ODRV_CMD_SET_INPUT_POS          0x00C
#define ODRV_CMD_SET_INPUT_VEL          0x00D
#define ODRV_CMD_SET_INPUT_TORQUE       0x00E
#define ODRV_CMD_SET_LIMITS             0x00F
#define ODRV_CMD_GET_IQ                 0x014
#define ODRV_CMD_GET_TEMPERATURE        0x015
#define ODRV_CMD_REBOOT                 0x016
#define ODRV_CMD_GET_BUS_VOLTAGE_CURRENT 0x017
#define ODRV_CMD_CLEAR_ERRORS           0x018

/* The RTR (remote-transmission-request) bit is the way ODrive expects
 * a "please reply with the latest value" message.  HAL FDCAN exposes
 * this via FDCAN_DATA_FRAME vs FDCAN_REMOTE_FRAME.  Only
 * odriveRequestBusVI() uses rtr=true; all setpoint commands are data
 * frames. */

/* ---------------------------------------------------------------------
 *  Module state
 * ---------------------------------------------------------------------*/
static FDCAN_HandleTypeDef hfdcan;
static odrive_state_t      s_states[ODRIVE_NUM_NODES];
static uint8_t             s_tracked_nodes[ODRIVE_NUM_NODES] = {
    ODRIVE_NODE_ID_LEFT,
    ODRIVE_NODE_ID_RIGHT,
};
static bool                s_ready = false;

extern void uart_print(const char *s);  /* command.c */

/* ---------------------------------------------------------------------
 *  Helpers
 * ---------------------------------------------------------------------*/

/* Returns array index 0..ODRIVE_NUM_NODES-1, or -1 if untracked. */
static int node_slot(uint8_t node_id)
{
    for (int i = 0; i < ODRIVE_NUM_NODES; i++) {
        if (s_tracked_nodes[i] == node_id) return i;
    }
    return -1;
}

static uint32_t build_can_id(uint8_t node_id, uint16_t cmd_id)
{
    return ((uint32_t)node_id << 5) | (cmd_id & 0x1F);
}

static void split_can_id(uint32_t can_id, uint8_t *node_id, uint16_t *cmd_id)
{
    *node_id = (uint8_t)((can_id >> 5) & 0x3F);
    *cmd_id  = (uint16_t)(can_id & 0x1F);
}

/* Encode a little-endian float into the payload buffer. */
static void pack_f32(uint8_t *p, float v)
{
    union { float f; uint32_t u; } x;
    x.f = v;
    p[0] = (uint8_t)(x.u & 0xFF);
    p[1] = (uint8_t)((x.u >> 8) & 0xFF);
    p[2] = (uint8_t)((x.u >> 16) & 0xFF);
    p[3] = (uint8_t)((x.u >> 24) & 0xFF);
}

static float unpack_f32(const uint8_t *p)
{
    union { float f; uint32_t u; } x;
    x.u = (uint32_t)p[0]
        | ((uint32_t)p[1] << 8)
        | ((uint32_t)p[2] << 16)
        | ((uint32_t)p[3] << 24);
    return x.f;
}

static void pack_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static uint32_t unpack_u32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

/* Low-level send.  data may be NULL when len == 0.  Returns true if
 * the frame was queued for TX. */
static bool can_send(uint8_t node_id, uint16_t cmd_id,
                     const uint8_t *data, uint8_t len, bool rtr)
{
    if (!s_ready) return false;
    if (len > 8) return false;

    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier          = build_can_id(node_id, cmd_id);
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    /* HAL encodes data length as FDCAN_DLC_BYTES_n constants, not a raw
     * byte count, so map explicitly. */
    switch (len) {
        case 0: tx.DataLength = FDCAN_DLC_BYTES_0; break;
        case 1: tx.DataLength = FDCAN_DLC_BYTES_1; break;
        case 2: tx.DataLength = FDCAN_DLC_BYTES_2; break;
        case 3: tx.DataLength = FDCAN_DLC_BYTES_3; break;
        case 4: tx.DataLength = FDCAN_DLC_BYTES_4; break;
        case 5: tx.DataLength = FDCAN_DLC_BYTES_5; break;
        case 6: tx.DataLength = FDCAN_DLC_BYTES_6; break;
        case 7: tx.DataLength = FDCAN_DLC_BYTES_7; break;
        case 8: tx.DataLength = FDCAN_DLC_BYTES_8; break;
    }
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    /* Classic CAN frames (not CAN-FD) — the ODrive S1 speaks classic CAN. */
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;

    uint8_t payload[8] = {0};
    if (data && len > 0) {
        memcpy(payload, data, len);
    }

    /* The STM32G4 FDCAN TX FIFO is only 3 elements deep.  Several call
     * sites (e.g. the arm sequence) queue 4+ frames back-to-back in a few
     * microseconds, far faster than the ~0.43 ms it takes to transmit one
     * at 250 kbit/s.  Without this wait the FIFO fills and the 4th frame —
     * which happens to be Set_Axis_State(CLOSED_LOOP) — is silently
     * dropped, leaving the axis stuck in IDLE with no error.  Block
     * briefly for a free slot.  On a healthy bus a slot frees within a
     * frame time; the 10 ms cap just stops us hanging if the bus is dead. */
    uint32_t t0 = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan) == 0) {
        if ((uint32_t)(HAL_GetTick() - t0) > 10U) {
            return false;   /* bus stuck / not being ACKed */
        }
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &tx, payload) != HAL_OK) {
        /* TX FIFO full or peripheral fault — caller can retry. */
        return false;
    }
    return true;
}

/* ---------------------------------------------------------------------
 *  Init
 * ---------------------------------------------------------------------*/

bool odriveCanInit(void)
{
    memset(s_states, 0, sizeof s_states);

    hfdcan.Instance                  = ODRIVE_CAN_INSTANCE;
    hfdcan.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    hfdcan.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
    hfdcan.Init.Mode                 = FDCAN_MODE_NORMAL;
    hfdcan.Init.AutoRetransmission   = ENABLE;
    hfdcan.Init.TransmitPause        = DISABLE;
    hfdcan.Init.ProtocolException    = DISABLE;
    hfdcan.Init.NominalPrescaler     = ODRIVE_CAN_NOMINAL_PRESCALER;
    hfdcan.Init.NominalSyncJumpWidth = ODRIVE_CAN_NOMINAL_SJW;
    hfdcan.Init.NominalTimeSeg1      = ODRIVE_CAN_NOMINAL_TIME_SEG1;
    hfdcan.Init.NominalTimeSeg2      = ODRIVE_CAN_NOMINAL_TIME_SEG2;
    /* Data-phase fields don't matter in CLASSIC frame format but HAL
     * still wants them initialised to legal values. */
    hfdcan.Init.DataPrescaler        = ODRIVE_CAN_NOMINAL_PRESCALER;
    hfdcan.Init.DataSyncJumpWidth    = ODRIVE_CAN_NOMINAL_SJW;
    hfdcan.Init.DataTimeSeg1         = ODRIVE_CAN_NOMINAL_TIME_SEG1;
    hfdcan.Init.DataTimeSeg2         = ODRIVE_CAN_NOMINAL_TIME_SEG2;
    hfdcan.Init.StdFiltersNbr        = 1;
    hfdcan.Init.ExtFiltersNbr        = 0;
    hfdcan.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&hfdcan) != HAL_OK) return false;

    /* Accept every standard-frame ID into RX FIFO 0.  We'll filter by
     * node_id in software since both ODrives share the same bus. */
    FDCAN_FilterTypeDef f = {0};
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = 0x000;
    f.FilterID2    = 0x000;            /* mask = 0 → accept-all */
    HAL_FDCAN_ConfigFilter(&hfdcan, &f);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan,
                                 FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    if (HAL_FDCAN_ActivateNotification(&hfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        return false;
    }
    if (HAL_FDCAN_Start(&hfdcan) != HAL_OK) return false;

    s_ready = true;
    return true;
}

/* ---------------------------------------------------------------------
 *  Status queries
 * ---------------------------------------------------------------------*/

const odrive_state_t *odriveGetState(uint8_t node_id)
{
    int slot = node_slot(node_id);
    if (slot < 0) return NULL;
    return &s_states[slot];
}

bool odriveIsAlive(uint8_t node_id, uint32_t timeout_ms)
{
    const odrive_state_t *s = odriveGetState(node_id);
    if (!s || !s->seen) return false;
    return (HAL_GetTick() - s->last_heartbeat_ms) < timeout_ms;
}

/* ---------------------------------------------------------------------
 *  Outgoing commands
 * ---------------------------------------------------------------------*/

bool odriveSetAxisState(uint8_t node_id, odrive_axis_state_t state)
{
    uint8_t payload[4];
    pack_u32(payload, (uint32_t)state);
    return can_send(node_id, ODRV_CMD_SET_AXIS_STATE, payload, 4, false);
}

bool odriveSetControllerMode(uint8_t node_id,
                             odrive_control_mode_t ctrl,
                             odrive_input_mode_t   input)
{
    uint8_t payload[8];
    pack_u32(payload, (uint32_t)ctrl);
    pack_u32(payload + 4, (uint32_t)input);
    return can_send(node_id, ODRV_CMD_SET_CONTROLLER_MODE, payload, 8, false);
}

bool odriveSetVelocity(uint8_t node_id, float velocity_turns_per_s,
                       float torque_feedforward_nm)
{
    uint8_t payload[8];
    pack_f32(payload, velocity_turns_per_s);
    pack_f32(payload + 4, torque_feedforward_nm);
    return can_send(node_id, ODRV_CMD_SET_INPUT_VEL, payload, 8, false);
}

bool odriveSetTorque(uint8_t node_id, float torque_nm)
{
    uint8_t payload[4];
    pack_f32(payload, torque_nm);
    return can_send(node_id, ODRV_CMD_SET_INPUT_TORQUE, payload, 4, false);
}

bool odriveSetPosition(uint8_t node_id, float position_turns,
                       float velocity_ff_turns_per_s, float torque_ff_nm)
{
    /* Encoded as [pos:f32][vel_ff:i16 0.001 turns/s][tau_ff:i16 0.001 N·m]
     * per ODrive 0.6.x protocol.  We approximate by truncating to int16. */
    uint8_t payload[8];
    pack_f32(payload, position_turns);
    int16_t vel_ff = (int16_t)(velocity_ff_turns_per_s * 1000.0f);
    int16_t tau_ff = (int16_t)(torque_ff_nm * 1000.0f);
    payload[4] = (uint8_t)(vel_ff & 0xFF);
    payload[5] = (uint8_t)((vel_ff >> 8) & 0xFF);
    payload[6] = (uint8_t)(tau_ff & 0xFF);
    payload[7] = (uint8_t)((tau_ff >> 8) & 0xFF);
    return can_send(node_id, ODRV_CMD_SET_INPUT_POS, payload, 8, false);
}

bool odriveSetLimits(uint8_t node_id, float vel_limit_turns_per_s,
                     float current_limit_a)
{
    uint8_t payload[8];
    pack_f32(payload, vel_limit_turns_per_s);
    pack_f32(payload + 4, current_limit_a);
    return can_send(node_id, ODRV_CMD_SET_LIMITS, payload, 8, false);
}

bool odriveClearErrors(uint8_t node_id)
{
    /* Per protocol the payload contains an identify-bool (0).  Send as
     * a single zero byte. */
    uint8_t payload[1] = {0};
    return can_send(node_id, ODRV_CMD_CLEAR_ERRORS, payload, 1, false);
}

bool odriveRequestBusVI(uint8_t node_id)
{
    /* Remote frame — ODrive replies with the same cmd_id carrying data. */
    return can_send(node_id, ODRV_CMD_GET_BUS_VOLTAGE_CURRENT, NULL, 0, true);
}

bool odriveEstop(uint8_t node_id)
{
    return can_send(node_id, ODRV_CMD_ESTOP, NULL, 0, false);
}

void odriveStopAll(void)
{
    for (int i = 0; i < ODRIVE_NUM_NODES; i++) {
        uint8_t node = s_tracked_nodes[i];
        odriveSetVelocity(node, 0.0f, 0.0f);
        odriveSetAxisState(node, ODRIVE_AXIS_STATE_IDLE);
    }
}

/* ---------------------------------------------------------------------
 *  RX path — called from the FDCAN1 interrupt
 * ---------------------------------------------------------------------*/

static void handle_rx(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    uint8_t  node_id;
    uint16_t cmd_id;
    split_can_id(can_id, &node_id, &cmd_id);

    int slot = node_slot(node_id);
    if (slot < 0) return;     /* ignore frames from untracked nodes */

    odrive_state_t *s = &s_states[slot];
    uint32_t now = HAL_GetTick();

    switch (cmd_id) {
    case ODRV_CMD_HEARTBEAT:
        if (len >= 7) {
            s->axis_error        = unpack_u32(data);
            s->axis_state        = (odrive_axis_state_t)data[4];
            s->procedure_result  = data[5];
            s->trajectory_done   = data[6];
            s->last_heartbeat_ms = now;
            s->seen              = true;
        }
        break;

    case ODRV_CMD_GET_ENCODER_ESTIMATES:
        if (len >= 8) {
            s->pos_estimate_turns        = unpack_f32(data);
            s->vel_estimate_turns_per_s  = unpack_f32(data + 4);
            s->last_encoder_ms           = now;
        }
        break;

    case ODRV_CMD_GET_IQ:
        if (len >= 8) {
            s->iq_setpoint_a = unpack_f32(data);
            s->iq_measured_a = unpack_f32(data + 4);
            s->last_iq_ms    = now;
        }
        break;

    case ODRV_CMD_GET_BUS_VOLTAGE_CURRENT:
        if (len >= 8) {
            s->bus_voltage_v = unpack_f32(data);
            s->bus_current_a = unpack_f32(data + 4);
            s->last_busvi_ms = now;
        }
        break;

    default:
        /* Other feedback / responses not consumed here. */
        break;
    }
}

void odriveCanRxISR(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan);
}

/* HAL invokes this from inside HAL_FDCAN_IRQHandler when RX FIFO 0 has
 * at least one pending message.  Pull every available frame so we never
 * leave the FIFO non-empty (which would mask the interrupt). */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hf, uint32_t flags)
{
    if (hf->Instance != ODRIVE_CAN_INSTANCE) return;
    if (!(flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) return;

    while (HAL_FDCAN_GetRxFifoFillLevel(hf, FDCAN_RX_FIFO0) > 0) {
        FDCAN_RxHeaderTypeDef rx;
        uint8_t buf[8] = {0};
        if (HAL_FDCAN_GetRxMessage(hf, FDCAN_RX_FIFO0, &rx, buf) != HAL_OK) {
            break;
        }
        /* HAL stores DLC in the same len<<16 encoding; decode by table. */
        uint8_t len = 0;
        switch (rx.DataLength) {
            case FDCAN_DLC_BYTES_0: len = 0; break;
            case FDCAN_DLC_BYTES_1: len = 1; break;
            case FDCAN_DLC_BYTES_2: len = 2; break;
            case FDCAN_DLC_BYTES_3: len = 3; break;
            case FDCAN_DLC_BYTES_4: len = 4; break;
            case FDCAN_DLC_BYTES_5: len = 5; break;
            case FDCAN_DLC_BYTES_6: len = 6; break;
            case FDCAN_DLC_BYTES_7: len = 7; break;
            case FDCAN_DLC_BYTES_8: len = 8; break;
            default: len = 0; break;
        }
        handle_rx(rx.Identifier, buf, len);
    }
}
