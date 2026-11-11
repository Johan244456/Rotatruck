/*
 *  odrive_can.c — minimal STM32F4 bxCAN (CAN2) driver for the ODrive S1.
 *
 *  Ported from the Nucleo-G4 FDCAN1 version: only the low-level send /
 *  init / receive code changed; the ODrive protocol layer is unchanged.
 *  Pins + bit timing in board.h (CAN2, PB12 = RX, PB13 = TX, 250 kbit/s).
 *
 *  All ODrive command + feedback IDs match the protocol documented at
 *  https://docs.odriverobotics.com/v/latest/manual/can-protocol.html
 *  (as of firmware 0.6.12).
 */

#include "odrive_can.h"
#include "board.h"
#include "stm32f4xx_hal.h"

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
 * a "please reply with the latest value" message.  HAL CAN exposes
 * this via CAN_RTR_DATA vs CAN_RTR_REMOTE.  Only
 * odriveRequestBusVI() uses rtr=true; all setpoint commands are data
 * frames. */

/* ---------------------------------------------------------------------
 *  Module state
 * ---------------------------------------------------------------------*/
static CAN_HandleTypeDef   hcan;
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

    CAN_TxHeaderTypeDef tx = {0};
    tx.StdId              = build_can_id(node_id, cmd_id);
    tx.IDE                = CAN_ID_STD;
    tx.RTR                = rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    tx.DLC                = len;
    tx.TransmitGlobalTime = DISABLE;

    uint8_t payload[8] = {0};
    if (data && len > 0) {
        memcpy(payload, data, len);
    }

    /* bxCAN has only 3 TX mailboxes.  Several call sites (e.g. the arm
     * sequence) queue 4+ frames back-to-back in a few microseconds, far
     * faster than the ~0.43 ms it takes to transmit one at 250 kbit/s.
     * Without this wait the mailboxes fill and the 4th frame — which
     * happens to be Set_Axis_State(CLOSED_LOOP) — is silently dropped,
     * leaving the axis stuck in IDLE with no error.  Block briefly for a
     * free mailbox.  On a healthy bus one frees within a frame time; the
     * 10 ms cap just stops us hanging if the bus is dead. */
    uint32_t t0 = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
        if ((uint32_t)(HAL_GetTick() - t0) > 10U) {
            return false;   /* bus stuck / not being ACKed */
        }
    }

    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(&hcan, &tx, payload, &mailbox) != HAL_OK) {
        /* no free mailbox or peripheral fault — caller can retry. */
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
    s_ready = false;

    /* The bit timing in board.h is only right for one PCLK1.  A wrong
     * clock gives a wrong baud rate and a bus full of error frames, so
     * refuse to start rather than disturb the ODrives. */
    if (HAL_RCC_GetPCLK1Freq() != ODRIVE_CAN_PCLK1_HZ) {
        uart_print("CAN: PCLK1 is not 42 MHz - fix the bit timing in board.h.\r\n");
        return false;
    }

    hcan.Instance                  = ODRIVE_CAN_INSTANCE;
    hcan.Init.Prescaler            = ODRIVE_CAN_PRESCALER;
    hcan.Init.Mode                 = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth        = ODRIVE_CAN_SJW;
    hcan.Init.TimeSeg1             = ODRIVE_CAN_BS1;
    hcan.Init.TimeSeg2             = ODRIVE_CAN_BS2;
    hcan.Init.TimeTriggeredMode    = DISABLE;
    hcan.Init.AutoBusOff           = ENABLE;   /* recover from bus-off by itself */
    hcan.Init.AutoWakeUp           = DISABLE;
    hcan.Init.AutoRetransmission   = ENABLE;
    hcan.Init.ReceiveFifoLocked    = DISABLE;  /* overwrite oldest on overrun */
    /* Send in the order queued, not by lowest ID.  Default bxCAN priority
     * is by identifier, which could reorder e.g. Set_Controller_Mode (0x0B)
     * after Set_Axis_State (0x07) within the arm sequence. */
    hcan.Init.TransmitFifoPriority = ENABLE;

    if (HAL_CAN_Init(&hcan) != HAL_OK) return false;

    /* Accept every standard-frame ID into RX FIFO 0.  We'll filter by
     * node_id in software since both ODrives share the same bus.
     * CAN2 owns banks ODRIVE_CAN_FILTER_BANK..27 (HAL writes CAN2SB into
     * CAN1's filter registers, hence the CAN1 clock in the MSP). */
    CAN_FilterTypeDef f = {0};
    f.FilterBank           = ODRIVE_CAN_FILTER_BANK;
    f.SlaveStartFilterBank = ODRIVE_CAN_FILTER_BANK;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;   /* mask = 0 → accept-all */
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan, &f) != HAL_OK) return false;

    /* HAL_CAN_Start waits (10 ms) for 11 recessive bits on RX.  It fails
     * if PB12 never sees an idle bus — usually an unpowered transceiver. */
    if (HAL_CAN_Start(&hcan) != HAL_OK) return false;
    if (HAL_CAN_ActivateNotification(&hcan,
            CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return false;
    }

    s_ready = true;
    return true;
}

void odriveCanGetBusStatus(uint8_t *tec, uint8_t *rec, bool *bus_off)
{
    uint32_t esr = s_ready ? hcan.Instance->ESR : 0U;
    if (tec)     *tec     = (uint8_t)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
    if (rec)     *rec     = (uint8_t)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
    if (bus_off) *bus_off = (esr & CAN_ESR_BOFF) != 0U;
}

/* ---------------------------------------------------------------------
 *  MSP — clocks, pins, NVIC.  Kept here (not in a *_hal_msp.c) so the
 *  driver is self-contained in every build env.
 * ---------------------------------------------------------------------*/
void HAL_CAN_MspInit(CAN_HandleTypeDef *h)
{
    if (h->Instance != ODRIVE_CAN_INSTANCE) return;

    __HAL_RCC_CAN1_CLK_ENABLE();   /* CAN2 needs CAN1's clock (shared filters) */
    __HAL_RCC_CAN2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_PP;
    /* No internal pull: a 5 V transceiver drives RX above VDD, and the
     * pull-up must stay off on a 5 V-tolerant pin when that happens. */
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = ODRIVE_CAN_GPIO_AF;

    g.Pin = ODRIVE_CAN_TX_PIN;  HAL_GPIO_Init(ODRIVE_CAN_TX_PORT, &g);
    g.Pin = ODRIVE_CAN_RX_PIN;  HAL_GPIO_Init(ODRIVE_CAN_RX_PORT, &g);

    HAL_NVIC_SetPriority(ODRIVE_CAN_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ODRIVE_CAN_IRQn);
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
 *  RX path — called from the CAN2 RX0 interrupt
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
    HAL_CAN_IRQHandler(&hcan);
}

/* HAL invokes this from inside HAL_CAN_IRQHandler when RX FIFO 0 has
 * at least one pending message.  Pull every available frame so we never
 * leave the FIFO non-empty. */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h)
{
    if (h->Instance != ODRIVE_CAN_INSTANCE) return;

    while (HAL_CAN_GetRxFifoFillLevel(h, CAN_RX_FIFO0) > 0) {
        CAN_RxHeaderTypeDef rx;
        uint8_t buf[8] = {0};
        if (HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &rx, buf) != HAL_OK) {
            break;
        }
        /* ODrive feedback is always standard-ID data frames. */
        if (rx.IDE != CAN_ID_STD || rx.RTR != CAN_RTR_DATA) continue;
        handle_rx(rx.StdId, buf, (uint8_t)(rx.DLC > 8 ? 8 : rx.DLC));
    }
}
