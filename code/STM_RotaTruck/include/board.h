#ifndef BOARD_H
#define BOARD_H

#include "stm32f4xx_hal.h"

/* =====================================================================
 *  RotaTruck — ODrive S1 branch
 *
 *  Pin map  (Nucleo-G491RE — see docs/ODRIVE_INTEGRATION.md for the
 *  full hardware wiring diagram, including the CAN transceiver and
 *  ODrive connector mapping).
 *
 *  Motor drive: TWO ODrive S1 controllers (one per wheel) on a single
 *               shared CAN bus.  No PWM, no F/R, no brake GPIO from the
 *               STM32 — the ODrives own all the low-level motor control.
 *               The STM32 just sends setpoints (velocity, torque, state)
 *               and reads feedback (encoder, current, voltage) over CAN.
 *
 *    CAN2 RX = PB12  (DISC1 header P1) <- transceiver RX  (red,   CB1-1)
 *    CAN2 TX = PB13  (DISC1 header P1) -> transceiver TX  (brown, CB2-1)
 *    (Was FDCAN1 on PA11/PA12 on the Nucleo-G4.  On the DISC1, PA11/PA12
 *    are the micro-USB data lines and CAN1's other pairs clash with the
 *    IMU (PB8/PB9) - PB12/PB13 are free.)
 *
 *  Sensors / UI:
 *    ISM330DHCX IMU : I2C1 on PB8/PB9 (D14/D15), 0x6B
 *    16x2 LCD       : same I2C1 bus via PCF8574 backpack
 *    2 x HX711      : PA10/PB3 (CH1), PB5/PB4 (CH2) - load detection
 *    Button         : PA9 (assist toggle), active-high with pull-down
 *
 *  Debug:
 *    USART2       : PA2/PA3, via ST-Link VCP, 115200 8N1
 * =====================================================================
 */

/* =====================================================================
 *  CAN2 (bxCAN) — ODrive CAN bus on PB12 (RX) / PB13 (TX)
 *
 *  ODrive S1 default baud = 250 kbit/s.  Configurable up to 1 Mbit/s
 *  in the ODrive Web GUI (Interfaces page → CAN baudrate).  Match this
 *  constant to whatever you set there.
 *
 *  CAN2 is the "slave" bxCAN on the F407: it has no filter banks of its
 *  own, so the CAN1 clock must also be on, and its filters are banks
 *  14..27 of the shared CAN1 filter block (ODRIVE_CAN_FILTER_BANK).
 *
 *  Transceiver: TJA-series (5 V part) - power its VCC from the DISC1 5V
 *  pin, not 3V.  Its 5 V RX output is fine on PB12 (5 V-tolerant pin),
 *  and it reads the STM32's 3 V TX as high.  Don't move RX to a non-FT pin.
 *
 *  Bit-timing for 250 kbit/s from PCLK1 = 42 MHz (168 MHz / 4):
 *    Prescaler 12 → TQ = 285.7 ns
 *    14 TQ per bit:  Sync(1) + BS1(11) + BS2(2) — sample point 85.7 %
 *  odriveCanInit() refuses to start if PCLK1 isn't ODRIVE_CAN_PCLK1_HZ.
 * =====================================================================*/
#define ODRIVE_CAN_INSTANCE         CAN2
#define ODRIVE_CAN_TX_PORT          GPIOB
#define ODRIVE_CAN_TX_PIN           GPIO_PIN_13
#define ODRIVE_CAN_RX_PORT          GPIOB
#define ODRIVE_CAN_RX_PIN           GPIO_PIN_12
#define ODRIVE_CAN_GPIO_AF          GPIO_AF9_CAN2
#define ODRIVE_CAN_IRQn             CAN2_RX0_IRQn
#define ODRIVE_CAN_FILTER_BANK      14U   /* first CAN2 bank (= CAN2SB) */

/* CAN bit-timing — 250 kbit/s at PCLK1 = 42 MHz, 85.7 % sample point. */
#define ODRIVE_CAN_PCLK1_HZ         42000000U
#define ODRIVE_CAN_PRESCALER        12U
#define ODRIVE_CAN_SJW              CAN_SJW_1TQ
#define ODRIVE_CAN_BS1              CAN_BS1_11TQ
#define ODRIVE_CAN_BS2              CAN_BS2_2TQ

/* =====================================================================
 *  ODrive logical configuration — node IDs and motion mapping
 *
 *  These must match the ODrive's Web-GUI Interfaces page → Node ID.
 *  Connect the LEFT motor's ODrive over USB first, set Node ID = 0,
 *  enable CAN, save.  Then the RIGHT motor's ODrive: Node ID = 1.
 *  Repeat for any additional axes.
 * =====================================================================*/
#define ODRIVE_NODE_ID_LEFT         0U
#define ODRIVE_NODE_ID_RIGHT        1U

/* Encoder counts come back from the ODrive in TURNS (not raw counts),
 * via the Get_Encoder_Estimates feedback message.  Multiply by wheel
 * circumference for linear position.  No quadrature decoding here. */

/* Wheel geometry — used for any host-side m/s ↔ rev/s conversion. */
#define WHEEL_DIAMETER_M            0.1651f             /* 6.5 in default */
#define WHEEL_RADIUS_M              (WHEEL_DIAMETER_M * 0.5f)
#define WHEEL_CIRCUMFERENCE_M       (WHEEL_DIAMETER_M * 3.14159265f)

/* Velocity limits (forward-only assist for now).
 * Sent to each ODrive via Set_Limits when an axis is armed, so these are
 * the enforced limits, not just advisory. */
#define MAX_VELOCITY_REV_PER_S      4.0f                /* ~2 m/s linear */
#define MAX_TORQUE_NM               5.0f                /* per-axis cap   */

/* Per-motor direction sign.  If a wheel spins the wrong way for a forward
 * (+) command on first bring-up, flip its sign here rather than rewiring
 * the motor phases or re-negating at each call site (design decision #6).
 *   +1.0f = ODrive's native forward already matches truck-forward
 *   -1.0f = inverted
 * Applied to BOTH the outgoing velocity/torque setpoint and the incoming
 * encoder/Iq feedback, so "+ = truck-forward" is consistent across command
 * and telemetry (important for the admittance / momentum-observer loops). */
#define MOTOR_LEFT_DIR              (-1.0f)             /* left runs reversed */
#define MOTOR_RIGHT_DIR            (+1.0f)

/* =====================================================================
 *  IMU — ISM330DHCX (SparkFun SEN-19764) on I2C1 (PB8=SCL, PB9=SDA, AF4)
 *  3V3 only.  ADR jumper as shipped -> 0x6B (0x6A also probed).
 *  The MPU6050 it replaced was removed 2026-11-10.
 * =====================================================================*/
#define IMU_I2C_INSTANCE            I2C1
#define IMU_I2C_SCL_PORT            GPIOB
#define IMU_I2C_SCL_PIN             GPIO_PIN_8
#define IMU_I2C_SDA_PORT            GPIOB
#define IMU_I2C_SDA_PIN             GPIO_PIN_9
#define IMU_I2C_GPIO_AF             GPIO_AF4_I2C1
/* STM32F4 I2C (no TIMINGR): the HAL derives the bit timing from PCLK1
 * (42 MHz) and this clock speed.  100 kHz standard mode. */
#define IMU_I2C_CLOCK_HZ            100000U

/* Sensor -> CART frame.  Each ISM330DHCX reading is multiplied by these
 * signs.  The cart frame is the one the pose thresholds, tilt protection,
 * observer and load-detect pose references were tuned in:
 *   pitch = atan2(ay, az): STANDING ~ -43.6 deg, DRIVING ~ -1.2 deg
 *   pitch rate = +gx towards DRIVING, yaw rate = gz.
 * As mounted, the SparkFun board sits rotated 180 deg about Y (measured
 * 2026-11-10: pitch r = +0.997, gx / gz r = -0.995 / -0.999 against the
 * previous sensor).  If the board is remounted, check with `I S` while
 * tilting STANDING <-> DRIVING and flip the signs here. */
#define IMU_AXIS_SIGN_X             (-1.0f)
#define IMU_AXIS_SIGN_Y             (+1.0f)
#define IMU_AXIS_SIGN_Z             (-1.0f)

/* =====================================================================
 *  User interface
 *
 *  LCD: common 16x2 HD44780 display with a PCF8574 I2C backpack.  Most
 *  backpacks are either 0x27 or 0x3F; the firmware probes both.
 *  It shares I2C1 with the ISM330DHCX, so wire SDA/SCL in parallel.  Check
 *  the backpack's SDA/SCL pull-ups: if they go to 5 V they over-drive the
 *  3.3 V-only ISM330DHCX - remove them or power the backpack from 3V3.
 *
 *  Front-panel buttons: POLLED with a per-button debounce state machine
 *  (mode_input.c) — not interrupt-driven.  Edges are noisy on long wiring
 *  next to motors/CAN, so the driver samples the stable LEVEL and fires
 *  once per debounced press; that kills bounce/phantom/double clicks.
 *    +5 kg (INC)    -> PB6  (Arduino D10)  manual payload (fallback when
 *    -5 kg (DEC)    -> PC7  (Arduino D9)   load detection is not valid;
 *                                           see payload.h)
 *    assist toggle  -> PA9  (Arduino D8)   (same as the on-board B1 USER button)
 *  ACTIVE-HIGH wiring (matches the build): button -> 3.3 V, external
 *  pull-down to GND, so released = LOW and pressed = HIGH; the driver also
 *  enables the internal pull-down.  (Set ACTIVE_LOW 1 for button -> GND +
 *  pull-up wiring.) */
#define UI_PAYLOAD_INC_BUTTON_PORT  GPIOB
#define UI_PAYLOAD_INC_BUTTON_PIN   GPIO_PIN_6     /* D10, +step */
#define UI_PAYLOAD_DEC_BUTTON_PORT  GPIOC
#define UI_PAYLOAD_DEC_BUTTON_PIN   GPIO_PIN_7     /* D9,  -step */
#define UI_MODE_TOGGLE_BUTTON_PORT  GPIOA
#define UI_MODE_TOGGLE_BUTTON_PIN   GPIO_PIN_9     /* D8,  assist toggle */
#define UI_MODE_BUTTON_ACTIVE_LOW   0
#define UI_MODE_BUTTON_DEBOUNCE_MS  40U            /* level-stable time to commit */
#define UI_PAYLOAD_STEP_KG          5.0f           /* +/- per button press */
#define UI_PAYLOAD_MAX_KG           250.0f         /* clamp ceiling */

/* Handle safety (dead-man) switch — PE7 (DISC1 header P1).  deadman.c.
 * (Was A0/PA0 on the Nucleo; on the F407 Discovery PA0 is the blue USER
 * button.  Verified 2026-11-11: PE7 to GND, clean presses, no bounce.)
 * Released -> wheels ramp down to 0 and are held (assist_control.c).
 * WIRING (as built): one switch terminal to PE7, the other to GND, so
 * pressed = LOW.  ACTIVE-LOW with the internal pull-UP on PE7: a released
 * switch, a broken wire or an unplugged switch all read HIGH = RELEASED =
 * stop (fail-safe).  (Switch to 3.3 V instead -> DEADMAN_ACTIVE_LOW 0.)
 * Check with `M` (shows "handle PRESSED/RELEASED") before driving.
 * DEADMAN_ENABLE 0 ignores the switch - bench testing ONLY. */
#define DEADMAN_ENABLE              1
#define DEADMAN_PORT                GPIOE
#define DEADMAN_PIN                 GPIO_PIN_7
#define DEADMAN_ACTIVE_LOW          1
#define DEADMAN_RELEASE_DEBOUNCE_MS 20U            /* fast: release = stop      */
#define DEADMAN_PRESS_DEBOUNCE_MS   80U            /* slower: no bounce re-arm  */

/* =====================================================================
 *  Load detection — 2 x HX711 strain-gauge ADCs (loadcell.c)
 *  Ported from P102_LoadDetect_G491RE v4.2 (same pins as that rig).
 *    CH1 middle strip   (POISSON_HALF)  DT PA10 (D2)  SCK PB3 (D3)
 *    CH2 left + right   (POISSON_FULL)  DT PB5  (D4)  SCK PB4 (D5)
 *  Both HX711 VCC from 3V3 (logic matches the MCU), GND common.
 *  DT inputs have internal pull-ups so a loose wire reads NO_DATA.
 *  PB3 leaves JTAG (SWD on PA13/PA14 is unaffected).
 *  Status LED LD2 (PA5): solid = load valid, slow blink = settling / not
 *  calibrated / held / overload, fast blink = fault.
 *  Settings live in the LAST flash page (0x0807F800, 2 KB) — keep the
 *  firmware image well below 510 KB.
 * =====================================================================*/
#define LC_CH1_DT_PORT              GPIOA
#define LC_CH1_DT_PIN               GPIO_PIN_10
#define LC_CH1_SCK_PORT             GPIOB
#define LC_CH1_SCK_PIN              GPIO_PIN_3
#define LC_CH2_DT_PORT              GPIOB
#define LC_CH2_DT_PIN               GPIO_PIN_5
#define LC_CH2_SCK_PORT             GPIOB
#define LC_CH2_SCK_PIN              GPIO_PIN_4
#define LC_LED_PORT                 GPIOA
#define LC_LED_PIN                  GPIO_PIN_5

/* LCD row 1, third field: 1 = handle switch state (PRESS / RELSD),
 * 0 = load-detect status (OK / NOCAL / NODAT / HELD ...). */
#define LCD_ROW1_SHOW_HANDLE        1

#define LCD_I2C_ADDR_PRIMARY        0x27u
#define LCD_I2C_ADDR_FALLBACK       0x3Fu
#define LCD_I2C_TIMEOUT_MS          25U

/* =====================================================================
 *  Debug UART — USART2 via ST-Link VCP
 * =====================================================================*/
#define DEBUG_UART_INSTANCE         USART2
#define DEBUG_UART_BAUD             115200U
#define DEBUG_UART_TX_PORT          GPIOA
#define DEBUG_UART_TX_PIN           GPIO_PIN_2
#define DEBUG_UART_RX_PORT          GPIOA
#define DEBUG_UART_RX_PIN           GPIO_PIN_3
#define DEBUG_UART_AF               GPIO_AF7_USART2
#define DEBUG_UART_IRQn             USART2_IRQn

#endif /* BOARD_H */
