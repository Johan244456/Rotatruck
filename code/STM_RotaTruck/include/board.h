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
 *    2 x HX711      : PE11/PE12 (CH1), PE13/PE14 (CH2) - load detection, LED PD12
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
/* Initialised LOAD at power-up (kg): used until a load is accepted with the
 * handle or set with the +/- buttons / `W <kg>`.  The assist always gets
 * LOAD_ASSIST_SCALE x the initialised load (15 kg -> 10.5 kg).  The original
 * default was 0. */
#define PAYLOAD_DEFAULT_KG          15.0f

/* Fraction of the MEASURED load handed to the assist (observer mass): the
 * assist must never be sized above the real load (no over-assist), so it is
 * deliberately under-sized.  Was 0.9; 0.7 for the DISC1 trials.  Used by
 * load detection (loadcell.c) and the accepted payload (payload.c). */
#define LOAD_ASSIST_SCALE           0.7f

/* Accept the load with the HANDLE SWITCH: one click, while the IMU says the
 * trolley is at the standing angle (within 5 deg of the captured `q s`
 * reference), takes the last settled standing reading (from the
 * PAYLOAD_ACCEPT_MAX_AGE_MS before the press - gripping the handle disturbs
 * the plate) and sets the assist payload to LOAD_ASSIST_SCALE x measured.
 * Not standing / no valid reading -> refused with the reason, payload kept.
 * 0 = original rule (latch on STANDING -> DRIVE after arming at STANDING). */
#define PAYLOAD_ACCEPT_ON_HANDLE    1
#define PAYLOAD_ACCEPT_MAX_AGE_MS   3000U
/* Accept only at the STANDING angle itself: the IMU must be within this many
 * degrees of the captured standing pose (`q s`) at the click, and the reading
 * accepted must have been taken that close too.  (The live STANDING zone for
 * load detection stays at ~5 deg.)  ~2 deg = "exactly" standing within IMU noise. */
#define PAYLOAD_ACCEPT_MAX_DEV_DEG  2.0f

/* AUTO-ACCEPT: whenever the trolley is exactly upright (<= PAYLOAD_ACCEPT_MAX_DEV_DEG)
 * and the reading has settled, the detected load becomes the load - no click
 * needed.  It stays frozen while tipped back / driving; standing up again
 * re-reads the TOTAL on the plate, so load can be added between drives.
 * Updates only when it differs by more than PAYLOAD_AUTO_MIN_CHANGE_KG.
 * (One click still accepts manually.)  0 = click only. */
#define PAYLOAD_AUTO_ACCEPT         1
#define PAYLOAD_AUTO_MIN_CHANGE_KG  0.5f

/* One pose definition for the whole system: the motor pose machine
 * (assist_control.c: STANDING = wheels held, DRIVING = assist) uses the SAME
 * captured references as load detection (`q s` standing / `q r` tipped back)
 * instead of the fixed pitch thresholds tuned for the truck's IMU mounting.
 *   STANDING   = within ~5 deg of `q s`   -> wheels held, load accepted here
 *   TIPPED BACK= >= 15 deg back from it    -> assist drives (0.7 x load)
 *   in between = keeps the previous pose;  IMU lost = STANDING (wheels held)
 * 0 = original pitch thresholds (POSE_*_ENTER_DEG in assist_tuning.h). */
#define POSE_FROM_LOAD_REFS         1

/* Assist only when tipped back FULLY: the motors are released to the assist
 * only within POSE_DRIVE_FULL_ENTER_DEG of the captured tipped-back pose (`q r`),
 * and keep assisting until more than POSE_DRIVE_FULL_EXIT_DEG away (so normal
 * handle movement while pushing doesn't toggle it).  Anywhere else - upright,
 * partly tilted, IMU lost - the wheels are held. */
#define POSE_DRIVE_FULL_ENTER_DEG   5.0f
#define POSE_DRIVE_FULL_EXIT_DEG    10.0f

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
/* DEADMAN_ARMS_ASSIST 1 (no assist buttons wired yet): PRESSING the handle
 * switch arms the assist (AS 1 + friction / gravity comp, as the B1 / D8
 * button did); RELEASING it ramps the wheels to 0 (DEADMAN_DECEL_REV_PER_S2)
 * and, once stopped and held for DEADMAN_DISARM_HOLD_MS, disarms it (axes
 * IDLE).  0 = original behaviour: the switch only gates an assist armed by
 * a button / `AS 1`, and the wheels are held at 0 after a release. */
#define DEADMAN_ARMS_ASSIST         1
#define DEADMAN_DISARM_HOLD_MS      500U
/* Handle gestures (deadman.c, payload.c, assist_control.c):
 *   ONE CLICK (press <= DEADMAN_CLICK_MAX_MS, no 2nd click within
 *           DEADMAN_CLICK_GAP_MS) -> accept the most recent settled load,
 *           only while STANDING (PAYLOAD_ACCEPT_ON_HANDLE).
 *   HOLD  = pressed >= DEADMAN_HOLD_MS  -> arms the assist (quick clicks never
 *           arm the motors).  Release still stops at once.
 *   TRIPLE CLICK = 3 presses each <= DEADMAN_CLICK_MAX_MS, gaps <=
 *           DEADMAN_CLICK_GAP_MS -> manual load-detect zero, only when the IMU
 *           says the trolley is at / near tipped back (where the zero is taken). */
#define DEADMAN_HOLD_MS             400U
#define DEADMAN_CLICK_MAX_MS        350U
#define DEADMAN_CLICK_GAP_MS        500U
#define DEADMAN_PRESS_DEBOUNCE_MS   80U            /* slower: no bounce re-arm  */

/* =====================================================================
 *  Load detection — 2 x HX711 strain-gauge ADCs (loadcell.c)
 *  Ported from P102_LoadDetect_G491RE v4.2.
 *    CH1 middle strip   (POISSON_HALF)  DT PE11  SCK PE12
 *    CH2 left + right   (POISSON_FULL)  DT PE13  SCK PE14
 *  Both moved off the Nucleo pins (PA10/PB3, PB5/PB4) onto free DISC1
 *  pins next to each other on the long header (PA10 is the micro-USB ID
 *  line, PB3 the ST-Link SWO line).
 *  Both HX711 VCC from 3V (DISC1 = 3.0 V; the HX711 runs from 2.6 V, the
 *  bridge excitation is a little lower than on 3.3 V - recalibrate after
 *  any supply change).  GND common.
 *  DT inputs have internal pull-ups so a loose wire reads NO_DATA.
 *  Status LED: DISC1 LD4 (green, PD12) - was LD2/PA5 on the Nucleo (PA5 is
 *  the on-board accelerometer's SPI clock on the DISC1).  Solid = load
 *  valid, slow blink = settling / not calibrated / held / overload, fast
 *  blink = fault.
 *  Settings live in flash sector 11 (0x080E0000, 128 KB) - keep the
 *  firmware image below 896 KB.
 * =====================================================================*/
#define LC_CH1_DT_PORT              GPIOE
#define LC_CH1_DT_PIN               GPIO_PIN_11
#define LC_CH1_SCK_PORT             GPIOE
#define LC_CH1_SCK_PIN              GPIO_PIN_12
#define LC_CH2_DT_PORT              GPIOE
#define LC_CH2_DT_PIN               GPIO_PIN_13
#define LC_CH2_SCK_PORT             GPIOE
#define LC_CH2_SCK_PIN              GPIO_PIN_14
#define LC_LED_PORT                 GPIOD
#define LC_LED_PIN                  GPIO_PIN_12

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

/* =====================================================================
 *  Optional parts on the F407 DISC1 bring-up
 * =====================================================================*/
/* IMU_ENABLE 1: ISM330DHCX on I2C1 drives the pose machine (STANDING /
 * DRIVING), the observer slope and load-detection pose (accepting the load
 * on the handle press needs it).  0 = never started or read even if wired:
 * "dev mode", the handle switch alone gates the assist. */
#ifndef IMU_ENABLE
#define IMU_ENABLE                  1
#endif

/* UI_BUTTONS_ENABLE 0: the front-panel buttons (payload +/-, D8 assist
 * toggle) are not wired on the DISC1 yet, and their Nucleo pins clash with
 * DISC1 parts (PB6 has the audio chip's pull-up = reads "pressed").  Payload
 * stays at its manual value (W <kg> / load detection). */
#define UI_BUTTONS_ENABLE           0

#endif /* BOARD_H */
