/*
 *  command.c — UART line-buffered command parser.
 *
 *  ODrive S1 branch — commands now operate on motor velocity (rev/s)
 *  or torque (N·m) instead of PWM percent, because the BLD300B is gone
 *  and the ODrive expects physical units over CAN.
 */

#include "command.h"
#include "board.h"
#include "motor_control.h"
#include "odrive_can.h"
#include "imu.h"
#include "ism330.h"
#include "assist_control.h"
#include "assist_tuning.h"
#include "observer.h"
#include "admittance.h"
#include "autoseq.h"
#include "loadcell.h"
#include "payload.h"
#include "deadman.h"
#include "rtt.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

/* =====================================================================
 *  USART handle + RX line buffer
 * =====================================================================*/
UART_HandleTypeDef huart_dbg;

#define RX_LINE_MAX  96

static volatile char     rx_byte = 0;
static volatile char     rx_line[RX_LINE_MAX];
static volatile uint16_t rx_idx = 0;
static volatile uint8_t  line_ready = 0;

/* =====================================================================
 *  TX helpers (blocking)
 * =====================================================================*/
/* F407 DISC1: the ST-Link VCP is not wired to the MCU, so the console is
 * SEGGER RTT over the ST-Link (PlatformIO serial monitor, socket :9090) -
 * and also USART2 TX = PA2 for a USB-serial adapter. */
void uart_print(const char *s)
{
    size_t n = strlen(s);
    rttWrite(s, n);
    HAL_UART_Transmit(&huart_dbg, (uint8_t *)s, (uint16_t)n, HAL_MAX_DELAY);
}

void uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) {
        if (n >= (int)sizeof buf) n = (int)sizeof buf - 1;
        rttWrite(buf, (size_t)n);
        HAL_UART_Transmit(&huart_dbg, (uint8_t *)buf, (uint16_t)n, HAL_MAX_DELAY);
    }
}

/* =====================================================================
 *  Init
 * =====================================================================*/
static void print_banner(void);
static void print_help(void);
static void print_imu_sample(void);
static void do_imu_calibrate(void);
static void print_mode(void);
static void set_assist_enabled(bool en);
static void print_observer(void);
static void print_assist_status(void);
static void print_pose_pitch(void);
static void print_imu_stream(void);
static void run_imu_still_test(void);

/* Observer telemetry stream (OB S toggles it; printed from commandPoll). */
static bool     ob_stream_on   = false;
static uint32_t ob_stream_last = 0;

/* Assist status stream (AS S toggles it; printed from commandPoll). */
static bool     as_stream_on   = false;
static uint32_t as_stream_last = 0;

/* Pose / pitch stream (M S toggles it; for finding pose thresholds). */
static bool     ms_stream_on   = false;
static uint32_t ms_stream_last = 0;

/* Raw 6-DOF IMU stream (I S toggles it; for checking IMU mounting/axes). */
static bool     is_stream_on   = false;
static uint32_t is_stream_last = 0;

/* Load / payload live log (PL 1 / PL 0 / PL S, 5 Hz; host/live_load_log.py).
 * PL,t_ms,handle,pitch_deg,lc_pose,stand_dev_deg,rest_dev_deg,load_status,measured_kg,
 *    offer_kg,load_kg,payload_kg,payload_src,assist_on,assist_pose
 *    (load_kg = initialised load, payload_kg = LOAD_ASSIST_SCALE x load_kg) */
static bool     pl_stream_on   = false;
static uint32_t pl_stream_last = 0;

static void print_payload_log(void)
{
    uart_printf("PL,%lu,%d,%.1f,%s,%.1f,%.1f,%s,%.2f,%.2f,%.2f,%.2f,%s,%d,%s\r\n",
                (unsigned long)HAL_GetTick(), deadmanPressed() ? 1 : 0,
                (double)assistGetPitchDeg(), loadcellPoseName(), (double)loadcellStandDevDeg(),
                (double)loadcellRestDevDeg(),
                loadcellStatusName(), (double)loadcellMeasuredKg(), (double)loadcellAssistKg(),
                (double)payloadGetLoadKg(), (double)payloadGetKg(),
                payloadIsManual() ? "MANUAL" : "ACCEPTED",
                admittanceIsEnabled() ? 1 : 0, assistGetPoseName());
}

void commandSetup(void)
{
    huart_dbg.Instance        = DEBUG_UART_INSTANCE;
    huart_dbg.Init.BaudRate   = DEBUG_UART_BAUD;
    huart_dbg.Init.WordLength = UART_WORDLENGTH_8B;
    huart_dbg.Init.StopBits   = UART_STOPBITS_1;
    huart_dbg.Init.Parity     = UART_PARITY_NONE;
    huart_dbg.Init.Mode       = UART_MODE_TX_RX;
    huart_dbg.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart_dbg.Init.OverSampling     = UART_OVERSAMPLING_16;
    rttInit();                 /* console over the ST-Link (RTT) */
    HAL_UART_Init(&huart_dbg);

    HAL_UART_Receive_IT(&huart_dbg, (uint8_t *)&rx_byte, 1);

    print_banner();
}

/* =====================================================================
 *  RX ISR
 * =====================================================================*/
void commandUartISR(void)
{
    HAL_UART_IRQHandler(&huart_dbg);
}

/* One received character -> line buffer (USART2 ISR and RTT poll). */
static void rx_char(char c)
{
    if (c == '\r' || c == '\n') {
        if (rx_idx > 0) {
            rx_line[rx_idx] = '\0';
            line_ready = 1;
        }
    } else if ((unsigned char)c < 32 || (unsigned char)c > 126) {
        /* ignore junk bytes (some USB-serial ports send one on open) */
    } else if (rx_idx < RX_LINE_MAX - 1 && !line_ready) {
        rx_line[rx_idx++] = c;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != DEBUG_UART_INSTANCE) return;
    rx_char(rx_byte);
    HAL_UART_Receive_IT(huart, (uint8_t *)&rx_byte, 1);
}

/* =====================================================================
 *  Tokeniser
 * =====================================================================*/
static char   tokbuf[RX_LINE_MAX];
static char  *tokv[6];
static int    tokc = 0;

static void tokenise(const char *line)
{
    strncpy(tokbuf, line, sizeof tokbuf - 1);
    tokbuf[sizeof tokbuf - 1] = '\0';
    tokc = 0;
    char *p = tokbuf;
    while (*p && tokc < 6) {
        while (*p && isspace((unsigned char)*p)) p++;
        if (!*p) break;
        tokv[tokc++] = p;
        while (*p && !isspace((unsigned char)*p)) p++;
        if (*p) *p++ = '\0';
    }
}

static char up(char c)
{
    return (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
}

static int parse_motor(const char *s, char *out)
{
    if (!s || !s[0] || s[1] != '\0') return 0;
    char m = up(s[0]);
    if (m == 'L' || m == 'R' || m == 'B') { *out = m; return 1; }
    return 0;
}

/* =====================================================================
 *  Command execution
 * =====================================================================*/
static void exec_vel_time(char motor, float seconds, float rps)
{
    switch (motor) {
    case 'L': moveMotorLeftForTime(rps, seconds);  break;
    case 'R': moveMotorRightForTime(rps, seconds); break;
    case 'B': moveBothForTime(rps, seconds);       break;
    }
}

static void exec_vel_set(char motor, float rps)
{
    switch (motor) {
    case 'L': setMotorLeftVelocity(rps);  break;
    case 'R': setMotorRightVelocity(rps); break;
    case 'B': setMotorBothVelocity(rps);  break;
    }
    uart_printf("OK: %c velocity = %.3f rev/s\r\n", motor, (double)rps);
}

static void exec_torque_set(char motor, float nm)
{
    switch (motor) {
    case 'L': setMotorLeftTorque(nm);  break;
    case 'R': setMotorRightTorque(nm); break;
    case 'B': setMotorLeftTorque(nm); setMotorRightTorque(nm); break;
    }
    uart_printf("OK: %c torque = %.3f N·m\r\n", motor, (double)nm);
}

/* True if the line is the `LD` console switch (`LD`, `LD 0`, `LD 1`). */
static bool is_ld_line(const char *s)
{
    while (*s == ' ') s++;
    return up(s[0]) == 'L' && up(s[1]) == 'D' && (s[2] == '\0' || s[2] == ' ');
}

/* `LD` toggles the load-detect console, `LD 1` / `LD 0` force it.  On entry
 * the assist is switched off (motors idle) and the motor-side streams stop,
 * so the bench-rig CSV is clean for the host GUIs. */
static void exec_ld(void)
{
    bool on = !loadcellConsoleActive();
    if (tokc >= 2) {
        if (tokv[1][0] == '1' && tokv[1][1] == '\0')      on = true;
        else if (tokv[1][0] == '0' && tokv[1][1] == '\0') on = false;
        else { uart_print("ERR: use LD, LD 1 or LD 0\r\n"); return; }
    }
    if (on) {
        if (admittanceIsEnabled()) admittanceEnable(false);
        ob_stream_on = as_stream_on = ms_stream_on = is_stream_on = pl_stream_on = false;
    }
    loadcellSetConsole(on);
    if (!on) uart_print("OK: motor console. Load detection keeps running (W shows the payload).\r\n");
}

static void execute_line(const char *line)
{
    tokenise(line);
    if (tokc == 0) return;

    if (up(tokv[0][0]) == 'L' && up(tokv[0][1]) == 'D' && tokv[0][2] == '\0') {
        exec_ld();
        return;
    }

    /* Single-token commands. */
    if (tokc == 1) {
        if (tokv[0][1] == '\0') {
            char c = up(tokv[0][0]);
            if      (c == 'S') {
                if (admittanceIsEnabled()) admittanceEnable(false); /* kill assist */
                stopMotors();
                uart_print("OK: stop\r\n");
                return;
            }
            else if (c == 'P') { printMotorStatus();  return; }
            else if (c == 'I') { print_imu_sample();  return; }
            else if (c == 'M') { print_mode();        return; }
            else if (c == 'W') { payloadPrintStatus(); return; }
            else if (c == 'H' || c == '?') { print_help(); return; }
        }
        if (tokv[0][1] && tokv[0][2] == '\0') {
            char c0 = up(tokv[0][0]);
            char c1 = up(tokv[0][1]);
            if (c0 == 'I' && c1 == 'C') { do_imu_calibrate(); return; }
            if (c0 == 'O' && c1 == 'D') {
                printOdriveStatus();
                return;
            }
            if (c0 == 'O' && c1 == 'B') { print_observer(); return; }
            if (c0 == 'A' && c1 == 'S') { print_assist_status(); return; }
            if (c0 == 'P' && c1 == 'L') { print_payload_log(); return; }
        }
        if (strlen(tokv[0]) == 3 && up(tokv[0][0]) == 'I' &&
            tokv[0][1] == '2' && up(tokv[0][2]) == 'C') {
            imuScanBus();
            return;
        }
        goto err_syntax;
    }

    /* Brake-style commands (K 0/1/A) — translate to ODrive axis state. */
    if (tokc == 2 && tokv[0][1] == '\0' && up(tokv[0][0]) == 'K') {
        if (tokv[1][1] != '\0') goto err_syntax;
        char arg = up(tokv[1][0]);
        if      (arg == '1') { assistForceBrakeOn();  uart_print("OK: ODrives forced IDLE\r\n"); return; }
        else if (arg == '0') { assistForceBrakeOff(); uart_print("OK: ODrives forced CLOSED_LOOP\r\n"); return; }
        else if (arg == 'A') { assistAutoBrake();     uart_print("OK: brake AUTO\r\n"); return; }
        goto err_syntax;
    }

    /* `E 1` / `E 0` — arm / disarm closed-loop on both axes. */
    if (tokc == 2 && tokv[0][1] == '\0' && up(tokv[0][0]) == 'E') {
        if (tokv[1][1] != '\0') goto err_syntax;
        char arg = tokv[1][0];
        if      (arg == '1') { set_assist_enabled(true);  return; }
        else if (arg == '0') { set_assist_enabled(false); return; }
        goto err_syntax;
    }

    /* `OD C` / `OD R` / `OD ?` — ODrive sub-commands. */
    if (tokc == 2 && tokv[0][0] == 'O' && tokv[0][1] == 'D' && tokv[0][2] == '\0') {
        char arg = up(tokv[1][0]);
        if      (arg == 'C') { motorControlClearErrors(); return; }
        else if (arg == '?') { printOdriveStatus();       return; }
        goto err_syntax;
    }

    /* `OB S/Z/T/G/F` — observer stream / zero pitch / tare / gravity / friction toggle. */
    if (tokc == 2 && up(tokv[0][0]) == 'O' && up(tokv[0][1]) == 'B' && tokv[0][2] == '\0') {
        char arg = up(tokv[1][0]);
        if (arg == 'S') {
            ob_stream_on   = !ob_stream_on;
            ob_stream_last = HAL_GetTick();
            uart_printf("OK: observer stream %s\r\n", ob_stream_on ? "ON" : "OFF");
            return;
        } else if (arg == 'Z') {
            observerZeroPitch();
            uart_print("OK: pitch baseline zeroed (do this in DRIVE pose)\r\n");
            return;
        } else if (arg == 'T') {
            observerTare();
            uart_print("OK: force estimate tared (resting Fext -> 0)\r\n");
            return;
        } else if (arg == 'G') {
            observerSetGravityComp(!observerGravityEnabled());
            uart_printf("OK: gravity comp %s\r\n",
                        observerGravityEnabled() ? "ON" : "OFF");
            return;
        } else if (arg == 'F') {
            observerSetFrictionComp(!observerFrictionEnabled());
            uart_printf("OK: friction comp %s%s\r\n",
                        observerFrictionEnabled() ? "ON" : "OFF",
                        observerFrictionEnabled() ? "" : " (keep OFF on a free bench wheel)");
            return;
        }
        goto err_syntax;
    }

    /* `PL` one line / `PL 1` `PL 0` `PL S` load + payload live log (5 Hz). */
    if (up(tokv[0][0]) == 'P' && up(tokv[0][1]) == 'L' && tokv[0][2] == '\0') {
        if (tokc == 1) { print_payload_log(); return; }
        if (tokc == 2 && tokv[1][1] == '\0') {
            char a = up(tokv[1][0]);
            if (a == '1' || a == '0' || a == 'S') {
                pl_stream_on   = (a == 'S') ? !pl_stream_on : (a == '1');
                pl_stream_last = HAL_GetTick();
                uart_printf("OK: load/payload log %s  (PL,t_ms,handle,pitch_deg,lc_pose,stand_dev_deg,"
                            "rest_dev_deg,load_status,measured_kg,offer_kg,load_kg,payload_kg,payload_src,assist_on,assist_pose)\r\n",
                            pl_stream_on ? "ON" : "OFF");
                return;
            }
        }
        goto err_syntax;
    }

    /* `AS 1` / `AS 0` (enable/disable) / `AS S` (toggle status stream). */
    if (tokc == 2 && up(tokv[0][0]) == 'A' && up(tokv[0][1]) == 'S' && tokv[0][2] == '\0') {
        if (tokv[1][1] != '\0') goto err_syntax;
        char arg = tokv[1][0];
        if      (arg == '1') { admittanceEnable(true);  return; }
        else if (arg == '0') { admittanceEnable(false); return; }
        else if (up(arg) == 'S') {
            as_stream_on   = !as_stream_on;
            as_stream_last = HAL_GetTick();
            uart_printf("OK: assist status stream %s\r\n", as_stream_on ? "ON" : "OFF");
            return;
        }
        goto err_syntax;
    }

    /* `W <kg>` — manual payload (same as the +/- buttons, not saved).
     * `W A`    — back to this session's latched load-detect value. */
    if (tokc == 2 && up(tokv[0][0]) == 'W' && tokv[0][1] == '\0') {
        if (up(tokv[1][0]) == 'A' && tokv[1][1] == '\0') {
            if (payloadUseLatched())
                uart_printf("OK: load = %.1f kg ACCEPTED -> assist payload %.1f kg\r\n",
                            (double)payloadGetLoadKg(), (double)payloadGetKg());
            else
                uart_print("ERR: nothing latched yet - switch the assist ON at STANDING, "
                           "then tip back.\r\n");
        } else {
            char *end;
            float kg = strtof(tokv[1], &end);
            if (end == tokv[1]) goto err_syntax;
            payloadSetManual(kg);
            uart_printf("OK: load = %.1f kg MANUAL -> assist payload %.1f kg (%.2f x)\r\n",
                        (double)payloadGetLoadKg(), (double)payloadGetKg(), (double)LOAD_ASSIST_SCALE);
        }
        return;
    }

    /* `M S` — toggle the pose / pitch stream (for tuning pose thresholds). */
    if (tokc == 2 && up(tokv[0][0]) == 'M' && tokv[0][1] == '\0' && up(tokv[1][0]) == 'S') {
        ms_stream_on   = !ms_stream_on;
        ms_stream_last = HAL_GetTick();
        uart_printf("OK: pose/pitch stream %s\r\n", ms_stream_on ? "ON" : "OFF");
        return;
    }

    /* `I S` — toggle the raw 6-DOF IMU stream (for checking mounting/axes). */
    if (tokc == 2 && up(tokv[0][0]) == 'I' && tokv[0][1] == '\0' && up(tokv[1][0]) == 'S') {
        is_stream_on   = !is_stream_on;
        is_stream_last = HAL_GetTick();
        uart_printf("OK: raw IMU stream %s\r\n", is_stream_on ? "ON" : "OFF");
        return;
    }

    /* `I B` — 2 s still test: gyro bias / noise + |a| of the IMU. */
    if (tokc == 2 && up(tokv[0][0]) == 'I' && tokv[0][1] == '\0'
        && up(tokv[1][0]) == 'B' && tokv[1][1] == '\0') {
        run_imu_still_test();
        return;
    }

    /* `R A` / `R M` — autonomous / manual run mode.  Checked before the
     * motor parser (a bare `R <A|M>` is never a valid motor command). */
    if (tokc == 2 && up(tokv[0][0]) == 'R' && tokv[0][1] == '\0') {
        char arg = up(tokv[1][0]);
        if (arg == 'A' && tokv[1][1] == '\0') { autoseqSetMode(RUN_MODE_AUTO);   return; }
        if (arg == 'M' && tokv[1][1] == '\0') { autoseqSetMode(RUN_MODE_MANUAL); return; }
    }

    /* Motor + mode commands.  parse_motor enforces L|R|B in first token. */
    char motor;
    if (!parse_motor(tokv[0], &motor)) goto err_syntax;

    if (!assistIsEnabled()) {
        uart_print("ERR: motors disabled (truck in STANDING). Tilt forward or `K 0`.\r\n");
        return;
    }

    /* Mode token must be single char T, V, or Q (torque). */
    if (tokv[1][1] != '\0') goto err_syntax;
    char mode = up(tokv[1][0]);

    if (mode == 'T') {
        /* `M T <secs> <rps>` — time-based velocity move */
        if (tokc != 4) goto err_args;
        float secs = strtof(tokv[2], NULL);
        float rps  = strtof(tokv[3], NULL);
        uart_print("OK\r\n");
        exec_vel_time(motor, secs, rps);
        return;
    }
    if (mode == 'V') {
        /* `M V <rps>` — set velocity (held until stop) */
        if (tokc != 3) goto err_args;
        float rps = strtof(tokv[2], NULL);
        exec_vel_set(motor, rps);
        return;
    }
    if (mode == 'Q') {
        /* `M Q <nm>` — set torque */
        if (tokc != 3) goto err_args;
        float nm = strtof(tokv[2], NULL);
        exec_torque_set(motor, nm);
        return;
    }
    goto err_syntax;

err_syntax:
    uart_print("ERR: syntax. Try 'H' for help.\r\n");
    return;
err_args:
    uart_print("ERR: wrong arg count.\r\n");
    return;
}

/* =====================================================================
 *  Poll — call from main loop
 * =====================================================================*/
void commandPoll(void)
{
    /* Observer telemetry stream — paced ~10 Hz, runs every loop iteration
     * regardless of whether a command line is pending. */
    if (ob_stream_on) {
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - ob_stream_last) >= 100U) {
            ob_stream_last = now;
            print_observer();
        }
    }
    if (as_stream_on) {
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - as_stream_last) >= 150U) {
            as_stream_last = now;
            print_assist_status();
        }
    }
    if (ms_stream_on) {
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - ms_stream_last) >= 150U) {
            ms_stream_last = now;
            print_pose_pitch();
        }
    }
    if (pl_stream_on) {
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - pl_stream_last) >= 200U) {
            pl_stream_last = now;
            print_payload_log();
        }
    }
    if (is_stream_on) {
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - is_stream_last) >= 100U) {
            is_stream_last = now;
            print_imu_stream();
        }
    }

    /* RTT input (ST-Link console): same line assembler as the USART ISR. */
    int c;
    while (!line_ready && (c = rttGetc()) >= 0) {
        __disable_irq();
        rx_char((char)c);
        __enable_irq();
    }

    if (!line_ready) return;
    char local[RX_LINE_MAX];
    __disable_irq();
    strncpy(local, (const char *)rx_line, sizeof local - 1);
    local[sizeof local - 1] = '\0';
    rx_idx = 0;
    line_ready = 0;
    __enable_irq();

    /* Inside the load-detect console every line except `LD ...` goes to the
     * bench-rig command parser (its letters clash with the motor console). */
    if (loadcellConsoleActive() && !is_ld_line(local)) {
        loadcellHandleLine(local);
        return;
    }
    execute_line(local);
}

/* =====================================================================
 *  IMU + status helpers
 * =====================================================================*/
/* One IMU sample in the CART frame (the axes the pose machine, observer and
 * load detection use - see board.h IMU_AXIS_SIGN_*), with |a| and pitch. */
static void print_imu_line(void)
{
    if (!imuRead()) {
        uart_print("IMU read failed (ISM330DHCX not responding)\r\n");
        return;
    }
    const float ax = imuGetAxG(), ay = imuGetAyG(), az = imuGetAzG();
    uart_printf("IMU a=[%+.3f %+.3f %+.3f]g |a|=%.3f  g=[%+6.2f %+6.2f %+6.2f]dps  "
                "pitch=%+6.1f deg\r\n",
                (double)ax, (double)ay, (double)az,
                (double)sqrtf(ax * ax + ay * ay + az * az),
                (double)imuGetGxDps(), (double)imuGetGyDps(), (double)imuGetGzDps(),
                (double)imuGetPitchDeg());
}

static void print_imu_sample(void) { print_imu_line(); }

/* ~10 Hz stream of the same line: tilt STANDING <-> DRIVING and check that
 * pitch swings ~-44 -> ~-1 deg and gx is + while tilting towards DRIVING. */
static void print_imu_stream(void) { print_imu_line(); }

/* Still test: ~2 s of samples with the truck at rest.  Reports gyro bias
 * (mean) and noise (std-dev) per axis plus mean |a| (should be ~1.000 g).
 * Blocks the main loop, so it refuses while the assist is on. */
#define STILL_SAMPLES 200U
static void run_imu_still_test(void)
{
    if (admittanceIsEnabled()) {
        uart_print("ERR: turn the assist off (AS 0) first - this test blocks ~2 s.\r\n");
        return;
    }
    uart_print("IMU still test: keep the truck completely still for ~2 s...\r\n");

    double sum[3] = {0}, sq[3] = {0}, amag = 0.0;
    unsigned n = 0;
    for (unsigned k = 0; k < STILL_SAMPLES; k++) {
        if (imuRead()) {
            const double g[3] = { imuGetGxDps(), imuGetGyDps(), imuGetGzDps() };
            for (int a = 0; a < 3; a++) { sum[a] += g[a]; sq[a] += g[a] * g[a]; }
            const double x = imuGetAxG(), y = imuGetAyG(), z = imuGetAzG();
            amag += sqrt(x * x + y * y + z * z);
            n++;
        }
        HAL_Delay(10);
    }
    if (n == 0) { uart_print("ERR: no IMU samples (ISM330DHCX not responding).\r\n"); return; }

    double mean[3], sd[3];
    for (int a = 0; a < 3; a++) {
        mean[a] = sum[a] / n;
        double var = sq[a] / n - mean[a] * mean[a];
        sd[a] = (var > 0.0) ? sqrt(var) : 0.0;
    }
    uart_print("            gyro bias X/Y/Z (dps)     gyro noise X/Y/Z (dps)   mean |a| (g)\r\n");
    uart_printf("ISM330DHCX  %+6.2f %+6.2f %+6.2f     %5.3f %5.3f %5.3f        %.3f   (%u samples)\r\n",
                mean[0], mean[1], mean[2], sd[0], sd[1], sd[2], amag / n, n);
    float t;
    if (ism330ReadTempC(&t)) uart_printf("ISM330DHCX die temperature: %.1f C\r\n", (double)t);
}

static void do_imu_calibrate(void)
{
    if (!imuIsPresent()) { uart_printf("ERR: IMU (%s) not responding.\r\n", imuGetSourceName()); return; }
    uart_print("IMU: calibrating — hold the truck still in current pose...\r\n");
    assistCalibrateNow();
    uart_print("IMU: calibration done.\r\n");
}

static void print_observer(void)
{
    const char *st;
    switch (observerGetStatus()) {
        case OBSERVER_OK:        st = "OK";       break;
        case OBSERVER_NOT_READY: st = "WARMUP";   break;
        case OBSERVER_DEGRADED:  st = "DEGRADED"; break;
        default:                 st = "?";        break;
    }
    if (admittanceIsEnabled()) {
        /* Combined view: operator push -> commanded/measured assist force. */
        /* slope = the estimated road slope the gravity comp uses (G=on);
         * pitch = frame tilt vs drive entry (mostly handle angle) - shown
         * for comparison only, it no longer feeds the gravity comp. */
        uart_printf("OBS %-8s Fext=%+6.1fN  Fmot=%+6.1fN | %s  cmd=%+6.1fN  "
                    "v=%+5.2fm/s  slope=%+5.1fdeg pitch=%+5.1fdeg G=%s\r\n",
                    st,
                    (double)observerGetTauExtTrans(),
                    (double)observerGetTauMotorTrans(),
                    admittanceStateName(),
                    (double)admittanceGetAssistForce(),
                    (double)observerGetVTransMps(),
                    (double)observerGetSlopeDeg(),
                    (double)observerGetPitchDeg(),
                    observerGravityEnabled() ? "on" : "off");
    } else {
        uart_printf("OBS %-8s Fext=%+7.1fN  Mext=%+6.2fNm | v=%+5.2fm/s  "
                    "Fmot=%+7.1fN  slope=%+5.1fdeg pitch=%+5.1fdeg G=%s\r\n",
                    st,
                    (double)observerGetTauExtTrans(),
                    (double)observerGetTauExtYaw(),
                    (double)observerGetVTransMps(),
                    (double)observerGetTauMotorTrans(),
                    (double)observerGetSlopeDeg(),
                    (double)observerGetPitchDeg(),
                    observerGravityEnabled() ? "on" : "off");
    }
}

/* Continuous assist status: IDLE / FWD / BACK, the assist force applied,
 * and the operator push force being sensed. */
static void print_assist_status(void)
{
    if (!admittanceIsEnabled()) {
        uart_print("ASSIST: OFF    (send 'AS 1' to enable)\r\n");
        return;
    }
    /* push   = force you apply (observer Fext)
     * cmd    = assist force the loop commands (GAIN * push)
     * motorF = force the motor reports applying (Iq*Kt) — should track cmd;
     *          if cmd is high but motorF stays ~0, the ODrive Iq feedback
     *          isn't reaching us (check the GUI cyclic Iq message). */
    uart_printf("ASSIST: %-11s | push=%+6.1fN  cmd=%+6.1fN  motorF=%+6.1fN  "
                "v=%+5.2fm/s\r\n",
                admittanceStateName(),
                (double)observerGetTauExtTrans(),
                (double)admittanceGetAssistForce(),
                (double)observerGetTauMotorTrans(),
                (double)observerGetVTransMps());
}


/* Live pose + signed pitch, with the current thresholds, for tuning
 * POSE_DRIVE_ENTER_DEG / POSE_STANDING_ENTER_DEG. */
static void print_pose_pitch(void)
{
    uart_printf("POSE %-8s | pitch=%+6.1f deg   (DRIVE>%+.0f  STAND<%+.0f)%s\r\n",
                assistGetPoseName(), (double)assistGetPitchDeg(),
                (double)POSE_DRIVE_ENTER_DEG, (double)POSE_STANDING_ENTER_DEG,
                assistIsTilting() ? "  [TILTING-HELD]" : "");
}

static void print_mode(void)
{
    uart_printf("Pose: %-8s  pitch=%+.1f deg  (DRIVE>%+.0f STAND<%+.0f) | "
                "assist %s | motors %s | handle %s\r\n",
                assistGetPoseName(), (double)assistGetPitchDeg(),
                (double)POSE_DRIVE_ENTER_DEG, (double)POSE_STANDING_ENTER_DEG,
                admittanceIsEnabled() ? "ON" : "OFF",
                assistIsBraked() ? "BRAKED" : (assistDriveActive() ? "DRIVE" : "idle"),
                deadmanPressed() ? "PRESSED" : "RELEASED");
}

static void set_assist_enabled(bool en)
{
    if (en && !assistIsEnabled()) {
        uart_print("ERR: cannot enable — truck in STANDING. Tilt forward first.\r\n");
        return;
    }
    if (en) {
        if (motorControlEnableClosedLoop()) {
            uart_print("OK: ODrives in CLOSED_LOOP_CONTROL.\r\n");
        } else {
            uart_print("ERR: failed to arm — see ODrive status above.\r\n");
        }
    } else {
        motorControlDisableClosedLoop();
        uart_print("OK: ODrives in IDLE.\r\n");
    }
}

/* =====================================================================
 *  Banner + help
 * =====================================================================*/
static void print_banner(void)
{
    uart_print("\r\n");
    uart_print("================================================\r\n");
    uart_print(" STM RotaTruck — ODrive S1 (CAN bus motor ctrl)\r\n");
    uart_print("================================================\r\n");
    uart_print("Type 'H' for help. End each line with <enter>.\r\n");
}

static void print_help(void)
{
    uart_print(
        "Commands (case-insensitive, space-separated):\r\n"
        "\r\n"
        "Velocity (rev/s at motor shaft, signed = direction):\r\n"
        "  L V <rps>        Left  motor — hold velocity\r\n"
        "  R V <rps>        Right motor\r\n"
        "  B V <rps>        Both motors\r\n"
        "  L T <secs> <rps> Left  motor — drive at rps for secs, then stop\r\n"
        "  R T <secs> <rps> Right motor\r\n"
        "  B T <secs> <rps> Both motors\r\n"
        "\r\n"
        "Torque (N·m at motor shaft, signed):\r\n"
        "  L Q <nm>         Left  motor torque setpoint\r\n"
        "  R Q <nm>         Right motor torque setpoint\r\n"
        "  B Q <nm>         Both motors\r\n"
        "\r\n"
        "ODrive control:\r\n"
        "  E 1 / E 0        Arm / disarm CLOSED_LOOP_CONTROL on both axes\r\n"
        "  K 1 / K 0 / K A  Force IDLE / force CLOSED_LOOP / AUTO (pose-driven)\r\n"
        "  OD               ODrive status snapshot (axis state, Vbus, errors)\r\n"
        "  OD ?             same as 'OD'\r\n"
        "  OD C             Clear latched ODrive errors on both axes\r\n"
        "  S                Stop: disable assist + hold both motors at 0 (stays armed)\r\n"
        "\r\n"
        "Sensors / state:\r\n"
        "  P                Wheel positions / velocities / Iq (cached)\r\n"
        "  I                IMU sample (accel + gyro)\r\n"
        "  I S              Toggle ~10 Hz raw 6-DOF IMU stream (g, dps, tilt/plane)\r\n"
        "  IC               Calibrate IMU in current pose\r\n"
        "  I2C              Scan the I2C bus (lists IMU / LCD addresses)\r\n"
        "  I B              2 s still test: gyro bias + noise, |a| (assist off)\r\n"
        "\r\n"
        "Observer (operator-force estimate, read-only):\r\n"
        "  OB               One-shot: Fext (push N), Mext (twist Nm), v, Fmot\r\n"
        "  OB S             Toggle ~10 Hz observer telemetry stream\r\n"
        "  OB Z             Zero IMU pitch baseline (do it in DRIVE pose)\r\n"
        "  OB T             Tare the resting force estimate to 0\r\n"
        "  OB G             Toggle gravity comp (OFF for flat bench)\r\n"
        "  OB F             Toggle friction comp (OFF for free bench wheel)\r\n"
        "\r\n"
        "Power assist (torque loop — gains in assist_tuning.h):\r\n"
        "  AS 1 / AS 0      Enable / disable assist (push the truck to move)\r\n"
        "  AS               Assist status (force + per-wheel torque)\r\n"
        "  AS S             Toggle live assist stream: IDLE/FWD/BACK, push+assist N\r\n"
        "  PL / PL 1 / PL 0 Load + payload live log (5 Hz); handle press at STANDING accepts the load\r\n"
        "  W                Payload in use (LATCHED / MANUAL), latch + load status\r\n"
        "  W <kg> / W A     Manual payload (like the +/- buttons) / back to the latch\r\n"
        "                   Latch: assist ON at STANDING, then tip back -> the last\r\n"
        "                   valid standing load becomes the assist payload.\r\n"
        "  LD               Load-detect console: strain-gauge calibration (bench-rig\r\n"
        "                   commands d b v x t z c s u q g i ...), assist forced off.\r\n"
        "                   LD again to leave.\r\n"
        "  R A / R M        Autonomous / manual mode (auto self-calibrate+arm in DRIVE)\r\n"
        "  M                Print pose + pitch + motor state\r\n"
        "  M S              Toggle live pose/pitch stream (tune pose angles)\r\n"
        "  H or ?           This help\r\n"
        "\r\n"
        "Motor commands run only in DRIVE pose (tilt forward) unless you\r\n"
        "override via K 0.  Each ODrive must be configured via the Web GUI\r\n"
        "first: Node ID = 0 for LEFT, 1 for RIGHT, CAN baud = 250 kbit/s.\r\n");
}
