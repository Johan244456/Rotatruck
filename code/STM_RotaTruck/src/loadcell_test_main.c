/*
 *  loadcell_test_main.c — standalone load-detection bring-up for the
 *  STM32F407G-DISC1.
 *
 *  Built ONLY by [env:loadcell_test].  Runs the real load-detection code
 *  (loadcell.c) with the IMU pose gating (imu.c), and nothing else: no CAN,
 *  no motors, no assist.  Use it for LOAD_DETECTION.md "Bring-up" steps 1-5
 *  before running the full firmware.
 *
 *  Wiring (board.h):  HX711 #1 (CH1, middle strip)    DT PE11  SCK PE12
 *                     HX711 #2 (CH2, left + right)    DT PE13  SCK PE14
 *                     both VCC -> 3V, GND -> GND;  IMU on PB8/PB9 as before.
 *                     Status LED: green LD4 (PD12) on the board.
 *
 *  Console (PlatformIO serial monitor over RTT, see bringup_io.h): the
 *  load-detect console is ON from boot, so the bench-rig commands work
 *  directly - type a command and press Enter (b, d, v, x, t, z, c, s, q, ...,
 *  see LOAD_DETECTION.md).  The 10 Hz CSV streams while it is on.
 *  `LD` toggles the console (`LD 1` / `LD 0` force it, as the host GUIs send);
 *  with it off, only load status changes are printed.
 */

#include "stm32f4xx_hal.h"
#include "board.h"
#include "command.h"
#include "bringup_io.h"
#include "imu.h"
#include "loadcell.h"

#include <string.h>

#define IMU_READ_PERIOD_MS  10U     /* 100 Hz, as observer.c does in the full firmware */

/* LCTEST_IMU 0 (platformio.ini): the IMU is never started or read, even if it
 * is wired - load detection runs exactly as with no IMU (bench calibration,
 * pose gating off 'q 0'). 1 = use the ISM330DHCX for pose gating. */
#ifndef LCTEST_IMU
#define LCTEST_IMU 1
#endif

void SystemClock_Config(void);      /* clock_f4.c */

/* `LD`, `LD 0`, `LD 1` (case-insensitive) -> true and sets *on. */
static bool parse_ld(const char *line, bool *on)
{
    if ((line[0] != 'L' && line[0] != 'l') || (line[1] != 'D' && line[1] != 'd')) return false;
    const char *p = line + 2;
    while (*p == ' ') p++;
    if (*p == '\0')                  { *on = !loadcellConsoleActive(); return true; }
    if (p[0] == '1' && p[1] == '\0') { *on = true;  return true; }
    if (p[0] == '0' && p[1] == '\0') { *on = false; return true; }
    return false;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    bringupIoInit();

    uart_print("\r\n=== RotaTruck F407 load-detection test ===\r\n"
               "HX711 CH1 DT=PE11 SCK=PE12 | CH2 DT=PE13 SCK=PE14 | LED = green LD4\r\n");

#if LCTEST_IMU
    if (imuBegin()) {
        uart_printf("IMU: %s OK (pose gating).\r\n", imuGetSourceName());
    } else {
        uart_print("IMU: none detected - load status will be IMU_FAULT until it answers.\r\n");
    }
#else
    uart_print("IMU: DISABLED in this build (LCTEST_IMU=0) - not started or read even if wired.\r\n"
               "     Use pose gating OFF ('q 0') for live readings, or 'q f s/r' to simulate a pose.\r\n");
#endif

    loadcellInit();          /* prints "LOAD: HX711 CH1 ok/MISSING, CH2 ..." */
    uart_print("First checks: 'd' = HX711 diagnostic (both channels), 'b' = bridge report.\r\n"
               "'LD' toggles the console / CSV stream.\r\n");
    loadcellSetConsole(true);

    uint32_t last_imu = HAL_GetTick();
    char line[64];

    while (1) {
        uint32_t now = HAL_GetTick();
#if LCTEST_IMU
        if ((uint32_t)(now - last_imu) >= IMU_READ_PERIOD_MS) {
            last_imu = now;
            (void)imuRead();     /* loadcell.c reads it via imuGetLatest() */
        }
#else
        (void)last_imu;
        (void)now;
#endif

        loadcellUpdate();

        if (bringupGetLine(line, sizeof line)) {
            bool on;
            if (parse_ld(line, &on)) {
                loadcellSetConsole(on);
                if (!on) uart_print("Console off - 'LD' to turn it back on.\r\n");
            } else if (loadcellConsoleActive()) {
                loadcellHandleLine(line);
            } else {
                uart_print("Console is off - type 'LD' first.\r\n");
            }
        }
    }
}
