#ifndef LOADCELL_H
#define LOADCELL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 *  loadcell — automatic payload detection (spec FR5).
 *
 *  Port of P102_LoadDetect_G491RE v4.2 (Code_STM/P102_LoadDetect_Main)
 *  into the RotaTruck firmware as an isolated subsystem:
 *    - 2 x HX711 (CH1 middle strip POISSON_HALF, CH2 left+right POISSON_FULL),
 *      pins in board.h (LC_*).  Bit-banged, non-blocking in normal running.
 *    - Same signal chain, views (x 0/1/2/3), stable-hold / smooth filters,
 *      IMU pose gating with rest zero, payload status codes, 0.9 safety
 *      factor, flash settings (last page, same layout v6 as the bench rig).
 *    - Pose comes from the shared IMU layer (imu.c, ISM330DHCX) instead of
 *      its own I2C driver.
 *
 *  It only MEASURES: status + LOAD_ASSIST_SCALE x measured load (0 on any fault or before
 *  calibration).  payload.c latches the last valid standing value into the
 *  assist on STANDING -> tipped back (if the assist was switched on at
 *  STANDING), with the +/- buttons as the manual fallback.
 *
 *  Serial: the bench-rig command set (d, b, v, x, t, z, c, s, u, q, g, i,
 *  n, f, k, m, h, a, p, r) and its 10 Hz CSV are available only inside the
 *  load-detect console, entered with `LD` (see command.c).  The console is a
 *  maintenance mode: the assist is forced off and refused while it is on,
 *  because calibration commands block the main loop for seconds.
 * =====================================================================*/

/* Payload status codes (same numbers as the bench rig CSV). */
typedef enum {
    LC_ST_OK = 0, LC_ST_SETTLING, LC_ST_NO_CAL, LC_ST_RAILED, LC_ST_FROZEN,
    LC_ST_NO_DATA, LC_ST_IMPLAUSIBLE, LC_ST_OVERLOAD, LC_ST_HELD, LC_ST_IMU_FAULT
} loadcell_status_t;

/* Pins, flash settings, zero.  Call after imuBegin() and commandSetup().
 * Blocks ~1-2 s only if no zero is saved (power-up tare). */
void  loadcellInit(void);

/* Every main-loop pass.  Non-blocking: reads an HX711 only when it has a
 * conversion ready (10 Hz), interrupts off ~60 us per read. */
void  loadcellUpdate(void);

/* Load-detect console (LD). */
bool  loadcellConsoleActive(void);
void  loadcellSetConsole(bool on);
void  loadcellHandleLine(const char *line);   /* one bench-rig command line */

/* Status / results. */
loadcell_status_t loadcellStatus(void);
const char *loadcellStatusName(void);         /* "OK", "NO_CAL", ...        */
const char *loadcellStatusShort(void);        /* <= 5 chars, for the LCD    */
bool  loadcellValid(void);                    /* OK, OVERLOAD or HELD       */
float loadcellAssistKg(void);                 /* LOAD_ASSIST_SCALE x measured, 0 on fault */
float loadcellMeasuredKg(void);               /* latest reading, any status */
/* Which value reaches the assist (latch / manual) is decided in payload.c. */

/* Pose as load detection sees it (IMU gravity vs the `q s` / `q r` references). */
bool  loadcellImuOk(void);
bool  loadcellPoseStanding(void);             /* within ~5 deg of the standing pose */
bool  loadcellPoseTipped(void);               /* >= 15 deg back from it (driving position) */
float loadcellStandDevDeg(void);              /* degrees away from the standing pose */
float loadcellRestDevDeg(void);               /* degrees away from the FULL tipped-back pose (q r) */
const char *loadcellPoseName(void);

/* Manual zero from the handle triple click: only at / near tipped back, uses
 * samples from before first_press_ms, RAM only.  False (reason printed) if refused. */
bool  loadcellZeroFromHandle(uint32_t first_press_ms);

#ifdef __cplusplus
}
#endif

#endif /* LOADCELL_H */
