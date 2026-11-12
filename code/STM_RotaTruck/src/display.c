/*
 *  display.c — operator status screen (16x2 LCD).
 *
 *    line 1:  <position> <live scale reading>      e.g. "UPRIGHT  30.2kg"
 *    line 2:  <AUTO|MAN> <assist kg> <ON|OFF|ERR>  e.g. "AUTO  21.0kg ON"
 *
 *  Position (the motor pose machine, assist_control.c):
 *    DRIVE    fully tipped back (within POSE_DRIVE_FULL_ENTER_DEG of `q r`):
 *             the assist may drive the wheels
 *    UPRIGHT  everything else (upright, partly tipped, or no IMU): wheels held
 *  Scale reading: "30.2kg" settled, "~30.2kg" not settled / not weighing
 *  (moving, tipped back), "!" over the rated load; out of the plausible range
 *  shows the actual value ("-12.1kg", "2480kg") instead of IMPL; the other
 *  faults show their code (NOCAL RAIL FROZ NODAT IMU!).
 *  Line 2: the weight the assist is sized for (AUTO: LOAD_ASSIST_SCALE 0.7 x the
 *  measured load; MAN: MANUAL_ASSIST_SCALE 1.0 x the typed / default load),
 *  AUTO = from the scale (read upright / handle click), MAN = power-up default
 *  or `W <kg>`; then the assist ON / OFF, or ERR if an ODrive reports a fault.
 *  In the LD console line 2 reads "LD CONSOLE <status>".
 *
 *  Building the lines is cheap; lcdSetLines() only flags a repaint when the
 *  text changes, and lcdService() spreads the repaint over many loop passes.
 */

#include "display.h"
#include "lcd_i2c.h"
#include "assist_control.h"
#include "admittance.h"
#include "odrive_can.h"
#include "loadcell.h"
#include "payload.h"
#include "board.h"

#include <stdio.h>
#include <string.h>

/* ERROR if either ODrive reports a non-zero axis error word. */
static bool motor_error_present(void)
{
    const odrive_state_t *l = odriveGetState(ODRIVE_NODE_ID_LEFT);
    const odrive_state_t *r = odriveGetState(ODRIVE_NODE_ID_RIGHT);
    if (l && l->seen && l->axis_error) return true;
    if (r && r->seen && r->axis_error) return true;
    return false;
}

/* Position, <= 7 chars: the motor pose machine's two states. */
static void position_text(char *out, size_t n)
{
    snprintf(out, n, (assistGetPose() == POSE_DRIVE) ? "DRIVE" : "UPRIGHT");
}

/* Live scale reading, <= 8 chars: "30.2kg" settled, "~30.2kg" not settled /
 * not weighing, "!160kg" over the rated load.  Out of the plausible range
 * (IMPLAUSIBLE) shows the actual value anyway ("-12.1kg", "2480kg") instead of
 * IMPL; the other faults keep their code (NOCAL RAIL FROZ NODAT IMU!). */
static void scale_text(char *out, size_t n)
{
    char flag;
    switch (loadcellStatus()) {
    case LC_ST_OK:
    case LC_ST_IMPLAUSIBLE: flag = ' '; break;          /* actual value, no IMPL code */
    case LC_ST_SETTLING:
    case LC_ST_HELD:        flag = '~'; break;
    case LC_ST_OVERLOAD:    flag = '!'; break;
    default:                snprintf(out, n, "%s", loadcellStatusShort()); return;   /* fault code */
    }
    float kg = loadcellMeasuredKg();
    if (kg >  9999.0f) kg =  9999.0f;
    if (kg < -999.0f)  kg = -999.0f;
    if (kg > -9.95f && kg < 99.95f) snprintf(out, n, "%c%.1fkg", flag, (double)kg);
    else                            snprintf(out, n, "%c%.0fkg", flag, (double)kg);
}

bool displayInit(void)
{
    if (!lcdBegin()) return false;
    lcdSetLines("RotaTruck", "Starting...");
    return true;
}

void displayUpdate(void)
{
    if (!lcdIsPresent()) return;

    char pos[10], scale[12], line0[24], line1[24];
    position_text(pos, sizeof pos);
    scale_text(scale, sizeof scale);
    snprintf(line0, sizeof line0, "%-7s %8s", pos, scale);

    if (loadcellConsoleActive()) {
        snprintf(line1, sizeof line1, "LD CONSOLE %-5s", loadcellStatusShort());
    } else {
        float kg = payloadGetKg();                    /* what the assist is sized for */
        if (kg < 0.0f)   kg = 0.0f;
        if (kg > 999.0f) kg = 999.0f;
        const char *state = motor_error_present() ? "ERR"
                          : admittanceIsEnabled() ? "ON" : "OFF";
        snprintf(line1, sizeof line1, "%-4s %5.1fkg %-3s",
                 payloadIsManual() ? "MAN" : "AUTO", (double)kg, state);
    }

    lcdSetLines(line0, line1);   /* pads / truncates to 16; repaints only on change */
    lcdService();                /* push one byte of the repaint, if any  */
}
