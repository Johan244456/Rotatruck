/*
 *  display.c — operator status screen.
 *
 *  Pulls live state from the rest of the system and pushes it to the
 *  non-blocking LCD.  Building the lines is cheap (a few getters + two
 *  snprintf); lcdSetLines() only flags a repaint when the text actually
 *  changes, and lcdService() spreads that repaint over many loop passes.
 */

#include "display.h"
#include "lcd_i2c.h"
#include "assist_control.h"
#include "admittance.h"
#include "observer.h"
#include "odrive_can.h"
#include "loadcell.h"
#include "payload.h"
#include "deadman.h"
#include "board.h"

#include <stdio.h>
#include <string.h>

static const char *pose_short(void)
{
    return (assistGetPose() == POSE_DRIVE) ? "DRIVE" : "STAND";
}

/* Map the admittance state *name* to a short LCD label. Kept as string
 * compares to avoid pulling in the admittance enum - if those names change,
 * update here. BRAKING is shown as HOLD on purpose. */
static const char *assist_short(void)
{
    /* Handle switch released with the assist on: ramping down / held. */
    if (admittanceIsEnabled() && !deadmanPressed()) return "STOP";
    const char *s = admittanceStateName();
    if (strcmp(s, "ASSIST FWD")  == 0) return "FWD";
    if (strcmp(s, "ASSIST BACK") == 0) return "BACK";
    if (strcmp(s, "HOLD")        == 0) return "HOLD";
    if (strcmp(s, "BRAKING")     == 0) return "HOLD";
    return "IDLE";   /* IDLE / OFF / CAL */
}

/* ERROR if either ODrive reports a non-zero axis error word. */
static bool motor_error_present(void)
{
    const odrive_state_t *l = odriveGetState(ODRIVE_NODE_ID_LEFT);
    const odrive_state_t *r = odriveGetState(ODRIVE_NODE_ID_RIGHT);
    if (l && l->seen && l->axis_error) return true;
    if (r && r->seen && r->axis_error) return true;
    return false;
}

/* Top-right: the payload the assist is using (latched from load detection,
 * or 'M' suffix = manual / buttons), unless a motor fault overwrites it
 * with ERR. */
static void right_field(char *out, size_t n)
{
    if (motor_error_present()) {
        snprintf(out, n, "ERR");
        return;
    }
    bool manual = payloadIsManual();
    int kg = (int)(payloadGetLoadKg() + 0.5f);    /* the load the operator set / accepted */
    if (kg < 0)   kg = 0;
    if (kg > 999) kg = 999;
    snprintf(out, n, manual ? "%dkgM" : "%dkg", kg);
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

    /* Row 0: ASSIST:<ON|OFF>  <payload kg | ERR>
     * Row 1: <pose> <assist state> <load-detect status>
     *        e.g. "DRIVE FWD  HELD " / "STAND HOLD OK   " / "STAND IDLE NOCAL"
     *        In the LD console: "LD CONSOLE <status>" */
    char line0[17], line1[17], right[8];
    right_field(right, sizeof right);
    snprintf(line0, sizeof line0, "ASSIST:%-3s%6s",
             admittanceIsEnabled() ? "ON" : "OFF", right);
    if (loadcellConsoleActive()) {
        snprintf(line1, sizeof line1, "LD CONSOLE %-5s", loadcellStatusShort());
    } else {
#if LCD_ROW1_SHOW_HANDLE
        /* Third field = handle (dead-man) switch: PRESS / RELSD. */
        snprintf(line1, sizeof line1, "%-5s %-4s %-5s", pose_short(), assist_short(),
                 deadmanPressed() ? "PRESS" : "RELSD");
#else
        /* Third field = load-detect status (OK, NOCAL, NODAT, HELD ...). */
        snprintf(line1, sizeof line1, "%-5s %-4s %-5s", pose_short(), assist_short(),
                 loadcellStatusShort());
#endif
    }

    lcdSetLines(line0, line1);   /* cheap: flags a repaint only on change */
    lcdService();                /* push one byte of the repaint, if any  */
}