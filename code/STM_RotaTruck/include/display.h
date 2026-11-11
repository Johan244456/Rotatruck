#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 *  display.h — the operator status screen, a first-class system module.
 *
 *  Renders live system state to the 16x2 LCD:
 *    Row 0:  ASSIST:<ON|OFF> ........ <assist payload kg: latched from load
 *            detection, or 'M' suffix = manual / buttons; ERR if a motor fault>
 *    Row 1:  <STAND|DRIVE> <FWD|BACK|HOLD|IDLE|STOP> <third field>
 *            third field = handle switch PRESS / RELSD (LCD_ROW1_SHOW_HANDLE 1)
 *            or load status OK SETL NOCAL RAIL FROZ NODAT IMPL OVER HELD IMU!
 *            ("LD CONSOLE <load status>" in the LD console)
 *    Row 1:  <STAND|DRIVE> .......... <IDLE|FWD|BACK|HOLD>
 *
 *  displayUpdate() is called every main-loop pass; it samples the other
 *  subsystems (pose / admittance / observer / odrive) and drives
 *  the non-blocking LCD.  No blocking, no separate timing of its own.
 * =====================================================================*/

/* Returns true if the LCD was detected on the I2C bus. */
bool displayInit(void);
void displayUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_H */