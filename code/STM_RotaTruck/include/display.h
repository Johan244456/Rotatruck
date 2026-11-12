#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 *  display.h — the operator status screen, a first-class system module.
 *
 *  Renders live system state to the 16x2 LCD (layout details in display.c):
 *    Line 1:  <UPRIGHT|DRIVE> <live scale kg (actual value also when out of range), or a fault code>
 *    Line 2:  <AUTO|MAN> <kg the assist uses: AUTO 0.7 x, MAN 1.0 x load> <ON|OFF|ERR>
 *             ("LD CONSOLE <load status>" in the LD console)
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