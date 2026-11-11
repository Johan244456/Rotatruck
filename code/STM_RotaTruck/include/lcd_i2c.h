#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 *  lcd_i2c.h — non-blocking 16x2 HD44780 driver (PCF8574 I2C backpack).
 *
 *  lcdBegin() does the one-time power-on init (blocking, boot only).
 *  After that the display is driven WITHOUT blocking the control loop:
 *    - lcdSetLines() just stores the desired 16-char content (cheap).
 *    - lcdService(), called every main-loop pass, pushes ONE character to
 *      the panel per call (a few short I2C writes, no HAL_Delay) and is
 *      rate-limited, so a full refresh is spread over many passes instead
 *      of stalling the loop while a full 32-character refresh is written.
 *  Safe if the LCD is absent (lcdIsPresent() stays false; calls are no-ops).
 * =====================================================================*/

bool lcdBegin(void);
bool lcdIsPresent(void);

/* Set the desired two lines (each shown left-justified, padded/truncated
 * to 16 chars).  Cheap: only flags a refresh if the content changed. */
void lcdSetLines(const char *line0, const char *line1);

/* Pump the incremental refresh — call once per main-loop pass. */
void lcdService(void);

#ifdef __cplusplus
}
#endif

#endif /* LCD_I2C_H */