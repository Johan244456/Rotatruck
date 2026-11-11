#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* IMU layer — ISM330DHCX 6-DoF IMU (SparkFun SEN-19764, Qwiic) on I2C1.
 *
 * Wiring (STM RotaTruck):
 *   3V3  -> 3V3   (NOT 5 V - the chip is rated to 3.6 V, no regulator on the board)
 *   GND  -> GND
 *   SDA  -> D14   (PB9, I2C1_SDA)
 *   SCL  -> D15   (PB8, I2C1_SCL)
 *   ADR jumper left as shipped -> address 0x6B (0x6A also probed)
 * I2C1 is shared with the 16x2 LCD backpack (0x27 / 0x3F).
 *
 * Every reading is rotated into the CART frame (board.h IMU_AXIS_SIGN_*), so
 * pose detection, tilt protection, the observer and load detection all see
 * the same axes.  Used for chassis pitch, pitch rate and yaw rate; the assist
 * force estimate comes from the momentum observer, not the IMU. */

/* Initialise I2C1 (with a stuck-bus clear) and configure the ISM330DHCX.
 * True if the sensor was found and configured. */
bool imuBegin(void);

/* "ISM330DHCX" or "none". */
const char *imuGetSourceName(void);

/* Latest cached sample (the one the observer read, ~100 Hz) in the cart
 * frame: accel in g, gyro in deg/s, plus the HAL tick it was read at.
 * Does NOT touch the bus.  False if no IMU / never read.  Used by the load
 * detection pose classifier (loadcell.c), which checks the age itself. */
bool imuGetLatest(float acc_g[3], float gyro_dps[3], uint32_t *t_ms);

/* True if the ISM330DHCX ACKs on the bus right now.  Cheap probe. */
bool imuIsPresent(void);

/* Diagnostic: scan I2C1 and print every responding address (identifies the
 * ISM330DHCX via WHO_AM_I and the LCD backpack).  Call after imuBegin().
 * Backs the `I2C` UART command and the boot-time scan. */
void imuScanBus(void);

/* Collect <samples> readings (cart still + level) to compute the gravity
 * offset on the forward (X) axis.  Call after imuBegin(). */
void imuCalibrate(uint16_t samples);

/* Read one sample into the internal cache (cart frame).  Recovers a stuck
 * bus once, and reconfigures the sensor if it reads ~0 g (lost its
 * configuration).  Returns true on a good sample. */
bool imuRead(void);

/* Latest cached, gravity-corrected forward acceleration (m/s²). */
float imuGetAxMs2(void);

/* Latest cached yaw rate around Z (deg/s). */
float imuGetGzDps(void);

/* Latest cached values in g / dps, cart frame (debug / printing). */
float imuGetAxG(void);
float imuGetAyG(void);
float imuGetAzG(void);
float imuGetGxDps(void);
float imuGetGyDps(void);

/* Chassis pitch (deg) and pitch rate (deg/s) from the latest cached read.
 * DRIVING (~-1 deg) reads more positive than STANDING (~-44 deg). */
float imuGetPitchDeg(void);
float imuGetPitchRateDps(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
