#ifndef ISM330_H
#define ISM330_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ISM330DHCX 6-DoF IMU driver (SparkFun SEN-19764, Qwiic Micro).
 *
 * Low-level register access only, in the SENSOR frame.  imu.c owns the I2C1
 * handle (shared with the LCD), calls ism330Begin() and rotates readings
 * into the cart frame - use the imu.h API in application code.
 *
 * Wiring: 3V3 (NOT 5 V), GND, SDA -> PB9, SCL -> PB8.  ADR jumper as
 * shipped = I2C address 0x6B (0x6A, jumper cut, is also probed).
 *
 * Config: accel ±2 g, gyro ±250 dps, both at 208 Hz, block-data-update on
 * (no torn high/low bytes). */

typedef struct {
    float ax_g, ay_g, az_g;        /* acceleration, g     */
    float gx_dps, gy_dps, gz_dps;  /* angular rate, deg/s */
} ism330_sample_t;

/* Probe WHO_AM_I (expects 0x6B), soft-reset and configure.  True on success. */
bool ism330Begin(void);

/* True once ism330Begin() has succeeded. */
bool ism330IsReady(void);

/* Cheap bus probe: true if the chip ACKs at its address right now. */
bool ism330IsPresent(void);

/* 7-bit I2C address found by ism330Begin() (0x6B or 0x6A), 0 if none. */
uint8_t ism330Addr(void);

/* One 12-byte burst (gyro XYZ then accel XYZ), raw int16 counts. */
bool ism330ReadRaw(int16_t *ax, int16_t *ay, int16_t *az,
                   int16_t *gx, int16_t *gy, int16_t *gz);

/* Same burst converted to g / deg/s. */
bool ism330Read(ism330_sample_t *s);

/* Die temperature, deg C. */
bool ism330ReadTempC(float *temp_c);

#ifdef __cplusplus
}
#endif

#endif /* ISM330_H */
