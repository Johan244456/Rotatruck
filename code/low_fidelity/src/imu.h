#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

// Initialise I2C + MPU6050 (wake from sleep). Returns true on success.
bool imuBegin();

// Collect `samples` readings (cart must be still + level) to compute
// gravity offset on the X axis. Call once from setup after imuBegin().
void imuCalibrate(uint16_t samples = 200);

// Read latest accel + gyro into internal cache. Returns true on success.
bool imuRead();

// Last cached, gravity-corrected forward acceleration (m/s^2).
float imuGetAxMs2();

// Last cached yaw rate (degrees/second) around Z.
float imuGetGzDps();

// Last cached raw-g values (for debug/printing).
float imuGetAxG();
float imuGetAyG();
float imuGetAzG();

#endif // IMU_H
