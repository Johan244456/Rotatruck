#include "imu.h"
#include <Wire.h>

static const uint8_t MPU_ADDR         = 0x68;
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// ±2g range -> 16384 LSB/g. ±250 deg/s range -> 131 LSB/(deg/s).
static const float ACCEL_LSB_PER_G   = 16384.0f;
static const float GYRO_LSB_PER_DPS  = 131.0f;
static const float G_TO_MS2          = 9.81f;

// Raw gravity offset measured at calibration (assumes cart still + level).
// Stored in raw int16 LSB units on the X axis.
static int16_t gravity_offset_x_raw = 0;

// Latest cached readings
static float ax_g = 0, ay_g = 0, az_g = 0;
static float gx_dps = 0, gy_dps = 0, gz_dps = 0;
static float ax_ms2_corrected = 0;

static bool readBurstRaw(int16_t& ax, int16_t& ay, int16_t& az,
                         int16_t& tRaw,
                         int16_t& gx, int16_t& gy, int16_t& gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (Wire.available() < 14) return false;

  ax   = (Wire.read() << 8) | Wire.read();
  ay   = (Wire.read() << 8) | Wire.read();
  az   = (Wire.read() << 8) | Wire.read();
  tRaw = (Wire.read() << 8) | Wire.read();
  gx   = (Wire.read() << 8) | Wire.read();
  gy   = (Wire.read() << 8) | Wire.read();
  gz   = (Wire.read() << 8) | Wire.read();
  return true;
}

bool imuBegin() {
  Wire.begin();
  Wire.setClock(400000UL); // 400 kHz fast mode
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00); // wake
  return Wire.endTransmission(true) == 0;
}

void imuCalibrate(uint16_t samples) {
  long sum_ax = 0;
  uint16_t got = 0;
  for (uint16_t i = 0; i < samples; i++) {
    int16_t ax, ay, az, t, gx, gy, gz;
    if (readBurstRaw(ax, ay, az, t, gx, gy, gz)) {
      sum_ax += ax;
      got++;
    }
    delay(5);
  }
  if (got > 0) {
    gravity_offset_x_raw = (int16_t)(sum_ax / got);
  }
}

bool imuRead() {
  int16_t ax, ay, az, t, gx, gy, gz;
  if (!readBurstRaw(ax, ay, az, t, gx, gy, gz)) return false;

  ax_g = ax / ACCEL_LSB_PER_G;
  ay_g = ay / ACCEL_LSB_PER_G;
  az_g = az / ACCEL_LSB_PER_G;
  gx_dps = gx / GYRO_LSB_PER_DPS;
  gy_dps = gy / GYRO_LSB_PER_DPS;
  gz_dps = gz / GYRO_LSB_PER_DPS;

  int16_t ax_corr = ax - gravity_offset_x_raw;
  ax_ms2_corrected = (ax_corr / ACCEL_LSB_PER_G) * G_TO_MS2;
  return true;
}

float imuGetAxMs2() { return ax_ms2_corrected; }
float imuGetGzDps() { return gz_dps; }
float imuGetAxG()   { return ax_g; }
float imuGetAyG()   { return ay_g; }
float imuGetAzG()   { return az_g; }
