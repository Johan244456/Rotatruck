/*
 *  imu.c — IMU layer: ISM330DHCX (SparkFun SEN-19764) on I2C1.
 *
 *  Owns the I2C1 handle (shared with the 16x2 LCD), configures the sensor
 *  through ism330.c, rotates every reading into the CART frame (board.h
 *  IMU_AXIS_SIGN_*) and caches it for the pose machine, the observer and
 *  load detection.  The MPU6050 that this replaced has been removed.
 *
 *  Cart frame (what everything downstream was tuned in):
 *    pitch      = atan2(ay, az)   STANDING ~ -43.6 deg, DRIVING ~ -1.2 deg
 *    pitch rate = gx              (+ = towards DRIVING)
 *    yaw rate   = gz
 */

#include "imu.h"
#include "board.h"
#include "command.h"          /* uart_print / uart_printf */
#include "ism330.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <math.h>

/* =====================================================================
 *  HAL handle  (file-scope; HAL_I2C_MspInit picks it up by Instance)
 * =====================================================================*/
I2C_HandleTypeDef hi2c_imu;

#define G_TO_MS2            9.81f

/* Finite (short) timeout: a locked bus must return an error so the
 * recovery routine can run — HAL_MAX_DELAY would hang the firmware. */
#define I2C_TIMEOUT         25u

/* A real accelerometer at rest or moving reads ~1 g.  Several readings in
 * a row near 0 g mean the sensor lost its configuration (brown-out reset
 * puts it in power-down, outputs 0) even though I2C still ACKs. */
#define IMU_DEAD_ACCEL_G    0.3f
#define IMU_DEAD_COUNT      5

/* =====================================================================
 *  Cached state (cart frame)
 * =====================================================================*/
static bool     s_ok = false;               /* ISM330DHCX found + configured */
static float    gravity_offset_x_g = 0.0f;
static uint32_t s_last_ok_ms = 0;           /* HAL tick of the last good imuRead() */
static uint8_t  s_dead_count = 0;

static float ax_g = 0, ay_g = 0, az_g = 0;
static float gx_dps = 0, gy_dps = 0, gz_dps = 0;
static float ax_ms2_corrected = 0.0f;

/* =====================================================================
 *  Bus helpers
 * =====================================================================*/
/* Nine SCL pulses + a STOP: releases a sensor that is holding SDA low
 * after an interrupted transfer (MCU reset mid-read, or motor/CAN noise
 * glitching the bus).  Leaves PB8/PB9 as GPIO; HAL_I2C_Init's MspInit puts
 * them back on AF4. */
static void i2c_bus_clear_pins(void)
{
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_WritePin(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN, GPIO_PIN_SET);
    g.Pin = IMU_I2C_SCL_PIN; HAL_GPIO_Init(IMU_I2C_SCL_PORT, &g);
    g.Pin = IMU_I2C_SDA_PIN; HAL_GPIO_Init(IMU_I2C_SDA_PORT, &g);
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_SET);
        HAL_Delay(1);
    }
    /* STOP: SDA low->high while SCL is high. */
    HAL_GPIO_WritePin(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
}

static bool i2c_peripheral_init(void)
{
    hi2c_imu.Instance              = IMU_I2C_INSTANCE;
    hi2c_imu.Init.ClockSpeed      = IMU_I2C_CLOCK_HZ;
    hi2c_imu.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c_imu.Init.OwnAddress1     = 0;
    hi2c_imu.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c_imu.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c_imu.Init.OwnAddress2     = 0;
    hi2c_imu.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c_imu.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c_imu) != HAL_OK) return false;
    /* (The F407's I2C has no configurable analog/digital noise filter.) */
    return true;
}

/* Bus error recovery WITHOUT a board reset: a glitched transfer can latch
 * the STM32 I2C peripheral into a bus-error state until re-init.  The
 * ISM330DHCX keeps its configuration (only a power loss resets it, which
 * the dead-sensor check in imuRead() catches). */
static void i2c_bus_recover(void)
{
    HAL_I2C_DeInit(&hi2c_imu);
    i2c_bus_clear_pins();
    i2c_peripheral_init();
}

/* =====================================================================
 *  Init / probe
 * =====================================================================*/
#define IMU_BEGIN_TRIES       3
#define IMU_RETRY_PERIOD_MS   500U
static uint32_t s_retry_ms = 0;

bool imuBegin(void)
{
    i2c_bus_clear_pins();
    if (!i2c_peripheral_init()) return false;
    /* WHO_AM_I at 0x6B (or 0x6A) + configure.  Retried: right after an
     * MCU reset (e.g. an ST-Link upload) the sensor can miss the first
     * configuration sequence even though it ACKs. */
    for (int i = 0; i < IMU_BEGIN_TRIES && !s_ok; i++) {
        if (i > 0) {
            uart_printf("IMU: ISM330DHCX setup failed, retry %d/%d\r\n", i, IMU_BEGIN_TRIES - 1);
            i2c_bus_recover();
            HAL_Delay(20);
        }
        s_ok = ism330Begin();
    }
    s_retry_ms = HAL_GetTick();
    return s_ok;
}

/* Not configured (boot failure): try again every IMU_RETRY_PERIOD_MS from
 * imuRead(), so the IMU comes back by itself instead of staying dead until
 * the next reset. */
static void lazy_reinit(void)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_retry_ms) < IMU_RETRY_PERIOD_MS) return;
    s_retry_ms = now;
    if (hi2c_imu.State == HAL_I2C_STATE_RESET && !i2c_peripheral_init()) return;
    s_ok = ism330Begin();
    if (s_ok) uart_print("IMU: ISM330DHCX configured (recovered).\r\n");
}

const char *imuGetSourceName(void) { return s_ok ? "ISM330DHCX" : "none"; }

bool imuIsPresent(void) { return s_ok && ism330IsPresent(); }

/* I2C1 bus scan — diagnostic only.  Lists every 7-bit address that ACKs
 * and names the parts expected on this bus.  For an ISM330DHCX candidate
 * (0x6A/0x6B) it also reads WHO_AM_I (reg 0x0F, expected 0x6B) so a wiring
 * check confirms the chip itself, not just "something answered". */
#define ISM330_REG_WHO_AM_I   0x0Fu
#define ISM330_WHO_AM_I_VAL   0x6Bu

void imuScanBus(void)
{
    if (hi2c_imu.State == HAL_I2C_STATE_RESET) {
        uart_print("I2C scan: bus not initialised.\r\n");
        return;
    }
    uart_print("I2C scan (I2C1: SDA=PB9/D14, SCL=PB8/D15):\r\n");
    unsigned found = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (HAL_I2C_IsDeviceReady(&hi2c_imu, (uint16_t)(a << 1), 1, 2) != HAL_OK) continue;
        found++;
        if (a == 0x6A || a == 0x6B) {
            uint8_t who = 0;
            if (HAL_I2C_Mem_Read(&hi2c_imu, (uint16_t)(a << 1), ISM330_REG_WHO_AM_I,
                                 I2C_MEMADD_SIZE_8BIT, &who, 1, I2C_TIMEOUT) == HAL_OK) {
                uart_printf("  0x%02X  WHO_AM_I=0x%02X  %s\r\n", a, who,
                            (who == ISM330_WHO_AM_I_VAL) ? "-> ISM330DHCX OK"
                                                         : "-> unexpected ID");
            } else {
                uart_printf("  0x%02X  ISM330DHCX? (WHO_AM_I read failed)\r\n", a);
            }
            continue;
        }
        const char *name = "unknown device";
        if (a == 0x27 || a == 0x3F) name = "LCD backpack (PCF8574)";
        uart_printf("  0x%02X  %s\r\n", a, name);
    }
    uart_printf("I2C scan: %u device(s) found.\r\n", found);
}

/* =====================================================================
 *  Sample + accessor functions
 * =====================================================================*/
/* One ISM330DHCX read, rotated into the cart frame (board.h signs). */
static bool read_cart_frame(void)
{
    ism330_sample_t s;
    if (!ism330Read(&s)) return false;
    ax_g   = s.ax_g   * IMU_AXIS_SIGN_X;
    ay_g   = s.ay_g   * IMU_AXIS_SIGN_Y;
    az_g   = s.az_g   * IMU_AXIS_SIGN_Z;
    gx_dps = s.gx_dps * IMU_AXIS_SIGN_X;
    gy_dps = s.gy_dps * IMU_AXIS_SIGN_Y;
    gz_dps = s.gz_dps * IMU_AXIS_SIGN_Z;
    return true;
}

bool imuRead(void)
{
    if (!s_ok) {
        lazy_reinit();
        if (!s_ok) return false;
    }
    bool ok = read_cart_frame();
    if (!ok) {                          /* bus glitch: recover + one retry */
        i2c_bus_recover();
        ok = read_cart_frame();
    }
    if (!ok) return false;

    /* Dead-sensor check: reads succeed but the accel is ~0 g. */
    if (sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g) < IMU_DEAD_ACCEL_G) {
        if (++s_dead_count >= IMU_DEAD_COUNT) {
            s_dead_count = 0;
            uart_print("IMU: ISM330DHCX reads ~0 g - reconfiguring.\r\n");
            s_ok = ism330Begin();
            s_retry_ms = HAL_GetTick();        /* if it failed, lazy_reinit retries */
        }
        return false;                   /* don't publish a bogus sample */
    }
    s_dead_count = 0;

    ax_ms2_corrected = (ax_g - gravity_offset_x_g) * G_TO_MS2;
    s_last_ok_ms = HAL_GetTick();
    return true;
}

bool imuGetLatest(float acc_g[3], float gyro_dps[3], uint32_t *t_ms)
{
    if (!s_ok || s_last_ok_ms == 0) return false;
    acc_g[0] = ax_g;    acc_g[1] = ay_g;    acc_g[2] = az_g;
    gyro_dps[0] = gx_dps; gyro_dps[1] = gy_dps; gyro_dps[2] = gz_dps;
    if (t_ms) *t_ms = s_last_ok_ms;
    return true;
}

/* =====================================================================
 *  Calibration  (cart still + level)
 * =====================================================================*/
/* Only the X (forward) axis is calibrated and corrected - the cart must be
 * level and still so this captures the static gravity on X. Y and Z accel
 * are reported raw. */
void imuCalibrate(uint16_t samples)
{
    float    sum_ax = 0.0f;
    uint16_t got    = 0;

    for (uint16_t i = 0; i < samples; i++) {
        if (imuRead()) {
            sum_ax += ax_g;
            got++;
        }
        HAL_Delay(5);
    }
    if (got > 0) {
        gravity_offset_x_g = sum_ax / got;
    }
}

float imuGetAxMs2(void)  { return ax_ms2_corrected; }
float imuGetGzDps(void)  { return gz_dps; }
float imuGetAxG(void)    { return ax_g; }
float imuGetAyG(void)    { return ay_g; }
float imuGetAzG(void)    { return az_g; }
float imuGetGxDps(void)  { return gx_dps; }
float imuGetGyDps(void)  { return gy_dps; }

/* Mounting: the cart tilts (STANDING <-> DRIVING) about the cart X axis, so
 * gravity swings between Y and Z.  Measured with the ISM330DHCX 2026-11-10:
 * STANDING ~ -43.6 deg, DRIVING ~ -1.2 deg.  If the IMU is remounted, fix
 * the IMU_AXIS_SIGN_* in board.h (check with `I S`), not these lines. */
float imuGetPitchDeg(void)     { return atan2f(ay_g, az_g) * 57.29578f; }
float imuGetPitchRateDps(void) { return gx_dps; }
