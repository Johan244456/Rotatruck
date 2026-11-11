/*
 *  ism330.c — ISM330DHCX 6-DoF IMU driver (evaluation; see ism330.h).
 *
 *  Register map / scale factors from the ST ISM330DHCX datasheet:
 *    WHO_AM_I  0x0F = 0x6B
 *    CTRL1_XL  0x10  ODR_XL[7:4]  FS_XL[3:2]   (00 = ±2 g)
 *    CTRL2_G   0x11  ODR_G[7:4]   FS_G[3:1]    (000 = ±250 dps)
 *    CTRL3_C   0x12  BDU[6]  IF_INC[2]  SW_RESET[0]
 *    OUT_TEMP  0x20..0x21  (256 LSB/°C, 0 = 25 °C)
 *    OUTX_L_G  0x22..0x27  gyro  X/Y/Z, little-endian
 *    OUTX_L_A  0x28..0x2D  accel X/Y/Z, little-endian
 *    Sensitivity: ±2 g -> 0.061 mg/LSB, ±250 dps -> 8.75 mdps/LSB
 */

#include "ism330.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c_imu;      /* I2C1, initialised by imuBegin() */

/* 0x6B = SparkFun default (ADR jumper intact), 0x6A = ADR jumper cut.
 * Probed in that order (as in the P102 load-detect driver). */
static uint8_t s_addr7 = 0x6Bu;
#define ISM_ADDR_HAL         ((uint16_t)(s_addr7 << 1))

#define REG_WHO_AM_I         0x0Fu
#define REG_CTRL1_XL         0x10u
#define REG_CTRL2_G          0x11u
#define REG_CTRL3_C          0x12u
#define REG_OUT_TEMP_L       0x20u
#define REG_OUTX_L_G         0x22u

#define WHO_AM_I_VAL         0x6Bu

#define CTRL3_SW_RESET       0x01u
#define CTRL3_IF_INC         0x04u      /* auto-increment address on burst reads */
#define CTRL3_BDU            0x40u      /* block data update: H/L bytes from one sample */

#define ODR_208HZ            (0x5u << 4)
#define FS_XL_2G             (0x0u << 2)
#define FS_G_250DPS          (0x0u << 1)

#define ACCEL_G_PER_LSB      0.000061f  /* ±2 g    */
#define GYRO_DPS_PER_LSB     0.00875f   /* ±250 dps */
#define TEMP_LSB_PER_C       256.0f

#define I2C_TIMEOUT          25u

static bool s_ready = false;

static bool reg_write(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c_imu, ISM_ADDR_HAL, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, I2C_TIMEOUT) == HAL_OK;
}

static bool reg_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c_imu, ISM_ADDR_HAL, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, I2C_TIMEOUT) == HAL_OK;
}

bool ism330Begin(void)
{
    s_ready = false;
    if (hi2c_imu.State == HAL_I2C_STATE_RESET) return false;   /* bus not up */

    static const uint8_t cand[2] = { 0x6Bu, 0x6Au };
    bool found = false;
    for (int i = 0; i < 2 && !found; i++) {
        uint8_t who = 0;
        s_addr7 = cand[i];
        found = reg_read(REG_WHO_AM_I, &who, 1) && who == WHO_AM_I_VAL;
    }
    if (!found) { s_addr7 = 0x6Bu; return false; }

    /* Soft reset, then wait for SW_RESET to self-clear (typ. ~50 us). */
    if (!reg_write(REG_CTRL3_C, CTRL3_SW_RESET)) return false;
    for (int i = 0; i < 10; i++) {
        HAL_Delay(1);
        uint8_t c = CTRL3_SW_RESET;
        if (reg_read(REG_CTRL3_C, &c, 1) && !(c & CTRL3_SW_RESET)) break;
    }

    if (!reg_write(REG_CTRL3_C,  CTRL3_BDU | CTRL3_IF_INC)) return false;
    if (!reg_write(REG_CTRL1_XL, ODR_208HZ | FS_XL_2G))     return false;
    if (!reg_write(REG_CTRL2_G,  ODR_208HZ | FS_G_250DPS))  return false;

    HAL_Delay(20);                      /* let the first samples settle */
    s_ready = true;
    return true;
}

bool ism330IsReady(void) { return s_ready; }

uint8_t ism330Addr(void) { return s_ready ? s_addr7 : 0u; }

bool ism330IsPresent(void)
{
    return HAL_I2C_IsDeviceReady(&hi2c_imu, ISM_ADDR_HAL, 3, I2C_TIMEOUT) == HAL_OK;
}

bool ism330ReadRaw(int16_t *ax, int16_t *ay, int16_t *az,
                   int16_t *gx, int16_t *gy, int16_t *gz)
{
    if (!s_ready) return false;
    uint8_t b[12];
    if (!reg_read(REG_OUTX_L_G, b, sizeof b)) return false;
    *gx = (int16_t)((b[1]  << 8) | b[0]);
    *gy = (int16_t)((b[3]  << 8) | b[2]);
    *gz = (int16_t)((b[5]  << 8) | b[4]);
    *ax = (int16_t)((b[7]  << 8) | b[6]);
    *ay = (int16_t)((b[9]  << 8) | b[8]);
    *az = (int16_t)((b[11] << 8) | b[10]);
    return true;
}

bool ism330Read(ism330_sample_t *s)
{
    int16_t ax, ay, az, gx, gy, gz;
    if (!ism330ReadRaw(&ax, &ay, &az, &gx, &gy, &gz)) return false;
    s->ax_g   = ax * ACCEL_G_PER_LSB;
    s->ay_g   = ay * ACCEL_G_PER_LSB;
    s->az_g   = az * ACCEL_G_PER_LSB;
    s->gx_dps = gx * GYRO_DPS_PER_LSB;
    s->gy_dps = gy * GYRO_DPS_PER_LSB;
    s->gz_dps = gz * GYRO_DPS_PER_LSB;
    return true;
}

bool ism330ReadTempC(float *temp_c)
{
    if (!s_ready) return false;
    uint8_t b[2];
    if (!reg_read(REG_OUT_TEMP_L, b, sizeof b)) return false;
    int16_t raw = (int16_t)((b[1] << 8) | b[0]);
    *temp_c = 25.0f + raw / TEMP_LSB_PER_C;
    return true;
}
