/*
 * loadcell.c - automatic payload detection for the RotaTruck (spec FR5).
 *
 * Port of P102 Load Detection firmware v4.2 (Code_STM/P102_LoadDetect_Main/
 * src/main.c) into the RotaTruck firmware.  The signal chain, filters,
 * views, flash layout (v6), rest zero, payload status codes and the serial
 * command set are unchanged, so the host GUIs (host/stm_dashboard.py,
 * host/stm_dual_test.py) keep working.  Integration changes, marked [RT]:
 *
 *   - No own clock / LPUART / SysTick: the RotaTruck main.c runs the MCU at
 *     170 MHz and owns USART2 on the ST-LINK VCP.  delay_us() uses the DWT
 *     cycle counter with SystemCoreClock, so HX711 timing is unchanged.
 *   - No own IMU driver: pose uses the shared IMU layer (imu.c, ISM330DHCX
 *     at 0x6B) via imuGetLatest(), in the cart frame (board.h
 *     IMU_AXIS_SIGN_*).  The default pose references were re-measured in
 *     that frame on the truck (standing ~-43.6 deg, tipped back ~-1.2 deg).
 *   - Output: bench-rig text and the 10 Hz CSV only while the load-detect
 *     console is on (`LD`).  Outside it, only safety-relevant events
 *     (faults, IMU lost) are printed, so normal telemetry stays quiet (NFR5).
 *   - Power-up: no 6 s diagnostic; a short presence check + tare only if no
 *     zero is saved, and a one-line summary.
 *   - Results only (status, 0.9 x measured).  Which value the assist uses,
 *     and when, is decided in payload.c (latch on STANDING -> tipped back,
 *     manual buttons as the fallback).
 *
 * Bridges
 *   CH1  MIDDLE strip        POISSON_HALF  R1 crosswise gauge, R3 lengthwise gauge,
 *                                          R2 / R4 350 ohm completion pair
 *   CH2  LEFT + RIGHT strips POISSON_FULL  R1 L crosswise, R3 L lengthwise,
 *                                          R2 R lengthwise, R4 R crosswise
 *
 * Views ('x'), each with its OWN saved scale and offset:
 *     x 1   CH1 middle pair only           (2 gauges)   host CONFIG poisson_half
 *     x 2   CH2 left + right pairs only    (4 gauges)   host CONFIG poisson_full
 *     x 0   combined, CH1 + g x CH2        (6 gauges)   host CONFIG poisson_half
 *     x 3   dual test stream, both raw:  D,t_ms,raw1,raw2,ustr1,ustr2,ustr_comb,pose,tilt_deg
 *
 * CSV (views 0-2, console only):
 *   t_ms,raw,offset_raw,hx_mV,bridge_uV,microstrain,newtons,valid,status,assist_kg,assist_pct,pose,tilt_deg
 *
 * Console commands (inside `LD`, newline terminated) - see the bench-rig
 * RUN_GUIDE / LOAD_DETECTION.md:
 *   d / b / v    diagnostic both / banner + bridge report / live per-channel values
 *   x <0-3>      view                     t   temporary tare
 *   z [r1 r2]    save empty-plate zero    c <N>   one-point calibration
 *   s <cts/N> [off]  set scale + offset   u   clear all scales
 *   q [0|1|s|r|d]    IMU status / gating / capture standing, tipped-back / defaults
 *   g [m|s|w]    side weight              i <1|2>  flip a channel's sign
 *   n <s> f <s> k <ustr> m <0|1> h <ustr> a <kg> p [kg] r [R2] [R4]
 */

#include "loadcell.h"
#include "board.h"
#include "command.h"            /* [RT] uart_print */
#include "imu.h"                /* [RT] shared IMU layer */
#include "ism330.h"
#include "stm32g4xx_hal.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

/* ======================  EDIT THESE  ===================================== */
#define POISSON_RATIO     0.33f     /* aluminium 0.33, steel 0.29              */
#define GAUGE_FACTOR      2.0f      /* from the gauge packet                   */
#define R_GAUGE           350.0f
#define R_COMP_DEFAULT    350.0f    /* CH1 completion pair, rig standard       */
#define HX_AVDD           3.2f      /* HX711 VCC 3V3 -> about 3.2 V. 4.3 at 5 V */
#define RATED_KG_DEFAULT  150.0f
#define ASSIST_SF         0.9f      /* assist sized for 90 % of the load       */
#define STAND_DEG         5.0f      /* within this of the standing pose = standing */
#define TIP_DEG           15.0f     /* tilted back at least this = tipped back  */
#define STILL_DPS         3.0f      /* gyro below this ...                      */
#define STILL_G           0.05f     /* ... and |accel| within 1 g +/- this = still */
#define STILL_MS          1000U     /* still for this long before it counts     */
#define UNCAL_USTR_PER_KG 3.0f      /* zero band before calibration (test 15, middle strip) */
#define IMU_STALE_MS      100U      /* [RT] shared IMU sample older than this = missed */
#define BOOT_TARE_N       10        /* [RT] samples per bridge for the power-up tare  */
/* ========================================================================= */

#define HX_GAIN_PULSES    25        /* channel A, gain 128                     */
#define HX_RAIL_WARN      8300000L
#define HX_FS_MV          (0.5f * HX_AVDD / 128.0f * 1000.0f)
#define HX_COUNTS_PER_MV  (8388608.0f / HX_FS_MV)
#define SAMPLE_PERIOD_S   0.10f     /* HX711 at 10 SPS                         */
#define NO_DATA_MS        500U
#define FROZEN_N          20
#define MEDIAN_N          5
#define AVG_N             8
#define SH_MAX            16

/* ------------------------------------------------------------------ status */
#define ST_OK          LC_ST_OK
#define ST_SETTLING    LC_ST_SETTLING
#define ST_NO_CAL      LC_ST_NO_CAL
#define ST_RAILED      LC_ST_RAILED
#define ST_FROZEN      LC_ST_FROZEN
#define ST_NO_DATA     LC_ST_NO_DATA
#define ST_IMPLAUSIBLE LC_ST_IMPLAUSIBLE
#define ST_OVERLOAD    LC_ST_OVERLOAD
#define ST_HELD        LC_ST_HELD
#define ST_IMU_FAULT   LC_ST_IMU_FAULT
static const char *STATUS_NAME[] = { "OK", "SETTLING", "NO_CAL", "RAILED", "FROZEN",
                                     "NO_DATA", "IMPLAUSIBLE", "OVERLOAD", "HELD", "IMU_FAULT" };
static const char *STATUS_SHORT[] = { "OK", "SETL", "NOCAL", "RAIL", "FROZ",
                                      "NODAT", "IMPL", "OVER", "HELD", "IMU!" };
/* [RT] LP_ prefix: the RotaTruck pose machine already owns POSE_STANDING. */
enum { LP_UNKNOWN = 0, LP_STANDING, LP_TILTING, LP_TIPPED_STILL, LP_TIPPED_MOVING };
static const char *POSE_NAME[] = { "UNKNOWN", "STANDING", "TILTING", "TIPPED BACK (still)",
                                   "TIPPED BACK (moving)" };
static const char *VIEW_NAME[] = { "COMBINED (CH1 + g x CH2, 6 gauges)",
                                   "CH1 middle (2 gauges)",
                                   "CH2 left + right (4 gauges)",
                                   "dual test stream" };

/* ------------------------------------------------------------------ globals */
typedef struct {
  const char   *name;
  const char   *cfg;
  GPIO_TypeDef *dtPort;  uint16_t dtPin;
  GPIO_TypeDef *skPort;  uint16_t skPin;
  float   factor, cpu;            /* bridge factor, counts per microstrain     */
  int32_t sign;
  int32_t raw, med, tare;
  int32_t medBuf[MEDIAN_N]; uint8_t medIdx, medFull;
  int32_t avgBuf[AVG_N];    uint8_t avgIdx, avgN;
  int32_t lastRaw;          uint8_t sameCnt;
  uint32_t lastMs;          uint8_t seen, fresh;
} chan_t;

static chan_t ch[2] = {
  { .name = "CH1 middle",     .cfg = "POISSON_HALF",
    .dtPort = LC_CH1_DT_PORT, .dtPin = LC_CH1_DT_PIN, .skPort = LC_CH1_SCK_PORT, .skPin = LC_CH1_SCK_PIN },
  { .name = "CH2 left+right", .cfg = "POISSON_FULL",
    .dtPort = LC_CH2_DT_PORT, .dtPin = LC_CH2_DT_PIN, .skPort = LC_CH2_SCK_PORT, .skPin = LC_CH2_SCK_PIN },
};

/* settings kept in the last flash page (size must be a multiple of 8) */
#define STORE_MAGIC   0x50313032UL   /* "P102" */
#define STORE_VERSION 6UL            /* v4.2: offsets and IMU poses (v5 is migrated) */
typedef struct {
  uint32_t magic, version;
  float    compTop, compBot;
  float    cpn[3];        uint32_t calValid[3];     /* scale per view 0 / 1 / 2 */
  int32_t  zero1, zero2;  uint32_t zeroValid;       /* bit 0 CH1, bit 1 CH2     */
  float    ratedKg, sideWeight;
  int32_t  sign1, sign2;
  uint32_t view;
  /* ---- v6 additions, everything above is the v5 layout ---- */
  float    coff[3];                                  /* fitted offset per view, counts */
  float    imuStand[3], imuRest[3];                  /* gravity direction in each pose */
  uint32_t imuRef;                                   /* bit 0 stand, bit 1 rest captured */
  uint32_t poseMode;                                 /* 1 = IMU pose gating on   */
  uint32_t pad[1];
} store_t;
_Static_assert(sizeof(store_t) % 8 == 0, "store_t must be a whole number of flash double words");
_Static_assert(offsetof(store_t, coff) == 72, "v5 prefix must stay 72 bytes for the migration");
#define STORE_V5_PREFIX offsetof(store_t, coff)
static store_t cfg __attribute__((aligned(8)));

/* filter and display */
static float    FILTER_TAU_S = 2.0f, SNAP_USTRAIN = 1.0f, HOLD_BAND_USTRAIN = 0.5f;
static uint8_t  FILTER_MODE  = 1;                /* 1 stable-hold, 0 smooth     */
static const float FAST_TAU_S = 0.15f, FAST_HOLD_S = 1.5f, STABLE_WINDOW_S = 0.5f;
static const uint8_t RELEASE_N = 3;
static float    AZT_BAND_KG = 3.0f;
static const float AZT_TIME_S = 3.0f, AZT_DEADZONE_KG = 0.05f, AZT_WARN_KG = 15.0f;

static float    emaVal; static uint8_t emaInit; static uint32_t fastUntilMs, snapCount;
static float    shBuf[SH_MAX]; static uint8_t shIdx, shCount, shRelease, shHolding;
static float    shHeld;
static uint32_t aztSinceMs, aztCount; static uint8_t aztWarned;
static float    newtonsPerMicrostrain = 0.05f;   /* placeholder until calibrated */
static float    predictedOffsetMv;
static uint8_t  dualStream;

static uint8_t  payloadStatus = ST_NO_CAL, lastPrintedStatus = 255;
static float    assistKg, measuredKg;
static uint32_t lastNoDataRowMs, ledMs;

/* noise report: 0 CH1, 1 CH2, 2 combined */
static uint8_t  repActive; static uint32_t repStartMs, repEndMs, repN;
static float    repFirst[3], repLast[3], repMin[3], repMax[3], repMean[3], repM2[3];

/* g capture */
static float    gMidU1, gMidU2; static uint8_t gMidValid;

/* IMU pose */
#define IMU_PERIOD_MS 20U
static uint8_t  imuOk, imuFound; static uint32_t imuLastMs; static uint8_t imuFailN;
static float    aLP[3], gBias[3]; static uint8_t aInit;
static float    refS[3], refRp[3];               /* standing gravity, tip-back direction */
static float    tiltDeg, devDeg, tipLimDeg = TIP_DEG; static uint8_t pose = LP_UNKNOWN;
static uint32_t lastMoveMs; static uint8_t movedSinceRow = 1;

/* rest zero (zero tracking while tipped back and still) */
#define ZT_HIST 32                               /* rows of history (3.2 s)                 */
#define ZT_RUN  25                               /* still rows before tracking (2.5 s)      */
#define ZT_LAG  11                               /* newest sample used is 1.1 s old         */
#define ZT_N    8                                /* samples averaged into the zero          */
static int32_t  ztHist[2][ZT_HIST]; static uint8_t ztIdx, ztCount;
static uint16_t ztRun; static uint8_t ztTracking, ztBooted, ztWarned;
static uint32_t ztRejectSinceMs; static float ztEpisodeKg;
static float    heldKg;                          /* last valid standing load              */

/* [RT] load-detect console */
static bool     s_console   = false;

/* ------------------------------------------------------------------ low level */
static uint32_t millis(void) { return HAL_GetTick(); }

static void delay_us(uint32_t us) {
  uint32_t start = DWT->CYCCNT, n = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < n) { }
}

/* [RT] bench-rig text: console only */
static void out(const char *fmt, ...) {
  if (!s_console) return;
  char buf[224];
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  uart_print(buf);
}

/* [RT] safety-relevant events: always printed (NFR2: transitions logged) */
static void out_evt(const char *fmt, ...) {
  char buf[224];
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  uart_print(buf);
}

static void pins_init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  /* HX711 DT inputs, pulled up so a loose wire reads "not ready" -> NO_DATA */
  g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
  g.Pin = LC_CH1_DT_PIN; HAL_GPIO_Init(LC_CH1_DT_PORT, &g);
  g.Pin = LC_CH2_DT_PIN; HAL_GPIO_Init(LC_CH2_DT_PORT, &g);

  /* HX711 SCK outputs (PB3 / PB4 leave JTAG; SWD on PA13 / PA14 still works) */
  memset(&g, 0, sizeof(g));
  g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin = LC_CH1_SCK_PIN; HAL_GPIO_Init(LC_CH1_SCK_PORT, &g);
  g.Pin = LC_CH2_SCK_PIN; HAL_GPIO_Init(LC_CH2_SCK_PORT, &g);
  HAL_GPIO_WritePin(LC_CH1_SCK_PORT, LC_CH1_SCK_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LC_CH2_SCK_PORT, LC_CH2_SCK_PIN, GPIO_PIN_RESET);

  /* LD2 status LED */
  g.Pin = LC_LED_PIN; HAL_GPIO_Init(LC_LED_PORT, &g);

  /* cycle counter for microsecond delays */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ------------------------------------------------------------------ HX711 */
static int hx_ready(const chan_t *c) {
  return HAL_GPIO_ReadPin(c->dtPort, c->dtPin) == GPIO_PIN_RESET;
}

/* 24 data pulses + 1 for channel A gain 128. Interrupts off so SCK never
 * stays high long enough (60 us) to power the HX711 down. */
static int32_t hx_read(const chan_t *c) {
  uint32_t v = 0;
  __disable_irq();
  for (int i = 0; i < HX_GAIN_PULSES; i++) {
    c->skPort->BSRR = c->skPin;
    delay_us(1);
    if (i < 24) {
      v <<= 1;
      if (c->dtPort->IDR & c->dtPin) v |= 1U;
    }
    c->skPort->BRR = c->skPin;
    delay_us(1);
  }
  __enable_irq();
  if (v & 0x800000U) v |= 0xFF000000U;
  return (int32_t)v;
}

static int hx_wait(const chan_t *c, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (!hx_ready(c)) if (millis() - t0 > timeoutMs) return 0;
  return 1;
}

/* averaged raw reading of one channel, 0 on timeout */
static int hx_average(chan_t *c, int n, int32_t *result) {
  int64_t sum = 0;
  for (int i = 0; i < n; i++) {
    if (!hx_wait(c, 1000)) return 0;
    sum += hx_read(c);
  }
  *result = (int32_t)(sum / n);
  return 1;
}

/* ------------------------------------------------------------------ flash store */
static uint32_t store_addr(void) { return FLASH_BASE + FLASH_SIZE - FLASH_PAGE_SIZE; }

/* [RT] default poses in the RotaTruck cart frame (imu.c, board.h
 * IMU_AXIS_SIGN_*), measured on the truck 2026-11-10 with the ISM330DHCX:
 *   standing on the toe  : pitch atan2(ay,az) ~ -43.6 deg -> (0, -0.690, 0.724)
 *   tipped back (drive)  : pitch ~ -1.2 deg               -> (0, -0.021, 1.000)
 * Capture the real ones with `q s` / `q r` once the plate is fitted. */
static void imu_ref_defaults(void) {
  cfg.imuStand[0] = 0.0f;  cfg.imuStand[1] = -0.690f; cfg.imuStand[2] = 0.724f;
  cfg.imuRest[0]  = 0.0f;  cfg.imuRest[1]  = -0.021f; cfg.imuRest[2]  = 1.000f;
  cfg.imuRef = 0;
}

static void v6_defaults(void) {
  for (int v = 0; v < 3; v++) cfg.coff[v] = 0.0f;
  imu_ref_defaults();
  cfg.poseMode = 1;
  cfg.pad[0] = 0;
}

static void store_defaults(void) {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = STORE_MAGIC; cfg.version = STORE_VERSION;
  cfg.compTop = cfg.compBot = R_COMP_DEFAULT;
  cfg.ratedKg = RATED_KG_DEFAULT; cfg.sideWeight = 1.0f;
  cfg.sign1 = -1; cfg.sign2 = -1;            /* compression positive, as on the Uno */
  cfg.view = 0;
  v6_defaults();
}

static int finite3(const float *v) {
  return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]) &&
         (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) > 0.25f;
}

/* returns 0 = defaults, 1 = loaded, 2 = migrated from v4.1 (v5) */
static int store_load(void) {
  store_t s;
  int res = 1;
  memcpy(&s, (const void *)store_addr(), sizeof(s));
  if (s.magic != STORE_MAGIC || (s.version != STORE_VERSION && s.version != 5UL)) {
    store_defaults(); return 0;
  }
  if (s.version == 5UL) {                    /* same fields up to 'view', new ones after */
    store_defaults();
    memcpy(&cfg, &s, STORE_V5_PREFIX);
    cfg.version = STORE_VERSION;
    res = 2;
  } else {
    cfg = s;
  }
  if (!(cfg.ratedKg >= 1.0f && cfg.ratedKg <= 1000.0f)) cfg.ratedKg = RATED_KG_DEFAULT;
  if (!(cfg.sideWeight > 0.05f && cfg.sideWeight < 20.0f)) cfg.sideWeight = 1.0f;
  if (cfg.sign1 != 1) cfg.sign1 = -1;
  if (cfg.sign2 != 1) cfg.sign2 = -1;
  if (cfg.view > 2) cfg.view = 0;
  for (int v = 0; v < 3; v++) if (!isfinite(cfg.coff[v])) cfg.coff[v] = 0.0f;
  if (!finite3(cfg.imuStand) || !finite3(cfg.imuRest)) imu_ref_defaults();
  if (cfg.poseMode > 1) cfg.poseMode = 1;
  return res;
}

/* Page erase stalls the CPU ~20 ms; only reached from console commands,
 * where the assist is forced off. */
static int store_save(void) {
  FLASH_EraseInitTypeDef e = {0};
  uint32_t err = 0, addr = store_addr();
  HAL_StatusTypeDef st;
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  e.TypeErase = FLASH_TYPEERASE_PAGES;
  e.Banks = FLASH_BANK_1;
  e.Page = (FLASH_SIZE / FLASH_PAGE_SIZE) - 1U;
  e.NbPages = 1;
  st = HAL_FLASHEx_Erase(&e, &err);
  for (uint32_t i = 0; st == HAL_OK && i < sizeof(cfg) / 8U; i++) {
    uint64_t dw;
    memcpy(&dw, (const uint8_t *)&cfg + 8U * i, 8);
    st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + 8U * i, dw);
  }
  HAL_FLASH_Lock();
  if (st != HAL_OK) out_evt("# ERROR: load-detect flash save failed\r\n");
  return st == HAL_OK;
}

/* ------------------------------------------------------------------ view helpers */
static int   in_view(int i)   { return dualStream || cfg.view == 0 || (int)cfg.view == i + 1; }
static int   primary_ch(void) { return (!dualStream && cfg.view == 2) ? 1 : 0; }
static float ref_cpu(void)    { return ch[primary_ch()].cpu; }
static const char *view_cfg(void) { return primary_ch() == 1 ? "POISSON_FULL" : "POISSON_HALF"; }
static int   zero_ok(int i)   { return (cfg.zeroValid >> i) & 1U; }

/* ------------------------------------------------------------------ constants */
static int valid_r(float r) { return r == r && r >= 50.0f && r <= 10000.0f; }

static void compute_constants(void) {
  ch[0].factor = 1.0f + POISSON_RATIO;
  ch[1].factor = 2.0f * (1.0f + POISSON_RATIO);
  for (int i = 0; i < 2; i++)
    ch[i].cpu = ch[i].factor * (GAUGE_FACTOR / 4.0f) * 256.0f * 8.388608f;
  ch[0].sign = cfg.sign1; ch[1].sign = cfg.sign2;
  /* CH1: R1 / R3 gauges, R2 / R4 completion */
  float vA = R_GAUGE / (R_GAUGE + R_GAUGE);
  float vB = cfg.compBot / (cfg.compTop + cfg.compBot);
  predictedOffsetMv = HX_AVDD * (vA - vB) * 1000.0f;
  newtonsPerMicrostrain = cfg.calValid[cfg.view] ? ref_cpu() / cfg.cpn[cfg.view] : 0.05f;
}

/* tared counts of one channel, and the view value in the view's own counts */
static float tared(const chan_t *c, int32_t v) { return (float)(c->sign * (v - c->tare)); }
static float combine(float x1, float x2) {
  if (cfg.view == 1) return x1;
  if (cfg.view == 2) return x2;
  return x1 + cfg.sideWeight * x2 * (ch[0].cpu / ch[1].cpu);
}
static float counts_to_kg(float v) {
  return (v / ref_cpu()) * newtonsPerMicrostrain / 9.80665f;
}
/* load in newtons for a view value: (value - offset) / scale once calibrated */
static float view_newtons(float v) {
  uint32_t w = cfg.view;
  if (cfg.calValid[w]) return (v - cfg.coff[w]) / cfg.cpn[w];
  return (v / ref_cpu()) * newtonsPerMicrostrain;
}
/* size of a zero step in kg: slope only, nominal sensitivity before calibration */
static float step_kg(float dv) {
  if (cfg.calValid[cfg.view]) return counts_to_kg(dv);
  return dv / ref_cpu() / UNCAL_USTR_PER_KG;
}
static int gating(void) { return cfg.poseMode != 0; }

/* ------------------------------------------------------------------ filters */
static void filter_reset(void) {
  fastUntilMs = 0; emaInit = 0; emaVal = 0;
  shIdx = shCount = shRelease = shHolding = 0; shHeld = 0;
  for (int i = 0; i < 2; i++) { ch[i].medIdx = 0; ch[i].medFull = 0; ch[i].avgN = 0; ch[i].avgIdx = 0; }
}

static int32_t median_stage(chan_t *c, int32_t x) {
  c->medBuf[c->medIdx] = x;
  c->medIdx = (uint8_t)((c->medIdx + 1) % MEDIAN_N);
  if (c->medIdx == 0) c->medFull = 1;
  int n = c->medFull ? MEDIAN_N : c->medIdx;
  if (n < 3) return x;
  int32_t t[MEDIAN_N];
  for (int i = 0; i < n; i++) t[i] = c->medBuf[i];
  for (int i = 1; i < n; i++) {
    int32_t k = t[i]; int j = i - 1;
    while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; }
    t[j + 1] = k;
  }
  return t[n / 2];
}

static float low_pass_stage(float x) {
  if (FILTER_TAU_S <= 0 || !emaInit) { emaVal = x; emaInit = 1; return x; }
  if (SNAP_USTRAIN > 0 && fabsf(x - emaVal) > SNAP_USTRAIN * ref_cpu()) {
    emaVal = x; snapCount++;
    fastUntilMs = millis() + (uint32_t)(FAST_HOLD_S * 1000.0f);
    return x;
  }
  float tau = ((int32_t)(millis() - fastUntilMs) < 0) ? FAST_TAU_S : FILTER_TAU_S;
  emaVal += (SAMPLE_PERIOD_S / (tau + SAMPLE_PERIOD_S)) * (x - emaVal);
  return emaVal;
}

static uint8_t stable_n(void) {
  int n = (int)(STABLE_WINDOW_S / SAMPLE_PERIOD_S + 0.5f);
  if (n < 2) n = 2;
  if (n > SH_MAX) n = SH_MAX;
  return (uint8_t)n;
}

static float hold_stage(float x) {
  uint8_t n = stable_n();
  shBuf[shIdx] = x; shIdx = (uint8_t)((shIdx + 1) % n);
  if (shCount < n) shCount++;
  float band = HOLD_BAND_USTRAIN * ref_cpu();
  if (shHolding) {
    float dev = fabsf(x - shHeld);
    if (SNAP_USTRAIN > 0 && dev > SNAP_USTRAIN * ref_cpu()) { shHolding = 0; shRelease = 0; snapCount++; }
    else if (dev > band) { if (++shRelease >= RELEASE_N) { shHolding = 0; shRelease = 0; } }
    else shRelease = 0;
    if (shHolding) return shHeld;
  }
  if (shCount >= n) {
    float mn = shBuf[0], mx = shBuf[0], sum = 0;
    for (int i = 0; i < n; i++) {
      if (shBuf[i] < mn) mn = shBuf[i];
      if (shBuf[i] > mx) mx = shBuf[i];
      sum += shBuf[i];
    }
    if (mx - mn <= band) { shHeld = sum / n; shHolding = 1; return shHeld; }
  }
  return x;
}

/* ------------------------------------------------------------------ banner */
static void print_comp(void) {
  out("# comp R2 / R4        = %.0f / %.0f ohm  (CH1, saved in flash)\r\n", (double)cfg.compTop, (double)cfg.compBot);
}

static void print_view(void) {
  int v = dualStream ? 3 : (int)cfg.view;
  out("# view                 = x %d  %s\r\n", v, VIEW_NAME[v]);
  if (!dualStream)
    out("# host CONFIG          = %s\r\n", cfg.view == 2 ? "poisson_full" : "poisson_half");
}

static void print_cal(void) {
  uint32_t v = cfg.view;
  if (cfg.calValid[v]) {
    out("# saved scale (view %lu) = %.4f counts/N (flash)\r\n", (unsigned long)v, (double)cfg.cpn[v]);
    out("# saved offset         = %.0f counts = %.2f kg at the zero\r\n", (double)cfg.coff[v],
        (double)(-cfg.coff[v] / cfg.cpn[v] / 9.80665f));
  } else {
    out("# saved scale (view %lu) = none - calibrate this view, or 's <counts/N> [offset]'\r\n", (unsigned long)v);
  }
  if (zero_ok(0) || zero_ok(1)) {
    char a[16], b[16];
    if (zero_ok(0)) snprintf(a, sizeof(a), "%ld", (long)cfg.zero1); else strcpy(a, "none");
    if (zero_ok(1)) snprintf(b, sizeof(b), "%ld", (long)cfg.zero2); else strcpy(b, "none");
    out("# saved zero           = CH1 %s / CH2 %s raw (flash, used at power-up)\r\n", a, b);
  } else {
    out("# saved zero           = none - tares at power-up. Send 'z' tipped back, plate empty.\r\n");
  }
  if (AZT_BAND_KG <= 0) {
    out("# zero tracking        = off ('a <kg>' to turn on)\r\n");
  } else if (gating()) {
    out("# zero tracking        = REST ZERO: tipped back and still, steps up to +/-%.2f kg%s\r\n",
        (double)AZT_BAND_KG, cfg.calValid[v] ? "" : " (nominal before calibration)");
  } else if (cfg.calValid[v] && cfg.coff[v] != 0.0f) {
    out("# zero tracking        = off: this view has an offset. Zero by hand ('t' tipped back, empty)\r\n");
  } else if (cfg.calValid[v]) {
    out("# zero tracking        = auto-zero +/-%.2f kg after %.1f s stable\r\n", (double)AZT_BAND_KG, (double)AZT_TIME_S);
  } else {
    out("# zero tracking        = auto-zero +/-%.2f kg, starts once this view is calibrated\r\n", (double)AZT_BAND_KG);
  }
}

static void print_imu(void) {
  /* [RT] the host regex expects "ISM330DHCX" or "NOT FOUND" */
  if (imuOk) out("# IMU                  = ISM330DHCX at 0x%02X (shared IMU layer)\r\n", ism330Addr());
  else       out("# IMU                  = NOT FOUND (I2C1: SDA PB9 / D14, SCL PB8 / D15)\r\n");
  if (gating())
    out("# pose gating          = ON (q 1): zero when tipped back + still, read when standing\r\n");
  else
    out("# pose gating          = OFF (q 0): no IMU, treated as STANDING - readings always live, zero by hand\r\n");
  if (imuOk)
    out("# pose now             = %s, tilt %.1f deg back, %.1f deg from standing\r\n",
        POSE_NAME[pose], (double)tiltDeg, (double)devDeg);
  out("# pose references      = standing %s, tipped back %s; standing within %.0f deg, tipped back past %.0f deg\r\n",
      (cfg.imuRef & 1U) ? "captured" : "default (truck)",
      (cfg.imuRef & 2U) ? "captured" : "default (truck)", (double)STAND_DEG, (double)tipLimDeg);
  if (gating() && !imuOk)
    out("# [WARN] gating is on but no IMU: assist stays 0 (IMU_FAULT). Wire it, or send 'q 0'.\r\n");
}

static void print_assist(void) {
  out("# rated payload        = %.1f kg\r\n", (double)cfg.ratedKg);
  out("# assist safety factor = %.2f  (assist sized for this fraction of the measured load)\r\n", (double)ASSIST_SF);
  out("# payload status       = %s\r\n", STATUS_NAME[payloadStatus]);
  out("# latch to the assist  = on STANDING -> tipped back, if the assist was switched on at STANDING (payload.c)\r\n");
}

static void print_mode(void) {
  if (FILTER_MODE) out("# filter mode          = STABLE-HOLD (m 1): band +/-%.2f ustrain, window %.1f s\r\n",
                       (double)HOLD_BAND_USTRAIN, (double)STABLE_WINDOW_S);
  else             out("# filter mode          = SMOOTH (m 0): low-pass tau %.2f s with step snap\r\n", (double)FILTER_TAU_S);
}

static void print_side(void) {
  out("# side weight g        = %.4f  (combined = CH1 + g x CH2, in ustrain)\r\n", (double)cfg.sideWeight);
}

static void print_commands(void) {
  out("# commands: d / b / v / x <0-3> / t / z [r1 r2] / c <N> / s <cts/N> [off] / u / q [0|1|s|r|d] / g [m|s|w] / i <1|2> / n <s> / f <s> / k <ustr> / m <0|1> / h <ustr> / a <kg> / p [kg] / r [ohm] [ohm]\r\n");
  out("# LD = leave the load-detect console (assist can be armed again)\r\n");
}

static void print_banner(void) {
  out("# Strain gauge rig v4.2-rt, NUCLEO-G491RE - %s, HX711 x2 (RotaTruck integrated)\r\n", view_cfg());
  out("#   CH1 middle       %s  x%.3f  %.1f cts/ustr  DT PA10 (D2)  SCK PB3 (D3)  sign %+ld\r\n",
      ch[0].cfg, (double)ch[0].factor, (double)ch[0].cpu, (long)ch[0].sign);
  out("#   CH2 left+right   %s  x%.3f  %.1f cts/ustr  DT PB5 (D4)   SCK PB4 (D5)  sign %+ld\r\n",
      ch[1].cfg, (double)ch[1].factor, (double)ch[1].cpu, (long)ch[1].sign);
  out("# gauge / comp        = %.0f / %.0f ohm\r\n", (double)R_GAUGE, (double)cfg.compTop);
  print_comp();
  out("# bridge factor        = %.3f\r\n", (double)ch[primary_ch()].factor);
  out("# countsPerMicrostrain = %.1f\r\n", (double)ref_cpu());
  out("# HX711 AVDD           = %.2f V, full scale +/-%.2f mV\r\n", (double)HX_AVDD, (double)HX_FS_MV);
  print_view();
  print_side();
  print_mode();
  if (SNAP_USTRAIN > 0) out("# step snap            = %.2f ustrain, then tau %.2f s for %.1f s\r\n",
                            (double)SNAP_USTRAIN, (double)FAST_TAU_S, (double)FAST_HOLD_S);
  else                  out("# step snap            = off\r\n");
  print_cal();
  print_imu();
  print_assist();
  out("# WARM-UP: leave powered 20-40 min before saving a zero.\r\n");
  print_commands();
}

static void bridge_report(void) {
  float iCh1 = HX_AVDD / (2.0f * R_GAUGE) * 1000.0f + HX_AVDD / (cfg.compTop + cfg.compBot) * 1000.0f;
  float iCh2 = 2.0f * HX_AVDD / (2.0f * R_GAUGE) * 1000.0f;
  float a = R_GAUGE, arm = (a * 3.0f * a) / (4.0f * a);
  out("#\r\n# ===== BRIDGE REPORT =====\r\n");
  out("# predicted off : %.2f mV   (HX711 ceiling +/-%.1f mV)\r\n", (double)predictedOffsetMv, (double)HX_FS_MV);
  if (fabsf(cfg.compTop - cfg.compBot) > 0.5f)
    out("# [FAIL] R2 and R4 differ. Fit a matched pair.\r\n");
  else if (fabsf(predictedOffsetMv) > HX_FS_MV)
    out("# [FAIL] CH1 will rail. Completion pair must match the gauges.\r\n");
  else
    out("# [ OK ] CH1 offset fits. CH2 is a full bridge (no completion resistors).\r\n");
  out("# CH2 unpowered check (HX711 #2 unplugged): each arm %.0f ohm, diagonals %.0f ohm\r\n", (double)arm, (double)a);
  out("# bridge current: CH1 %.1f mA, CH2 %.1f mA (from 3V3 via HX711)\r\n", (double)iCh1, (double)iCh2);
  out("# =========================\r\n#\r\n");
}

/* ------------------------------------------------------------------ diagnostic */
static void diagnose_channel(chan_t *c) {
  out("# --- %s (%s) ---\r\n", c->name, c->cfg);
  if (!hx_wait(c, 1000)) {
    out("# [FAIL] HX711 not responding within 1 s. Check VCC 3V3, GND, DT and SCK.\r\n");
    return;
  }
  const int N = 30; int32_t s[30]; int64_t sum = 0;
  int32_t mn = 0x7FFFFFFF, mx = -0x7FFFFFFF;
  for (int i = 0; i < N; i++) {
    if (!hx_wait(c, 1000)) { out("# [FAIL] stopped responding\r\n"); return; }
    s[i] = hx_read(c); sum += s[i];
    if (s[i] < mn) mn = s[i];
    if (s[i] > mx) mx = s[i];
  }
  int32_t mean = (int32_t)(sum / N); double var = 0;
  for (int i = 0; i < N; i++) { double d = (double)(s[i] - mean); var += d * d; }
  float sd = (float)sqrt(var / N), mv = (float)mean / HX_COUNTS_PER_MV;
  out("# mean raw %ld   spread %ld   noise sd %.0f counts = %.3f ustrain\r\n",
      (long)mean, (long)(mx - mn), (double)sd, (double)(sd / c->cpu));
  out("# HX711 in %.5f mV = %.1f %% of full scale\r\n", (double)mv, (double)(fabsf(mv) / HX_FS_MV * 100.0f));
  int bad = 0;
  if (mean == -1 || mean == 0) { out("# [FAIL] NO DATA: raw is all ones or zeros. DT/SCK swapped or loose.\r\n"); bad = 1; }
  if (labs(mean) >= HX_RAIL_WARN) { out("# [FAIL] RAILED: an arm is open, or the bridge is unbalanced.\r\n"); bad = 1; }
  if (mx == mn && mean != 0 && mean != -1) { out("# [FAIL] FROZEN: every sample identical.\r\n"); bad = 1; }
  if (sd > c->cpu * 5.0f) { out("# [WARN] noisy: sd > 5 ustrain. Solder joints, twisted leads.\r\n"); bad = 1; }
  if (c == &ch[1] && !bad && fabsf(mv) > HX_FS_MV * 0.3f)
    out("# [WARN] full bridge offset over 30 %% of range: one arm differs. Check the four arm resistances.\r\n");
  if (!bad) out("# [ OK ] channel behaving normally\r\n");
  out("# suggested snap ~%.2f ustrain, hold band ~%.2f ustrain\r\n", (double)(10.0f * sd / c->cpu), (double)(2.0f * sd / c->cpu));
}

static void run_diagnostic(void) {
  out("#\r\n# ===== DIAGNOSTIC =====\r\n");
  diagnose_channel(&ch[0]);
  diagnose_channel(&ch[1]);
  out("# ======================\r\n#\r\n");
}

/* ------------------------------------------------------------------ actions */
/* Zero / tare EACH bridge on its own, so a fault on one never blocks the other.
 * [RT] n = samples per bridge; a bridge that is not converting at all is
 * skipped after 250 ms instead of 1 s per sample. */
static void do_tare_n(int save, int n) {
  int done = 0;
  if (gating() && imuOk && pose != LP_UNKNOWN && pose != LP_TIPPED_STILL)
    out("# [WARN] zeroing while %s. The zero should be taken tipped back, empty, hands off.\r\n",
        POSE_NAME[pose]);
  for (int i = 0; i < 2; i++) {
    int32_t r;
    if (!hx_wait(&ch[i], 250) || !hx_average(&ch[i], n, &r)) {
      out("# %s: SKIPPED, HX711 #%d not responding\r\n", ch[i].name, i + 1); continue;
    }
    if (labs(r) > HX_RAIL_WARN) {
      out("# %s: REFUSED, input is railed. Send 'd'.\r\n", ch[i].name); continue;
    }
    ch[i].tare = r; done++;
    if (save) {
      if (i == 0) cfg.zero1 = r; else cfg.zero2 = r;
      cfg.zeroValid |= (1U << i);
    }
    out("# %s: %s = %ld\r\n", ch[i].name, save ? "zero saved raw" : "tare (temporary)", (long)r);
  }
  if (!done) { out("# NOTHING ZEROED: neither HX711 gave a usable reading. Send 'd'.\r\n"); return; }
  filter_reset();
  ztBooted = 1; ztRejectSinceMs = 0; ztWarned = 0; heldKg = 0;   /* operator says: empty */
  if (save) { aztWarned = 0; store_save(); }
}
static void do_tare(int save) { do_tare_n(save, 20); }

static void save_zero(char *args) {
  char *e1, *e2;
  long a = strtol(args, &e1, 10);
  if (e1 == args) { out("# saving zero: plate EMPTY, trolley tipped back, hands off...\r\n"); do_tare(1); return; }
  long b = strtol(e1, &e2, 10);
  if (e2 == e1) { out("# ERROR: give both channels, e.g. 'z 123456 -65432'\r\n"); return; }
  cfg.zero1 = a; cfg.zero2 = b; cfg.zeroValid = 3;
  ch[0].tare = a; ch[1].tare = b; filter_reset(); store_save();
  out("# zero saved raw = %ld / %ld\r\n", a, b);
}

/* scale in counts per newton and offset in counts: N = (value - off) / cpn */
static void apply_scale(float cpn, float off) {
  if (cpn == 0 || !isfinite(cpn)) { out("# ERROR: scale of zero rejected\r\n"); return; }
  if (!isfinite(off)) { out("# ERROR: offset is not a number\r\n"); return; }
  uint32_t v = cfg.view;
  cfg.cpn[v] = cpn; cfg.coff[v] = off; cfg.calValid[v] = 1; store_save(); compute_constants();
  aztSinceMs = 0;
  out("# countsPerNewton (view %lu) = %.4f  ->  newtonsPerMicrostrain = %.6f\r\n",
      (unsigned long)v, (double)cpn, (double)newtonsPerMicrostrain);
  out("# offset (view %lu)       = %.0f counts = %.2f kg at the zero\r\n",
      (unsigned long)v, (double)off, (double)(-off / cpn / 9.80665f));
  if (off != 0.0f && !gating())
    out("# no-IMU mode: zero with 't' tipped back and empty; read standing. Tipped back it reads %.1f kg.\r\n",
        (double)(-off / cpn / 9.80665f));
  out("# scale and offset saved to flash (survives power-off)\r\n");
}

static void set_scale(char *args) {
  char *e1, *e2;
  float cpn = strtof(args, &e1);
  if (e1 == args) { print_cal(); return; }
  float off = strtof(e1, &e2);
  if (e2 == e1) off = 0.0f;
  apply_scale(cpn, off);
}

/* 'u': forget the saved scale and offset of every view, back to the placeholder.
 * The GUI sends this on launch so each session starts uncalibrated. */
static void clear_scales(void) {
  int had = 0;
  for (int v = 0; v < 3; v++) {
    if (cfg.calValid[v] || cfg.coff[v] != 0.0f) had = 1;
    cfg.calValid[v] = 0; cfg.cpn[v] = 0; cfg.coff[v] = 0;
  }
  if (had) store_save();                 /* skip the flash write if nothing changed */
  compute_constants(); aztSinceMs = 0;
  out("# scales cleared: all views uncalibrated (placeholder scale, no offset)\r\n");
}

/* view value averaged over n samples of each channel */
static int view_average(int n, float *v, float *u1, float *u2) {
  int32_t r1, r2;
  if (!hx_average(&ch[0], n, &r1) || !hx_average(&ch[1], n, &r2)) return 0;
  float x1 = tared(&ch[0], r1), x2 = tared(&ch[1], r2);
  if (u1) *u1 = x1 / ch[0].cpu;
  if (u2) *u2 = x2 / ch[1].cpu;
  *v = combine(x1, x2);
  return 1;
}

static void do_calibrate(float knownN) {
  float v;
  if (knownN <= 0) { out("# calibrate needs a positive force in NEWTONS (kg x 9.80665)\r\n"); return; }
  if (!view_average(20, &v, NULL, NULL)) { out("# ERROR: HX711 not responding\r\n"); return; }
  if (fabsf(v) < 1.0f) { out("# ERROR: no change from tare. Send 'd'.\r\n"); return; }
  out("# measured %.2f ustrain (view %lu)\r\n", (double)(v / ref_cpu()), (unsigned long)cfg.view);
  apply_scale(v / knownN, 0.0f);
}

static void side_weight(char *args) {
  while (*args == ' ') args++;
  if (*args == 'm' || *args == 'M') {
    float v;
    if (!view_average(20, &v, &gMidU1, &gMidU2)) { out("# ERROR: HX711 not responding\r\n"); return; }
    gMidValid = 1;
    out("# middle captured: CH1 %.2f ustr, CH2 %.2f ustr. Move the SAME mass to a side strip, send 'g s'.\r\n",
        (double)gMidU1, (double)gMidU2);
    return;
  }
  if (*args == 's' || *args == 'S') {
    float v, u1, u2;
    if (!gMidValid) { out("# send 'g m' first, with the mass on the middle strip\r\n"); return; }
    if (!view_average(20, &v, &u1, &u2)) { out("# ERROR: HX711 not responding\r\n"); return; }
    float den = u2 - gMidU2;
    if (fabsf(den) < 0.5f) { out("# ERROR: CH2 barely changed. Is the mass on a side strip?\r\n"); return; }
    float g = (gMidU1 - u1) / den;
    out("# side captured: CH1 %.2f ustr, CH2 %.2f ustr\r\n", (double)u1, (double)u2);
    if (!(g > 0.2f && g < 5.0f)) { out("# ERROR: g = %.3f is outside 0.2 to 5. Check signs with 'v'.\r\n", (double)g); return; }
    cfg.sideWeight = g; store_save(); gMidValid = 0; filter_reset();
    out("# side weight saved. Recalibrate the combined view (x 0).\r\n");
    print_side();
    return;
  }
  char *end; float w = strtof(args, &end);
  if (end != args) {
    if (!(w > 0.05f && w < 20.0f)) { out("# ERROR: g must be 0.05 to 20\r\n"); return; }
    cfg.sideWeight = w; store_save(); filter_reset();
  }
  print_side();
}

static void flip_sign(char *args) {
  int n = atoi(args);
  if (n != 1 && n != 2) { out("# usage: i 1  or  i 2\r\n"); return; }
  if (n == 1) cfg.sign1 = -cfg.sign1; else cfg.sign2 = -cfg.sign2;
  store_save(); compute_constants(); filter_reset();
  out("# CH%d sign now %+ld (saved). Re-zero and recalibrate.\r\n", n, (long)(n == 1 ? cfg.sign1 : cfg.sign2));
}

static void view_channels(void) {
  for (int i = 0; i < 2; i++) {
    chan_t *c = &ch[i];
    float x = tared(c, c->med);
    out("# %-15s raw %9ld  tared %9.0f  %8.2f ustr  %s\r\n", c->name, (long)c->raw, (double)x, (double)(x / c->cpu),
        (millis() - c->lastMs > NO_DATA_MS) ? "NO DATA" : "ok");
  }
  float u = tared(&ch[0], ch[0].med) / ch[0].cpu + cfg.sideWeight * tared(&ch[1], ch[1].med) / ch[1].cpu;
  out("# combined %.2f ustr   view x %d   status %s\r\n", (double)u, dualStream ? 3 : (int)cfg.view,
      STATUS_NAME[payloadStatus]);
  out("# pose %s, tilt %.1f deg   held load %.2f kg\r\n", imuOk ? POSE_NAME[pose] : "no IMU",
      (double)tiltDeg, (double)heldKg);
}

static void set_view(char *args) {
  char *end; long v = strtol(args, &end, 10);
  if (end == args || v < 0 || v > 3) { print_view(); return; }
  if (v == 3) dualStream = 1;
  else {
    dualStream = 0;
    if ((uint32_t)v != cfg.view) { cfg.view = (uint32_t)v; store_save(); }
  }
  compute_constants(); filter_reset(); lastPrintedStatus = 255; aztSinceMs = 0; ztRun = 0;
  if (dualStream) out("D_HEADER,t_ms,raw1,raw2,ustr1,ustr2,ustr_comb,pose,tilt_deg\r\n");
  else            print_banner();
}

static void set_rated(char *args) {
  char *end; float v = strtof(args, &end);
  if (end != args) {
    if (!(v >= 1.0f && v <= 1000.0f)) { out("# ERROR: rated payload must be 1 to 1000 kg\r\n"); return; }
    cfg.ratedKg = v; store_save(); out("# rated payload saved to flash\r\n");
  }
  print_assist();
}

static void set_comp(char *args) {
  char *e1, *e2;
  float a = strtof(args, &e1);
  if (e1 == args) { print_comp(); return; }
  float b = strtof(e1, &e2);
  if (e2 == e1) b = a;
  if (!valid_r(a) || !valid_r(b)) { out("# ERROR: resistor must be 50 to 10000 ohm\r\n"); return; }
  cfg.compTop = a; cfg.compBot = b; store_save(); compute_constants();
  out("#\r\n# completion resistors updated and saved\r\n");
  print_banner(); bridge_report(); run_diagnostic();
}

static void start_report(float secs) {
  if (secs <= 0) secs = 60;
  if (secs > 3600) secs = 3600;
  repActive = 1; repStartMs = millis(); repEndMs = repStartMs + (uint32_t)(secs * 1000.0f); repN = 0;
  for (int k = 0; k < 3; k++) { repMean[k] = 0; repM2[k] = 0; }
  out("# drift report (CH1, CH2, combined) running for %.0f s. Do not touch the rig.\r\n", (double)secs);
}

static void finish_report(void) {
  static const char *nm[3] = { "CH1 middle  ", "CH2 L+R     ", "combined    " };
  repActive = 0;
  if (repN < 2) { out("# report: not enough samples\r\n"); return; }
  float mins = (millis() - repStartMs) / 60000.0f;
  out("#\r\n# ----- DRIFT REPORT (unfiltered, ustrain) -----\r\n");
  out("# samples %lu over %.2f min\r\n", (unsigned long)repN, (double)mins);
  out("#               noise sd   peak-peak     drift   drift/min\r\n");
  for (int k = 0; k < 3; k++)
    out("# %s %9.3f %11.3f %9.3f %11.3f\r\n", nm[k], (double)sqrtf(repM2[k] / repN), (double)(repMax[k] - repMin[k]),
        (double)(repLast[k] - repFirst[k]), (double)((repLast[k] - repFirst[k]) / mins));
  out("# CH2 should drift no more than CH1. Much more = a lead, joint or bond issue.\r\n");
  out("# ----------------------------------------------\r\n#\r\n");
}

static void update_report(float u1, float u2) {
  if (!repActive) return;
  float v[3] = { u1, u2, u1 + cfg.sideWeight * u2 };
  repN++;
  for (int k = 0; k < 3; k++) {
    if (repN == 1) { repFirst[k] = repMin[k] = repMax[k] = v[k]; }
    repLast[k] = v[k];
    if (v[k] < repMin[k]) repMin[k] = v[k];
    if (v[k] > repMax[k]) repMax[k] = v[k];
    float x = v[k] - repFirst[k], d = x - repMean[k];
    repMean[k] += d / repN; repM2[k] += d * (x - repMean[k]);
  }
  if ((int32_t)(millis() - repEndMs) >= 0) finish_report();
}

/* ------------------------------------------------------------------ IMU pose */
static float dot3(const float *a, const float *b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

static float norm3(float *v) {                   /* normalise in place, return old length */
  float n = sqrtf(dot3(v, v));
  if (n > 1e-6f) { v[0] /= n; v[1] /= n; v[2] /= n; }
  return n;
}

/* refS = gravity direction when standing, refRp = direction it moves when tipping back */
static void pose_refs(void) {
  float s[3] = { cfg.imuStand[0], cfg.imuStand[1], cfg.imuStand[2] };
  float r[3] = { cfg.imuRest[0], cfg.imuRest[1], cfg.imuRest[2] };
  norm3(s); norm3(r);
  float k = dot3(r, s);
  float p[3] = { r[0] - k * s[0], r[1] - k * s[1], r[2] - k * s[2] };
  if (norm3(p) < 0.1f) {                         /* rest pose too close to standing: use -X */
    float d[3] = { -1.0f, 0.0f, 0.0f };
    k = dot3(d, s);
    p[0] = d[0] - k * s[0]; p[1] = d[1] - k * s[1]; p[2] = d[2] - k * s[2];
    if (norm3(p) < 0.1f) { p[0] = 0.0f; p[1] = 1.0f; p[2] = 0.0f; }
  }
  memcpy(refS, s, sizeof(refS));
  memcpy(refRp, p, sizeof(refRp));
  /* tipped back = past TIP_DEG, or past 60 % of a captured rest angle if that is shallower */
  tipLimDeg = TIP_DEG;
  if (cfg.imuRef & 2U) {
    float d = dot3(r, s);
    if (d > 1.0f) d = 1.0f;
    float lim = 0.6f * acosf(d) * 57.29578f;
    if (lim < tipLimDeg) tipLimDeg = lim;
    if (tipLimDeg < STAND_DEG + 3.0f) tipLimDeg = STAND_DEG + 3.0f;
  }
}

/* 50 Hz: take the shared IMU sample, low-pass gravity, classify the pose,
 * flag any movement.  [RT] The observer reads the IMU at ~100 Hz; this only
 * consumes the cached sample and declares IMU_FAULT if it goes stale. */
static void imu_service(void) {
  uint32_t now = millis();
  if (now - imuLastMs < IMU_PERIOD_MS) return;
  float dt = (float)(imuLastMs ? (now - imuLastMs) : IMU_PERIOD_MS) / 1000.0f;
  imuLastMs = now;

  float a[3], g[3]; uint32_t t = 0;
  int fresh = imuGetLatest(a, g, &t) && (now - t) <= IMU_STALE_MS;
  if (!fresh) {
    if (imuOk && ++imuFailN >= 5) {
      imuOk = 0; pose = LP_UNKNOWN;
      out_evt("# LOAD: IMU lost (no fresh sample) - load assist held at 0 until it returns\r\n");
    }
    return;
  }
  if (!imuOk) {
    imuOk = 1; aInit = 0; lastMoveMs = now; movedSinceRow = 1;
    if (imuFound) out_evt("# LOAD: IMU back (%s)\r\n", imuGetSourceName());
    imuFound = 1;
  }
  imuFailN = 0;
  if (!aInit) { memcpy(aLP, a, sizeof(aLP)); aInit = 1; lastMoveMs = now; }
  else {
    float k = dt / (0.2f + dt);                  /* 0.2 s low-pass on gravity */
    for (int i = 0; i < 3; i++) aLP[i] += k * (a[i] - aLP[i]);
  }

  float amag = sqrtf(dot3(a, a));
  float gc[3] = { g[0] - gBias[0], g[1] - gBias[1], g[2] - gBias[2] };
  float gmag = sqrtf(dot3(gc, gc));
  if (fabsf(amag - 1.0f) < 0.02f && gmag < 5.0f)          /* learn gyro offset at rest */
    for (int i = 0; i < 3; i++) gBias[i] += 0.01f * gc[i];
  if (fabsf(amag - 1.0f) > STILL_G || gmag > STILL_DPS) { lastMoveMs = now; movedSinceRow = 1; }

  float u[3] = { aLP[0], aLP[1], aLP[2] };
  if (norm3(u) < 0.5f) { pose = LP_UNKNOWN; return; }    /* not a gravity reading */
  float cs = dot3(u, refS);
  if (cs > 1.0f) cs = 1.0f;
  if (cs < -1.0f) cs = -1.0f;
  tiltDeg = atan2f(dot3(u, refRp), cs) * 57.29578f;
  devDeg  = acosf(cs) * 57.29578f;
  int still = (now - lastMoveMs) >= STILL_MS;
  float standLim = (pose == LP_STANDING) ? STAND_DEG + 1.0f : STAND_DEG;   /* 1 deg hysteresis */
  if (devDeg <= standLim)        pose = LP_STANDING;
  else if (tiltDeg >= tipLimDeg) pose = still ? LP_TIPPED_STILL : LP_TIPPED_MOVING;
  else                           pose = LP_TILTING;
}

/* 'q': IMU status, gating on / off, capture the standing / tipped-back pose */
static void imu_command(char *args) {
  while (*args == ' ') args++;
  char c = *args;
  if (c >= 'A' && c <= 'Z') c = (char)(c + 32);
  if (c == '0' || c == '1') {
    cfg.poseMode = (c == '1') ? 1U : 0U; store_save();
    ztRun = 0; ztTracking = 0; aztSinceMs = 0; lastPrintedStatus = 255;
  } else if (c == 's' || c == 'r') {
    if (!imuOk) { out("# ERROR: no IMU\r\n"); return; }
    if (millis() - lastMoveMs < STILL_MS) {
      out("# ERROR: the trolley is moving. Let it settle, hands off, and try again.\r\n"); return;
    }
    float u[3] = { aLP[0], aLP[1], aLP[2] };
    if (norm3(u) < 0.5f) { out("# ERROR: no gravity reading from the IMU\r\n"); return; }
    if (c == 's') {
      memcpy(cfg.imuStand, u, sizeof(u)); cfg.imuRef |= 1U;
    } else {
      float s[3] = { cfg.imuStand[0], cfg.imuStand[1], cfg.imuStand[2] };
      norm3(s);
      float d = dot3(u, s);
      if (d > 1.0f) d = 1.0f;
      float ang = acosf(d) * 57.29578f;
      if (ang < 2.0f * STAND_DEG + 3.0f) {
        out("# ERROR: only %.1f deg from the standing pose. Let it rest fully tipped back.\r\n", (double)ang);
        return;
      }
      memcpy(cfg.imuRest, u, sizeof(u)); cfg.imuRef |= 2U;
      out("# tipped-back pose is %.1f deg from standing\r\n", (double)ang);
    }
    store_save(); pose_refs();
    out("# %s pose captured and saved\r\n", c == 's' ? "standing" : "tipped-back");
  } else if (c == 'd') {
    imu_ref_defaults(); store_save(); pose_refs();
    out("# pose references back to defaults (measured on the truck)\r\n");
  }
  print_imu();
}

/* ------------------------------------------------------------------ rest zero
 * Once per row (10 Hz). While the trolley is tipped back and still, the zero
 * follows the reading, but only from samples 1.1 to 1.8 s old, so the moment
 * someone grabs it to stand it up never reaches the zero. A step bigger than
 * 'a' kg is refused (something on or touching the plate). */
static void rest_zero_end(void) {
  if (ztTracking) out("# rest zero: held for standing (moved %+.2f kg while tipped back)\r\n", (double)ztEpisodeKg);
  ztTracking = 0; ztRejectSinceMs = 0;
}

static void rest_zero_row(void) {
  for (int i = 0; i < 2; i++) ztHist[i][ztIdx] = ch[i].med;
  ztIdx = (uint8_t)((ztIdx + 1) % ZT_HIST);
  if (ztCount < ZT_HIST) ztCount++;
  int rowStill = gating() && imuOk && pose == LP_TIPPED_STILL && !movedSinceRow;
  movedSinceRow = 0;
  if (!rowStill) { ztRun = 0; rest_zero_end(); return; }
  if (ztRun < 0xFFFFU) ztRun++;
  if (AZT_BAND_KG <= 0 || repActive || ztRun < ZT_RUN || ztCount < ZT_LAG + ZT_N) return;

  /* every sample in the window must be inside the band, not just their average,
   * otherwise a sudden contact could creep in one eighth at a time */
  float band = ztBooted ? AZT_BAND_KG : 5.0f * AZT_BAND_KG;     /* first one after power-up */
  int64_t sum[2] = { 0, 0 };
  float worst = 0.0f;
  for (int k = 0; k < ZT_N; k++) {
    int j = (ztIdx + 2 * ZT_HIST - 1 - ZT_LAG - k) % ZT_HIST;
    sum[0] += ztHist[0][j]; sum[1] += ztHist[1][j];
    float dk = step_kg(combine(tared(&ch[0], ztHist[0][j]), tared(&ch[1], ztHist[1][j])));
    if (fabsf(dk) > fabsf(worst)) worst = dk;
  }
  int32_t cand[2] = { (int32_t)(sum[0] / ZT_N), (int32_t)(sum[1] / ZT_N) };
  float dkg = step_kg(combine(tared(&ch[0], cand[0]), tared(&ch[1], cand[1])));
  if (fabsf(worst) > band) {
    dkg = worst;
    uint32_t now = millis();
    if (!ztRejectSinceMs) ztRejectSinceMs = now ? now : 1U;
    else if (!ztWarned && now - ztRejectSinceMs > 30000U) {
      ztWarned = 1;
      out("# [WARN] rest zero refused 30 s: tipped back it reads %+.1f kg from the zero. "
          "Something on the plate? If empty, send 't'.\r\n", (double)dkg);
    }
    return;
  }
  ztRejectSinceMs = 0; ztWarned = 0;
  if (!ztTracking) { ztTracking = 1; ztEpisodeKg = 0; out("# rest zero: tipped back and still, tracking the zero\r\n"); }
  for (int i = 0; i < 2; i++) if (in_view(i)) ch[i].tare = cand[i];
  ztEpisodeKg += dkg; ztBooted = 1;
  heldKg = 0;                                    /* tipped back and matching the empty zero */
}

/* ------------------------------------------------------------------ commands */
static void handle_command(char *cmd) {
  char *args = cmd + 1;
  float arg = strtof(args, NULL);
  char c0 = cmd[0];
  if (c0 >= 'A' && c0 <= 'Z') c0 = (char)(c0 + 32);
  switch (c0) {
    case 'd': run_diagnostic(); break;
    case 'b': print_banner(); bridge_report(); break;
    case 'v': view_channels(); break;
    case 'x': set_view(args); break;
    case 't': do_tare(0); break;
    case 'z': save_zero(args); break;
    case 'c': do_calibrate(arg); break;
    case 's': set_scale(args); break;
    case 'q': imu_command(args); break;
    case 'u': clear_scales(); break;
    case 'g': side_weight(args); break;
    case 'i': flip_sign(args); break;
    case 'n': start_report(arg); break;
    case 'f': FILTER_TAU_S = arg < 0 ? 0 : arg; filter_reset(); print_mode(); break;
    case 'k': SNAP_USTRAIN = arg < 0 ? 0 : arg; out("# step snap = %.2f ustrain\r\n", (double)SNAP_USTRAIN); break;
    case 'm': { char *e; long v = strtol(args, &e, 10);
                if (e != args) { FILTER_MODE = v ? 1 : 0; filter_reset(); }
                print_mode(); } break;
    case 'h': if (arg > 0) { HOLD_BAND_USTRAIN = arg; filter_reset(); } print_mode(); break;
    case 'a': AZT_BAND_KG = arg < 0 ? 0 : arg; aztSinceMs = 0; print_cal(); break;
    case 'p': set_rated(args); break;
    case 'r': set_comp(args); break;
    default:  print_commands();
  }
}

void loadcellHandleLine(const char *line) {
  char buf[48];
  size_t n = 0;
  while (*line == ' ') line++;                       /* leading spaces */
  for (; *line && n < sizeof(buf) - 1; line++) {
    char c = *line;
    if (c < 32 || c > 126) continue;                 /* junk byte when a port opens */
    buf[n++] = c;
  }
  buf[n] = '\0';
  if (n == 0) return;
  handle_command(buf);
}

/* ------------------------------------------------------------------ payload logic */
/* valid = assist_kg may be used: live and settled, or held from a valid standing reading */
static int payload_valid(void) {
  return payloadStatus == ST_OK || payloadStatus == ST_OVERLOAD || payloadStatus == ST_HELD;
}

static int status_is_fault(uint8_t st) {
  return st == ST_RAILED || st == ST_FROZEN || st == ST_NO_DATA || st == ST_IMPLAUSIBLE || st == ST_IMU_FAULT;
}

static void print_assist_fields(void) {
  out(",%d,%d,%.2f,%.1f,%d,%.1f\r\n", payload_valid() ? 1 : 0, payloadStatus, (double)assistKg,
      (double)(assistKg / cfg.ratedKg * 100.0f), imuOk ? pose : LP_UNKNOWN, (double)(imuOk ? tiltDeg : 0.0f));
}

static void report_status_change(void) {
  if (payloadStatus == lastPrintedStatus) return;
  uint8_t prev = lastPrintedStatus;
  lastPrintedStatus = payloadStatus;
  if (s_console) {
    out("# payload %s: assist %.1f kg = %.0f %% of rated\r\n", STATUS_NAME[payloadStatus],
        (double)assistKg, (double)(assistKg / cfg.ratedKg * 100.0f));
  } else if (status_is_fault(payloadStatus) || (prev != 255 && status_is_fault(prev))) {
    /* [RT] outside the console: only faults and recovery from a fault */
    out_evt("# LOAD %s: assist payload %.1f kg\r\n", STATUS_NAME[payloadStatus], (double)assistKg);
  }
}

static void update_assist(float kgNow, int settled) {
  uint8_t st;
  int railed = 0, frozen = 0;
  for (int i = 0; i < 2; i++) {
    if (!in_view(i)) continue;
    if (labs(ch[i].raw) > HX_RAIL_WARN || ch[i].raw == -1) railed = 1;
    if (ch[i].sameCnt >= FROZEN_N) frozen = 1;
  }
  if (!cfg.calValid[cfg.view])                            st = ST_NO_CAL;
  else if (railed)                                        st = ST_RAILED;
  else if (frozen)                                        st = ST_FROZEN;
  else if (kgNow < -5.0f || kgNow > cfg.ratedKg * 1.5f)   st = ST_IMPLAUSIBLE;
  else if (kgNow > cfg.ratedKg)                           st = ST_OVERLOAD;
  else if (!settled)                                      st = ST_SETTLING;
  else                                                    st = ST_OK;
  payloadStatus = st;

  float target = ASSIST_SF * kgNow;
  if (target < 0) target = 0;
  if (target > cfg.ratedKg) target = cfg.ratedKg;
  if (st == ST_OK || st == ST_OVERLOAD) assistKg = target;          /* settled: follow  */
  else if (st == ST_SETTLING) { if (target < assistKg) assistKg = target; }  /* only drop */
  else assistKg = 0;                                                /* fault: no assist */
  report_status_change();
}

/* With pose gating: live only when standing (and nobody moving it), the last
 * valid standing load is held while tilted or driven, no IMU = no assist. */
static void update_payload(float kgNow, int settled) {
  if (gating()) {
    if (!imuOk) {
      payloadStatus = ST_IMU_FAULT; assistKg = 0; report_status_change(); return;
    }
    if (pose != LP_STANDING) {
      float t = ASSIST_SF * heldKg;
      if (t < 0) t = 0;
      if (t > cfg.ratedKg) t = cfg.ratedKg;
      payloadStatus = ST_HELD; assistKg = t; report_status_change(); return;
    }
    if (millis() - lastMoveMs < STILL_MS) settled = 0;          /* someone is handling it */
  }
  update_assist(kgNow, settled);
  if (gating() && (payloadStatus == ST_OK || payloadStatus == ST_OVERLOAD))
    heldKg = kgNow > 0 ? kgNow : 0;
}

static void update_led(void) {
  uint32_t now = millis();
  if (payloadStatus == ST_OK) { HAL_GPIO_WritePin(LC_LED_PORT, LC_LED_PIN, GPIO_PIN_SET); return; }
  uint32_t period = (payloadStatus == ST_SETTLING || payloadStatus == ST_NO_CAL ||
                     payloadStatus == ST_OVERLOAD || payloadStatus == ST_HELD) ? 500U : 100U;
  if (now - ledMs >= period) { ledMs = now; HAL_GPIO_TogglePin(LC_LED_PORT, LC_LED_PIN); }
}

/* Auto-zero: a stable reading close to zero for 3 s becomes the new zero.
 * Band in kg ('a'). Only runs once the current view is calibrated. */
static void auto_zero(float *offset) {
  if (!(AZT_BAND_KG > 0 && !repActive && cfg.calValid[cfg.view])) { aztSinceMs = 0; return; }
  int stable = FILTER_MODE ? shHolding : 1;
  float kg = counts_to_kg(*offset);
  int inBand = fabsf(kg) < AZT_BAND_KG && fabsf(kg) > AZT_DEADZONE_KG;
  if (!(stable && inBand)) { aztSinceMs = 0; return; }
  if (aztSinceMs == 0) { aztSinceMs = millis(); return; }
  if (millis() - aztSinceMs < (uint32_t)(AZT_TIME_S * 1000.0f)) return;
  for (int i = 0; i < 2; i++) {            /* new tare = recent median average */
    chan_t *c = &ch[i];
    if (!in_view(i) || c->avgN == 0) continue;
    int64_t s = 0;
    for (int k = 0; k < c->avgN; k++) s += c->avgBuf[k];
    c->tare = (int32_t)(s / c->avgN);
  }
  filter_reset(); *offset = 0; aztSinceMs = 0; aztCount++;
  out("# auto-zero: corrected %.2f kg\r\n", (double)kg);
  if (zero_ok(0) && zero_ok(1) && !aztWarned) {
    float d = combine(-tared(&ch[0], cfg.zero1), -tared(&ch[1], cfg.zero2));
    float dk = counts_to_kg(d);
    if (fabsf(dk) > AZT_WARN_KG) {
      aztWarned = 1;
      out("# WARNING: zero has moved %.1f kg from the saved zero. Check the mounting, then 'z'.\r\n", (double)dk);
    }
  }
}

static void sample_channels(int fresh[2]) {
  for (int i = 0; i < 2; i++) {
    chan_t *c = &ch[i];
    fresh[i] = 0;
    if (!hx_ready(c)) continue;
    int32_t s = hx_read(c);
    if (c->seen && s == c->lastRaw) { if (c->sameCnt < 255) c->sameCnt++; }
    else c->sameCnt = 0;
    c->lastRaw = s; c->raw = s; c->seen = 1; c->lastMs = millis();
    c->med = median_stage(c, s);
    c->avgBuf[c->avgIdx] = c->med; c->avgIdx = (uint8_t)((c->avgIdx + 1) % AVG_N);
    if (c->avgN < AVG_N) c->avgN++;
    fresh[i] = 1;
  }
}

static int check_no_data(void) {
  uint32_t now = millis();
  int dead = 0;
  for (int i = 0; i < 2; i++) if (in_view(i) && now - ch[i].lastMs > NO_DATA_MS) dead = 1;
  if (!dead) return 0;
  payloadStatus = ST_NO_DATA; assistKg = 0;
  if (lastPrintedStatus != ST_NO_DATA) {
    lastPrintedStatus = ST_NO_DATA;
    for (int i = 0; i < 2; i++)
      if (in_view(i) && now - ch[i].lastMs > NO_DATA_MS)
        out_evt("# LOAD NO_DATA: %s HX711 not responding, assist payload 0\r\n", ch[i].name);
  }
  if (s_console && !dualStream && now - lastNoDataRowMs > NO_DATA_MS) {
    lastNoDataRowMs = now;
    out("%lu,0,0,0,0,0,0", (unsigned long)now);
    print_assist_fields();
  }
  return 1;
}

/* ------------------------------------------------------------------ public API */
void loadcellInit(void) {
  pins_init();
  int loaded = store_load();
  compute_constants();
  pose_refs();

  /* each bridge: saved zero if it has one, otherwise a quick tare now */
  int32_t keep[2] = { cfg.zero1, cfg.zero2 };
  int present[2] = { hx_wait(&ch[0], 250), hx_wait(&ch[1], 250) };
  if ((!zero_ok(0) && present[0]) || (!zero_ok(1) && present[1])) do_tare_n(0, BOOT_TARE_N);
  for (int i = 0; i < 2; i++) if (zero_ok(i)) ch[i].tare = keep[i];
  ztBooted = 0;                                  /* the power-up zero is not operator-confirmed */
  filter_reset();
  for (int i = 0; i < 2; i++) ch[i].lastMs = millis();

  uart_printf("LOAD: HX711 CH1 %s, CH2 %s | zero %s | view x %lu | pose gating %s\r\n",
              present[0] ? "ok" : "MISSING", present[1] ? "ok" : "MISSING",
              (zero_ok(0) && zero_ok(1)) ? "saved" : "power-up tare",
              (unsigned long)cfg.view, gating() ? "ON" : "OFF");
  if (!loaded) uart_print("LOAD: no saved load-detect settings in flash - defaults\r\n");
  if (loaded == 2) uart_print("LOAD: settings migrated from v4.1 layout\r\n");
  if (!cfg.calValid[cfg.view])
    uart_print("LOAD: view not calibrated -> assist payload 0 kg (NO_CAL). Calibrate in the LD console.\r\n");
}

void loadcellUpdate(void) {
  imu_service();
  int fresh[2];
  sample_channels(fresh);
  update_led();
  if (check_no_data()) return;
  int p = primary_ch();
  if (!fresh[p]) return;                         /* one row per primary conversion (10 Hz) */

  rest_zero_row();                               /* zero follows the reading while tipped back */
  float u1raw = tared(&ch[0], ch[0].raw) / ch[0].cpu;
  float u2raw = tared(&ch[1], ch[1].raw) / ch[1].cpu;
  update_report(u1raw, u2raw);

  if (dualStream) {
    out("D,%lu,%ld,%ld,%.3f,%.3f,%.3f,%d,%.1f\r\n", (unsigned long)millis(), (long)ch[0].raw,
        (long)ch[1].raw, (double)u1raw, (double)u2raw, (double)(u1raw + cfg.sideWeight * u2raw),
        imuOk ? pose : LP_UNKNOWN, (double)(imuOk ? tiltDeg : 0.0f));
    return;
  }

  float view = combine(tared(&ch[0], ch[0].med), tared(&ch[1], ch[1].med));
  float offset = FILTER_MODE ? hold_stage(view) : low_pass_stage(view);
  if (!gating() && cfg.coff[cfg.view] == 0.0f) auto_zero(&offset);   /* standing auto-zero, old rig */

  float ustrain = offset / ref_cpu();
  float newtons = view_newtons(offset);
  float hxmV = offset / HX_COUNTS_PER_MV;
  int settled = FILTER_MODE ? shHolding : ((int32_t)(millis() - fastUntilMs) >= 0);
  measuredKg = newtons / 9.80665f;
  update_payload(measuredKg, settled);

  if (s_console) {
    out("%lu,%ld,%.0f,%.5f,%.1f,%.3f,%.4f", (unsigned long)millis(), (long)ch[p].raw, (double)offset,
        (double)hxmV, (double)(hxmV * 1000.0f), (double)ustrain, (double)newtons);
    print_assist_fields();
  }
}

bool loadcellConsoleActive(void) { return s_console; }

void loadcellSetConsole(bool on) {
  if (on == s_console) return;
  if (on) {
    s_console = true;
    lastPrintedStatus = 255;
    out("# load-detect console ON: bench-rig commands active, assist forced OFF. 'LD' to leave.\r\n");
    out("t_ms,raw,offset_raw,hx_mV,bridge_uV,microstrain,newtons,valid,status,assist_kg,assist_pct,pose,tilt_deg\r\n");
  } else {
    if (repActive) finish_report();
    out("# load-detect console OFF\r\n");
    s_console = false;
    if (dualStream) { dualStream = 0; compute_constants(); filter_reset(); }   /* x 3 is not saved */
    lastPrintedStatus = payloadStatus;            /* don't re-announce the current status */
  }
}

loadcell_status_t loadcellStatus(void)   { return (loadcell_status_t)payloadStatus; }
const char *loadcellStatusName(void)     { return STATUS_NAME[payloadStatus]; }
const char *loadcellStatusShort(void)    { return STATUS_SHORT[payloadStatus]; }
bool  loadcellValid(void)                { return payload_valid() != 0; }
float loadcellAssistKg(void)             { return assistKg; }
float loadcellMeasuredKg(void)           { return measuredKg; }

