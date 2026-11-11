#include "assist_control.h"
#include "motor_control.h"
#include "imu.h"

// --- Tuneable thresholds ---
static const float ACCEL_THRESHOLD    = 0.15f;   // m/s^2 - forward accel to start
static const float VEL_THRESHOLD      = 0.10f;   // rev/s - min motion (output shaft)
static const float STOPPED_VEL        = 0.03f;   // rev/s - fully stopped
static const unsigned long START_CONFIRM_MS = 100;
static const unsigned long STOP_CONFIRM_MS  = 300;
static const unsigned long STOP_RAMP_MS     = 200;
static const int   MAX_PWM            = 200;
static const float LPF_ALPHA          = 0.30f;
static const unsigned long ENCODER_TIMEOUT_MS = 500;
static const unsigned long LOOP_MS    = 20;      // 50 Hz control tick

// --- Duty-cycle intent detection ---
// Motor cycles between ON (drives assist) and OFF (coasts). During OFF the
// motor contributes no torque, so IMU forward acceleration is purely
// (user_force - friction)/mass. Positive -> user is actively pushing; this
// sets the assist level for the following ON window. This is what lets us
// tell motor-driven wheel motion from user-driven wheel motion.
static const unsigned long T_ASSIST_ON_MS  = 250;
static const unsigned long T_ASSIST_OFF_MS = 50;
static const unsigned long OFF_SETTLE_MS   = 15;    // skip this much of OFF to let
                                                    // the motor-disengage transient settle
                                                    // (short because low-PWM coast is fast)

// Ignore small accelerations during OFF - below this it's just noise/slope.
static const float INTENT_AX_THRESHOLD = 0.05f;     // m/s^2

// Translate OFF-window ax to ON-window PWM.
// NOTE: keep these small. Stored rotor/gearbox energy scales with PWM^2
// and drives the motor-coast time during the OFF window - too high and
// the motor is still spinning under its own inertia while we try to
// measure the user's push, which re-contaminates the measurement and
// sends the loop into the "jittering" failure mode. The 5V/100mA regime
// that worked well corresponds to a low PWM ceiling on a regular supply.
static const float ACCEL_ASSIST_GAIN_PWM = 100.0f;  // PWM per m/s^2
static const int   ASSIST_PWM_CEILING    = 60;      // keep low - see note above

// Smooth the per-cycle ax measurement so assist level doesn't step hard
// between cycles.
static const float CYCLE_SMOOTH_LPF = 0.5f;

// Motor dead-zone floor: static-friction breaker when the motor is starting
// from rest. Since we only assist while the wheels are already turning,
// kinetic friction is low and this floor can be small (or zero).
static const int   MIN_DRIVE_PWM_FLOOR = 30;

// E-stop input (active low, INPUT_PULLUP). Nothing wired -> reads HIGH.
static const uint8_t ESTOP_PIN = 8;

// --- State ---
static AssistState state = STATE_IDLE;
static unsigned long last_tick_ms = 0;
static unsigned long state_entered_ms = 0;
static unsigned long start_candidate_ms = 0;
static unsigned long stop_candidate_ms  = 0;
static unsigned long last_motion_ms = 0;

enum AssistPhase { PHASE_ON, PHASE_OFF };
static AssistPhase phase = PHASE_OFF;
static unsigned long phase_start_ms = 0;
static float ax_off_accum = 0.0f;
static int   ax_off_count = 0;
static float ax_off_smoothed = 0.0f;

struct MotorAssist {
  long  last_count;
  float smoothed_vel;   // rev/s, LPF - used for state detection only
};
static MotorAssist m1, m2;

static int ramp_start_pwm_m1 = 0, ramp_start_pwm_m2 = 0;
static int last_pwm_m1 = 0, last_pwm_m2 = 0;

static void resetMotorAssist(MotorAssist& m, long current_count) {
  m.last_count = current_count;
  m.smoothed_vel = 0.0f;
}

static float measureVelocityRevPerSec(MotorAssist& m, long current_count, float dt_s) {
  long dc = current_count - m.last_count;
  m.last_count = current_count;
  if (dt_s <= 0.0f) return 0.0f;
  return ((float)dc) / (float)COUNTS_PER_REV_OUTPUT / dt_s;
}

const char* assistStateName(AssistState s) {
  switch (s) {
    case STATE_IDLE:      return "IDLE";
    case STATE_ASSISTING: return "ASSISTING";
    case STATE_STOPPING:  return "STOPPING";
    case STATE_ESTOP:     return "ESTOP";
    default:              return "UNKNOWN";
  }
}
AssistState assistGetState() { return state; }

static int clampPwm(float v) {
  if (v >  MAX_PWM) v =  MAX_PWM;
  if (v < -MAX_PWM) v = -MAX_PWM;
  return (int)v;
}

static void enterState(AssistState next) {
  state = next;
  state_entered_ms = millis();
  Serial.print("State: ");
  Serial.println(assistStateName(next));

  if (next == STATE_IDLE || next == STATE_ESTOP) {
    stopMotors();
    last_pwm_m1 = 0;
    last_pwm_m2 = 0;
    ax_off_smoothed = 0.0f;
    ax_off_accum = 0.0f;
    ax_off_count = 0;
    start_candidate_ms = 0;
    stop_candidate_ms  = 0;
  } else if (next == STATE_STOPPING) {
    ramp_start_pwm_m1 = last_pwm_m1;
    ramp_start_pwm_m2 = last_pwm_m2;
  } else if (next == STATE_ASSISTING) {
    last_motion_ms = millis();
    // Start in OFF so we measure the user's push before committing any assist.
    phase = PHASE_OFF;
    phase_start_ms = millis();
    ax_off_accum = 0.0f;
    ax_off_count = 0;
    ax_off_smoothed = 0.0f;
    stopMotors();
  }
}

void assistSetup() {
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  resetEncoders();
  resetMotorAssist(m1, getEncoderCountM1());
  resetMotorAssist(m2, getEncoderCountM2());

  Serial.println("Assist: initialising IMU...");
  if (!imuBegin()) {
    Serial.println("Assist: IMU init FAILED (check SDA=A4, SCL=A5).");
    return;
  }
  delay(50);
  Serial.println("Assist: calibrating IMU gravity offset (hold still + level)...");
  imuCalibrate(200);
  Serial.println("Assist: calibration done.");

  last_tick_ms = millis();
  enterState(STATE_IDLE);
}

static void runAssistingDutyCycle(unsigned long now, float ax,
                                  float max_abs_vel) {
  unsigned long since_phase = now - phase_start_ms;

  if (phase == PHASE_OFF) {
    // Motor fully disengaged. Sample user's true contribution via IMU.
    stopMotors();
    last_pwm_m1 = 0;
    last_pwm_m2 = 0;

    if (since_phase >= OFF_SETTLE_MS) {
      ax_off_accum += ax;
      ax_off_count += 1;
    }

    if (since_phase >= T_ASSIST_OFF_MS) {
      float avg_ax = (ax_off_count > 0) ? (ax_off_accum / ax_off_count) : 0.0f;
      // One-pole smoothing across cycles so assist level doesn't jump.
      ax_off_smoothed = CYCLE_SMOOTH_LPF * avg_ax
                      + (1.0f - CYCLE_SMOOTH_LPF) * ax_off_smoothed;

      ax_off_accum = 0.0f;
      ax_off_count = 0;
      phase = PHASE_ON;
      phase_start_ms = now;
    }
    return;
  }

  // PHASE_ON: apply assist sized by the last OFF-window measurement.
  if (since_phase >= T_ASSIST_ON_MS) {
    phase = PHASE_OFF;
    phase_start_ms = now;
    ax_off_accum = 0.0f;
    ax_off_count = 0;
    stopMotors();
    last_pwm_m1 = 0;
    last_pwm_m2 = 0;
    return;
  }

  // User intent == positive ax measured while motor was off.
  float intent = ax_off_smoothed - INTENT_AX_THRESHOLD;
  if (intent <= 0.0f) {
    // User not pushing hard enough in last sample -> no assist this cycle.
    stopMotors();
    last_pwm_m1 = 0;
    last_pwm_m2 = 0;
    return;
  }

  float target_u = ACCEL_ASSIST_GAIN_PWM * intent;
  if (target_u > ASSIST_PWM_CEILING) target_u = ASSIST_PWM_CEILING;

  int cmd = (int)target_u;
  bool moving_fwd = (m1.smoothed_vel > VEL_THRESHOLD) ||
                    (m2.smoothed_vel > VEL_THRESHOLD);
  if (target_u > 5.0f && moving_fwd && cmd < MIN_DRIVE_PWM_FLOOR) {
    cmd = MIN_DRIVE_PWM_FLOOR;
  }

  last_pwm_m1 = clampPwm(cmd);
  last_pwm_m2 = clampPwm(cmd);
  setMotorA(last_pwm_m1);
  setMotorB(last_pwm_m2);
}

void assistUpdate() {
  unsigned long now = millis();
  if (now - last_tick_ms < LOOP_MS) return;
  float dt_s = (now - last_tick_ms) / 1000.0f;
  last_tick_ms = now;

  // --- E-stop ---
  if (digitalRead(ESTOP_PIN) == LOW) {
    if (state != STATE_ESTOP) enterState(STATE_ESTOP);
    return;
  }
  if (state == STATE_ESTOP) {
    enterState(STATE_IDLE);
    return;
  }

  // --- Sensor reads ---
  if (!imuRead()) {
    Serial.println("Assist: IMU read failed, entering IDLE.");
    enterState(STATE_IDLE);
    return;
  }
  float ax = imuGetAxMs2();

  long c1 = getEncoderCountM1();
  long c2 = getEncoderCountM2();
  float raw_v1 = measureVelocityRevPerSec(m1, c1, dt_s);
  float raw_v2 = measureVelocityRevPerSec(m2, c2, dt_s);

  m1.smoothed_vel = LPF_ALPHA * raw_v1 + (1.0f - LPF_ALPHA) * m1.smoothed_vel;
  m2.smoothed_vel = LPF_ALPHA * raw_v2 + (1.0f - LPF_ALPHA) * m2.smoothed_vel;

  float max_abs_vel = max(fabs(m1.smoothed_vel), fabs(m2.smoothed_vel));
  if (max_abs_vel > VEL_THRESHOLD) last_motion_ms = now;

  switch (state) {
    case STATE_IDLE: {
      bool accel_ok = (ax > ACCEL_THRESHOLD);
      bool vel_ok   = (max_abs_vel > VEL_THRESHOLD);
      if (accel_ok && vel_ok) {
        if (start_candidate_ms == 0) start_candidate_ms = now;
        if (now - start_candidate_ms >= START_CONFIRM_MS) {
          enterState(STATE_ASSISTING);
        }
      } else {
        start_candidate_ms = 0;
      }
      break;
    }

    case STATE_ASSISTING: {
      if (now - last_motion_ms > ENCODER_TIMEOUT_MS) {
        Serial.println("Assist: encoder stall/timeout -> STOPPING");
        enterState(STATE_STOPPING);
        break;
      }

      // Stop detection uses the CYCLE-smoothed OFF-window ax (true user
      // signal, motor-free). If that stays <= 0 and wheels are slow, the
      // user has let go.
      bool decel_ok = (ax_off_smoothed <= 0.0f);
      bool slow_ok  = (max_abs_vel < VEL_THRESHOLD);
      if (decel_ok && slow_ok) {
        if (stop_candidate_ms == 0) stop_candidate_ms = now;
        if (now - stop_candidate_ms >= STOP_CONFIRM_MS) {
          enterState(STATE_STOPPING);
          break;
        }
      } else {
        stop_candidate_ms = 0;
      }

      runAssistingDutyCycle(now, ax, max_abs_vel);
      break;
    }

    case STATE_STOPPING: {
      unsigned long since_entry = now - state_entered_ms;
      if (since_entry >= STOP_RAMP_MS ||
          (fabs(m1.smoothed_vel) < STOPPED_VEL &&
           fabs(m2.smoothed_vel) < STOPPED_VEL)) {
        enterState(STATE_IDLE);
        break;
      }
      float k = 1.0f - ((float)since_entry / (float)STOP_RAMP_MS);
      last_pwm_m1 = clampPwm(ramp_start_pwm_m1 * k);
      last_pwm_m2 = clampPwm(ramp_start_pwm_m2 * k);
      setMotorA(last_pwm_m1);
      setMotorB(last_pwm_m2);
      break;
    }

    case STATE_ESTOP:
      break;
  }
}
