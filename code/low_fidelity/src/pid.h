#ifndef PID_H
#define PID_H

#include <Arduino.h>

struct PID {
  float kp;
  float ki;
  float kd;

  float integrator;
  float prev_error;

  float out_min;
  float out_max;
  float i_clamp;
};

void pidInit(PID& p, float kp, float ki, float kd,
             float out_min, float out_max, float i_clamp);

// Reset integrator + derivative history. Call on state transitions
// that invalidate previous history (e.g. entering IDLE).
void pidReset(PID& p);

// Run one control step. dt_s is elapsed seconds since previous call.
// Returns output clamped to [out_min, out_max].
float pidUpdate(PID& p, float setpoint, float measured, float dt_s);

#endif // PID_H
