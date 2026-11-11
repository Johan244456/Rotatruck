#include "pid.h"

void pidInit(PID *p, float kp, float ki, float kd,
             float out_min, float out_max, float i_clamp)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->integrator = 0.0f;
    p->prev_error = 0.0f;
    p->out_min = out_min;
    p->out_max = out_max;
    p->i_clamp = i_clamp;
}

void pidReset(PID *p)
{
    p->integrator = 0.0f;
    p->prev_error = 0.0f;
}

/* Integrator stores ki*sum(error*dt), so i_clamp is in OUTPUT units, not
 * error units. Derivative is on error, so a setpoint step gives a
 * one-sample kick. */
float pidUpdate(PID *p, float setpoint, float measured, float dt_s)
{
    /* Guard the 1/dt in the derivative term; also skips the very first
     * call when dt is unknown. */
    if (dt_s <= 0.0f) return 0.0f;

    float error = setpoint - measured;
    float p_term = p->kp * error;

    p->integrator += p->ki * error * dt_s;
    if (p->integrator >  p->i_clamp) p->integrator =  p->i_clamp;
    if (p->integrator < -p->i_clamp) p->integrator = -p->i_clamp;

    float d_term = p->kd * (error - p->prev_error) / dt_s;
    p->prev_error = error;

    float out = p_term + p->integrator + d_term;
    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;
    return out;
}
