#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Generic PID, ported from the Arduino prototype.  Currently unused on
 * this build - kept for a future closed-loop velocity / load-adaptive
 * torque loop, identical to the prototype so porting stays mechanical. */

typedef struct {
    float kp;
    float ki;
    float kd;

    float integrator;
    float prev_error;

    float out_min;   /* out_min/out_max - final output clamp            */
    float out_max;
    float i_clamp;   /* max magnitude of the integral term, in OUTPUT
                      * units (already ki-scaled)                       */
} PID;

void  pidInit(PID *p, float kp, float ki, float kd,
              float out_min, float out_max, float i_clamp);

void  pidReset(PID *p);

float pidUpdate(PID *p, float setpoint, float measured, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
