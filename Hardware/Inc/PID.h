/* PID controller: position form and incremental PI. */
#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ---- Controller state ---- */
typedef struct {
    float Kp;               /* Proportional gain. */
    float Ki;               /* Integral gain. */
    float Kd;               /* Derivative gain. */

    float setpoint;         /* Target value. */
    float input;            /* Raw feedback value. */
    float output;           /* Controller output. */

    float error;            /* Current error. */
    float last_error;       /* Previous error, e(k-1). */
    float last_error2;      /* Error at e(k-2), retained for compatibility. */
    float integral;         /* Integral accumulator. */

    float prev_output;      /* Previous total output, incremental mode. */
    float output_inc;       /* Last effective output increment. */

    float output_limit;     /* Last requested symmetric output limit. */
    float integral_limit;   /* Last requested integral limit. */

    /* Feedback filter and derivative-on-measurement state for incremental mode. */
    float filtered_input;   /* Low-pass filtered feedback value. */
    float last_input;       /* Previous filtered feedback value. */
    float last_d_term;      /* Previous derivative term. */
    uint8_t filter_ready;   /* 0 = initialise filter on next compute call. */
} PID_Controller;

/* ---- Global instances used by the application ---- */
extern PID_Controller pid_speed_L;    /* Left-wheel speed loop. */
extern PID_Controller pid_speed_R;    /* Right-wheel speed loop. */
extern PID_Controller pid_pos_L;      /* Left-wheel position loop. */
extern PID_Controller pid_pos_R;      /* Right-wheel position loop. */

/* ---- Function declarations ---- */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd);

/* Position-form PID. */
float PID_ComputePos(PID_Controller *pid, float input, float out_limit, float dt_ms);

/* Incremental PID/PI; returns the saturated total output for motor PWM. */
float PID_ComputeInc(PID_Controller *pid, float input, float out_limit, float dt_ms);

void PID_SetSetpoint(PID_Controller *pid, float sp);
void PID_SetTunings(PID_Controller *pid, float Kp, float Ki, float Kd);
void PID_Reset(PID_Controller *pid);

#endif /* __PID_H__ */
