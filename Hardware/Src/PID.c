/* PID controller: position form and incremental PI, with output limiting. */
#include "pid.h"
#include <string.h>
#include <math.h>

/* ---- Global PID instances ---- */
PID_Controller pid_speed_L;    /* Left-wheel speed loop. */
PID_Controller pid_speed_R;    /* Right-wheel speed loop. */

/* Initialise all state to zero, then write the tuning coefficients. */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd)
{
    if (pid == NULL) return;

    memset(pid, 0, sizeof(PID_Controller));
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

/* ---- Position-form PID, retained for compatibility with existing users. ----
 * out_limit is the symmetric output limit; dt_ms is clamped to 0.1..100 ms. */
float PID_ComputePos(PID_Controller *pid, float input, float out_limit, float dt_ms)
{
    float dt = dt_ms * 0.001f;              /* ms -> s */

    if (pid == NULL) return 0.0f;

    if (dt < 0.0001f) dt = 0.0001f;         /* Minimum 0.1 ms. */
    if (dt > 0.1f)    dt = 0.1f;            /* Maximum 100 ms. */

    pid->output_limit   = out_limit;
    pid->integral_limit = out_limit * 0.5f;

    pid->input = input;
    pid->error = pid->setpoint - input;

    /* P */
    float p_out = pid->Kp * pid->error;

    /* I, with clamped integral. */
    pid->integral += pid->error * dt;

    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    float i_out = pid->Ki * pid->integral;

    /* D on error, retained for existing position-loop behaviour. */
    float d_out = pid->Kd * (pid->last_error - pid->error) / dt;
    pid->last_error = pid->error;

    pid->output = p_out + i_out + d_out;

    if (pid->output >  pid->output_limit) pid->output =  pid->output_limit;
    if (pid->output < -pid->output_limit) pid->output = -pid->output_limit;

    return pid->output;
}

/* ---- Incremental PI(D on measurement) ----
 * Du(k) = Kp*(e(k)-e(k-1)) + Ki*dt*e(k) + D(k)-D(k-1)
 * D(k)  = -Kd*(yf(k)-yf(k-1))/dt
 *
 * yf is a first-order low-pass version of the encoder feedback.  Filtering
 * the feedback before the derivative prevents the 5 ms encoder quantisation
 * noise from being amplified; applying D to the measurement also removes the
 * setpoint derivative kick.  Output saturation discards excess increments,
 * so the incremental controller cannot wind up.
 */
float PID_ComputeInc(PID_Controller *pid, float input, float out_limit, float dt_ms)
{
    const float dt_min = 0.0001f;       /* 0.1 ms */
    const float dt_max = 0.1000f;       /* 100 ms */
    const float lpf_tau = 0.020f;       /* 20 ms speed-feedback filter */
    float dt;
    float alpha;
    float d_term;
    float inc;
    float prev;
    float out;

    if (pid == NULL) return 0.0f;

    dt = dt_ms * 0.001f;                /* ms -> s */
    if (dt < dt_min) dt = dt_min;
    if (dt > dt_max) dt = dt_max;

    pid->output_limit   = out_limit;
    pid->integral_limit = out_limit * 0.5f;
    pid->input          = input;

    /* Initialise the filter from the first real sample; this avoids an
     * artificial transient after stop/target enable. */
    if (!pid->filter_ready) {
        pid->filtered_input = input;
        pid->last_input     = input;
        pid->last_d_term    = 0.0f;
        pid->filter_ready   = 1U;
    }

    /* Time-constant form works for both nominal 5 ms and delayed samples. */
    alpha = dt / (lpf_tau + dt);
    if (alpha > 1.0f) alpha = 1.0f;
    pid->filtered_input += alpha * (input - pid->filtered_input);

    d_term = -pid->Kd * (pid->filtered_input - pid->last_input) / dt;
    pid->error = pid->setpoint - pid->filtered_input;

    inc = pid->Kp * (pid->error - pid->last_error)
        + pid->Ki * dt * pid->error
        + (d_term - pid->last_d_term);

    pid->last_error2 = pid->last_error;
    pid->last_error  = pid->error;
    pid->last_input  = pid->filtered_input;
    pid->last_d_term = d_term;

    prev = pid->prev_output;
    out  = prev + inc;

    if (out >  pid->output_limit) out =  pid->output_limit;
    if (out < -pid->output_limit) out = -pid->output_limit;

    pid->prev_output = out;
    pid->output      = out;
    pid->output_inc  = out - prev;

    return out;
}

/* ---- Helper functions ---- */
void PID_SetSetpoint(PID_Controller *pid, float sp)
{
    if (pid == NULL) return;
    pid->setpoint = sp;
}

void PID_SetTunings(PID_Controller *pid, float Kp, float Ki, float Kd)
{
    if (pid == NULL) return;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void PID_Reset(PID_Controller *pid)   /* Clear history, keep tuning. */
{
    if (pid == NULL) return;

    pid->error          = 0.0f;
    pid->last_error     = 0.0f;
    pid->last_error2    = 0.0f;
    pid->integral       = 0.0f;
    pid->output         = 0.0f;
    pid->prev_output    = 0.0f;
    pid->output_inc     = 0.0f;
    pid->filtered_input = 0.0f;
    pid->last_input     = 0.0f;
    pid->last_d_term    = 0.0f;
    pid->filter_ready   = 0U;
}
