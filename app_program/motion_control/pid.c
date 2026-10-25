/** @file pid.c
 *
 * @brief Fixed-point PID step (32-bit temporaries, rule R6).
 *
 * Owner: Buddy 2 (PEH JUN WEI JOEL).
 */

#include <stdint.h>
#include "../integration/app_cfg.h"
#include "pid.h"

#define STATE_OFFSET    (1024)
#define ERR_LIMIT       (1023)
#define GAIN_SCALE      (100)
#define FIELD_MASK      (0x7FFu)       /* 11-bit bit-field.              */

static int32_t clamp(int32_t value, int32_t limit);

void
pid_reset (pid_state_t * p_state)
{
    p_state->integ_ofs    = (uint32_t)STATE_OFFSET;
    p_state->prev_err_ofs = (uint32_t)STATE_OFFSET;
    p_state->reserved     = 0u;
}

/*!
 * @brief One PID update.
 *
 * @param[in,out] p_state  Task-private state.
 * @param[in]     kp_x100  Proportional gain x 100.
 * @param[in]     ki_x100  Integral gain x 100 (per control cycle).
 * @param[in]     kd_x100  Derivative gain x 100 (per control cycle).
 * @param[in]     error    Set-point minus measurement (mm/s).
 *
 * @return Correction in duty units (0.1 %).
 */
int32_t
pid_step (pid_state_t * p_state, uint16_t kp_x100, uint16_t ki_x100,
          uint16_t kd_x100, int32_t error)
{
    int32_t const err   = clamp(error, ERR_LIMIT);
    int32_t const prev  = (int32_t)p_state->prev_err_ofs - STATE_OFFSET;
    int32_t const integ = clamp(pid_integrator(p_state) + err,
                                APP_PID_INTEG_LIMIT);
    int32_t       sum   = 0;

    sum = ((int32_t)kp_x100 * err) + ((int32_t)ki_x100 * integ) +
          ((int32_t)kd_x100 * (err - prev));

    /* Casts: both values were clamped into 0 .. 2047 above. */
    p_state->integ_ofs    = (uint32_t)(integ + STATE_OFFSET) & FIELD_MASK;
    p_state->prev_err_ofs = (uint32_t)(err + STATE_OFFSET) & FIELD_MASK;

    return sum / GAIN_SCALE;
}

int32_t
pid_integrator (pid_state_t const * p_state)
{
    return (int32_t)p_state->integ_ofs - STATE_OFFSET;
}

static int32_t
clamp (int32_t value, int32_t limit)
{
    int32_t result = value;

    if (value > limit)
    {
        result = limit;
    }
    else if (value < -limit)
    {
        result = -limit;
    }
    else
    {
        /* Within limits. */
    }

    return result;
}

/*** end of file ***/
