/** @file pid.h
 *
 * @brief Fixed-point wheel-speed PID with anti-windup.
 *
 * State is task-private, so it uses unsigned bit-fields (rule R4): the
 * integrator and previous error are stored as 11-bit offset-binary values.
 * Gains are x100 integers from g_config word A.
 *
 * Owner: Buddy 2 (PEH JUN WEI JOEL).
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "../integration/app_types.h"

typedef struct
{
    uint32_t integ_ofs    : 11;   /* integrator + 1024, clamped +/- 1000 */
    uint32_t prev_err_ofs : 11;   /* previous error + 1024               */
    uint32_t reserved     : 10;
} pid_state_t;

APP_STATIC_ASSERT(sizeof(pid_state_t) == 4u, pid_state_size);

void    pid_reset(pid_state_t * p_state);
int32_t pid_step(pid_state_t * p_state, uint16_t kp_x100, uint16_t ki_x100,
                 uint16_t kd_x100, int32_t error);
int32_t pid_integrator(pid_state_t const * p_state);

#endif /* PID_H */

/*** end of file ***/
