/** @file motion.h
 *
 * @brief T2 Motion Control: closed-loop speed, distance and turn control.
 *
 * T2 is the sole owner of the motors (Design Review 6.2). It accepts one
 * command at a time from mbf_motion_cmd (a new command pre-empts), follows
 * the line error from mbf_line_err in LINE_FOLLOW mode, and polls flg_sys
 * every 10 ms so ESTOP/FAULT zero the PWM within 20 ms (NFR9).
 *
 * Owner: Buddy 2 (PEH JUN WEI JOEL).
 */

#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

/* Motion states (5 values, 3 bits). */
#define MOTION_ST_IDLE          (0u)
#define MOTION_ST_EXECUTE       (1u)
#define MOTION_ST_STOPPING      (2u)
#define MOTION_ST_COMPLETE      (3u)
#define MOTION_ST_SAFE_STOP     (4u)

/* Motion modes (4 values, 2 bits). */
#define MOTION_MODE_LINE        (0u)
#define MOTION_MODE_MOVE        (1u)
#define MOTION_MODE_TURN        (2u)
#define MOTION_MODE_REVERSE     (3u)

void     motion_init(void);
void     motion_task(INT stacd, void * p_exinf);
uint16_t motion_odometer_mm(void);

#endif /* MOTION_H */

/*** end of file ***/
