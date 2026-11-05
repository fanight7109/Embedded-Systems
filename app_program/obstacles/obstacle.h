/** @file obstacle.h
 *
 * @brief T5 Obstacle Scan & Profiling task (Design Review 3 and 6.3).
 *
 * T5 owns the ultrasonic sensor and the scan servo at all times. In
 * MONITOR it pings straight ahead every 60 ms and reports OBSTACLE after
 * two consecutive valid echoes within the threshold. On request from T6
 * it runs a coarse scan (30, 60, 90, 120, 150 degrees) or a fine scan
 * (5 degree steps, +/- 30 degrees around a centre) and reports PROFILE,
 * or SCAN_FAULT if more than 30 % of the angles stayed invalid.
 *
 * Owner: Buddy 5 (OON JUN KANG SHAWN).
 */

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdint.h>
#include <tk/tkernel.h>

/* Obstacle subsystem states (10 values, 4 bits). */
#define OBSTACLE_ST_HOLD        (0u)
#define OBSTACLE_ST_SELF_TEST   (1u)
#define OBSTACLE_ST_MONITOR     (2u)
#define OBSTACLE_ST_CONFIRM     (3u)
#define OBSTACLE_ST_COARSE      (4u)
#define OBSTACLE_ST_FINE        (5u)
#define OBSTACLE_ST_PROFILING   (6u)
#define OBSTACLE_ST_REPORTED    (7u)
#define OBSTACLE_ST_SCAN_FAIL   (8u)
#define OBSTACLE_ST_FAULT       (9u)

#define OBSTACLE_TEST_PENDING   (0u)
#define OBSTACLE_TEST_PASS      (1u)
#define OBSTACLE_TEST_FAIL      (2u)

void     obstacle_init(void);
void     obstacle_task(INT stacd, void * p_exinf);
uint8_t  obstacle_self_test(void);
uint16_t obstacle_forward_mm(void);

#endif /* OBSTACLE_H */

/*** end of file ***/
