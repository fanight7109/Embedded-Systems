/** @file imu.h
 *
 * @brief T4 IMU / Terrain Monitor task (Design Review 3 and 6.4).
 *
 * T4 is the only user of I2C1. It never commands the motors: humps,
 * collisions and IMU faults go to T6 through mbf_nav, and a collision
 * also raises F08 (critical, flg_sys FAULT).
 *
 * Owner: Buddy 4 (GO JUN JIE).
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

/* Self-test result (single writer T4). */
#define IMU_TEST_PENDING        (0u)
#define IMU_TEST_PASS           (1u)
#define IMU_TEST_FAIL           (2u)

void    imu_task(INT stacd, void * p_exinf);
uint8_t imu_self_test(void);
uint8_t imu_hump_peak_mm(void);
uint8_t imu_hump_count(void);
bool    imu_on_hump(void);

#endif /* IMU_H */

/*** end of file ***/
