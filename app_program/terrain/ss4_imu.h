/** @file ss4_imu.h
 *
 * @brief Subsystem 4, terrain: T4, LSM303 on I2C1.
 *
 * Called only from usermain() (app_main.c): init in phase 1, start in
 * phase 2, test in TEST=1 builds after every task is running.
 *
 * Owner: Buddy 4 (GO JUN JIE).
 */

#ifndef SS4_IMU_H
#define SS4_IMU_H

#include <tk/tkernel.h>

ER   ss4_imu_init(void);
ID   ss4_imu_start(void);

#if (1 == APP_TEST)
void ss4_imu_test(void);
#endif

#endif /* SS4_IMU_H */

/*** end of file ***/
