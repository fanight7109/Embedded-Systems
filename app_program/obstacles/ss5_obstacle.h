/** @file ss5_obstacle.h
 *
 * @brief Subsystem 5, obstacles: T5, HC-SR04 and the scan servo.
 *
 * Called only from usermain() (app_main.c): init in phase 1, start in
 * phase 2, test in TEST=1 builds after every task is running.
 *
 * Owner: Buddy 5 (OON JUN KANG SHAWN).
 */

#ifndef SS5_OBSTACLE_H
#define SS5_OBSTACLE_H

#include <tk/tkernel.h>

ER   ss5_obstacle_init(void);
ID   ss5_obstacle_start(void);

#if (1 == APP_TEST)
void ss5_obstacle_test(void);
#endif

#endif /* SS5_OBSTACLE_H */

/*** end of file ***/
