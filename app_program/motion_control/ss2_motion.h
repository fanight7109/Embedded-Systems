/** @file ss2_motion.h
 *
 * @brief Subsystem 2, motion control: T2, motors (PWM) and encoders.
 *
 * Called only from usermain() (app_main.c): init in phase 1, start in
 * phase 2, test in TEST=1 builds after every task is running.
 *
 * Owner: Buddy 2 (PEH JUN WEI JOEL).
 */

#ifndef SS2_MOTION_H
#define SS2_MOTION_H

#include <tk/tkernel.h>

ER   ss2_motion_init(void);
ID   ss2_motion_start(void);

#if (1 == APP_TEST)
void ss2_motion_test(void);
#endif

#endif /* SS2_MOTION_H */

/*** end of file ***/
