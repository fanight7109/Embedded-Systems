/** @file ss3_line.h
 *
 * @brief Subsystem 3, line and barcode: T3, IR sensors and the ADC.
 *
 * Called only from usermain() (app_main.c): init in phase 1, start in
 * phase 2, test in TEST=1 builds after every task is running.
 *
 * Owner: Buddy 3 (LIM JUN HAO).
 */

#ifndef SS3_LINE_H
#define SS3_LINE_H

#include <tk/tkernel.h>

ER   ss3_line_init(void);
ID   ss3_line_start(void);

#if (1 == APP_TEST)
void ss3_line_test(void);
#endif

#endif /* SS3_LINE_H */

/*** end of file ***/
