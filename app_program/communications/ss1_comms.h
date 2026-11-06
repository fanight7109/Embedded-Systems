/** @file ss1_comms.h
 *
 * @brief Subsystem 1, communications: T1 telemetry and the MQTT bridge.
 *
 * Called only from usermain() (app_main.c): init in phase 1, start in
 * phase 2, test in TEST=1 builds after every task is running.
 *
 * Owner: Buddy 1 (JONATHAN TAN JIANG MING).
 */

#ifndef SS1_COMMS_H
#define SS1_COMMS_H

#include <tk/tkernel.h>

ER   ss1_comms_init(void);
ID   ss1_comms_start(void);

#if (1 == APP_TEST)
void ss1_comms_test(void);
#endif

#endif /* SS1_COMMS_H */

/*** end of file ***/
