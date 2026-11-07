/** @file nav.h
 *
 * @brief T6 Navigation Coordinator: mission state and behaviour arbiter
 *        (Design Review Sections 3 and 5).
 *
 * T6 is the single behavioural arbiter. It owns no hardware: it turns
 * events from mbf_nav into motion commands (T2) and scan requests (T5),
 * so line, barcode and obstacle behaviours can never command the motors
 * at the same time. States are listed in sys_state.h.
 *
 * Owner: Integration (all buddies); bypass rules from Buddy 5.
 */

#ifndef NAV_H
#define NAV_H

#include <tk/tkernel.h>

void nav_init(void);
void nav_task(INT stacd, void * p_exinf);

#endif /* NAV_H */

/*** end of file ***/
