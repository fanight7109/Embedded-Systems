/** @file sys_state.h
 *
 * @brief System (mission) states owned by T6 (Design Review 1.5, area 5).
 *
 * Nine states, 4 bits. Shared by T6 (writer), T1 (command state checks and
 * telemetry) and the laptop dashboard.
 *
 * Owner: Integration.
 */

#ifndef SYS_STATE_H
#define SYS_STATE_H

#define SYS_ST_INIT             (0u)
#define SYS_ST_CALIBRATE        (1u)
#define SYS_ST_READY            (2u)
#define SYS_ST_LINE_FOLLOW      (3u)
#define SYS_ST_BARCODE_ACTION   (4u)
#define SYS_ST_LINE_SEARCH      (5u)
#define SYS_ST_OBSTACLE         (6u)
#define SYS_ST_STOPPED          (7u)
#define SYS_ST_SAFE_STOP        (8u)

#endif /* SYS_STATE_H */

/*** end of file ***/
