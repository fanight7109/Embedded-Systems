/** @file ss5_obstacle.c
 *
 * @brief Subsystem 5 entry points: ultrasonic and servo set-up, start T5,
 *        on-target tests.
 *
 * Owner: Buddy 5 (OON JUN KANG SHAWN).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "../integration/app_cfg.h"
#include "../app_main.h"
#include "drv_ultrasonic.h"
#include "obstacle.h"
#include "ss5_obstacle.h"

#if (1 == APP_TEST)
#include <tm/tmonitor.h>
#include "../integration/msg_fmt.h"
#include "../integration/test_log.h"
#endif

/*!
 * @brief HC-SR04 trigger/echo pins and the servo PWM (centred).
 *
 * Skipped when the sensor is not fitted: an unconnected ECHO input with
 * its edge interrupt enabled floats and can flood the GPIO interrupt
 * (F12). Left alone, GP5 keeps its reset pull-down.
 */
ER
ss5_obstacle_init (void)
{
    if (0 != APP_ULTRASONIC_FITTED)
    {
        obstacle_init();
    }

    return E_OK;
}

/*!
 * @brief Start T5, unless the sensor is not fitted.
 *
 * @return Task ID, or 0 when APP_ULTRASONIC_FITTED is 0.
 */
ID
ss5_obstacle_start (void)
{
    ID tskid = 0;

    if (0 != APP_ULTRASONIC_FITTED)
    {
        tskid = app_main_start_task(obstacle_task, APP_PRI_OBSTACLE,
                                    APP_STK_OBSTACLE);
    }

    return tskid;
}

#if (1 == APP_TEST)

#define ECHO_100MM_US       (583u)      /* 2 x 100 mm at 343 m/s.      */
#define RANGE_TOL_MM        (3u)
#define LONG_ECHO_US        (30000u)
#define SETTLE_MS           (1500)

/*!
 * @brief S5 tests: echo width conversion and the sensor self-test.
 */
void
ss5_obstacle_test (void)
{
    uint16_t const range = drv_ultrasonic_width_to_mm(ECHO_100MM_US);

    test_log_begin();

    test_log_check("S5-U01", (range >= (100u - RANGE_TOL_MM)) &&
                             (range <= (100u + RANGE_TOL_MM)),
                   "583 us echo converts to 100 mm +/- 3 mm");
    test_log_check("S5-U02",
                   (MSG_RANGE_NONE == drv_ultrasonic_width_to_mm(
                                          LONG_ECHO_US)),
                   "30 ms echo saturates to 'no echo'");

    if (0 != APP_ULTRASONIC_FITTED)
    {
        (void)tk_dly_tsk((RELTIM)SETTLE_MS);
        test_log_check("S5-U03",
                       (OBSTACLE_TEST_PASS == obstacle_self_test()),
                       "HC-SR04 answers and passes self-test");
    }
    else
    {
        (void)tm_printf((UB *)
                        "[TEST] S5-U03 SKIP (APP_ULTRASONIC_FITTED 0)\n");
    }

    test_log_summary("subsystem5");
}

#endif /* APP_TEST */

/*** end of file ***/
