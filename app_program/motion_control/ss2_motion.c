/** @file ss2_motion.c
 *
 * @brief Subsystem 2 entry points: motor and encoder set-up, start T2,
 *        on-target tests.
 *
 * Owner: Buddy 2 (PEH JUN WEI JOEL).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "../integration/app_cfg.h"
#include "../app_main.h"
#include "motion.h"
#include "ss2_motion.h"

#if (1 == APP_TEST)
#include "../integration/test_log.h"
#include "drv_encoder.h"
#include "pid.h"
#endif

/*!
 * @brief PWM slices for both motors (stopped) and the encoder inputs.
 */
ER
ss2_motion_init (void)
{
    motion_init();

    return E_OK;
}

ID
ss2_motion_start (void)
{
    return app_main_start_task(motion_task, APP_PRI_MOTION, APP_STK_MOTION);
}

#if (1 == APP_TEST)

#define TEST_MM             (500)
#define TEST_ERR            (50)
#define GAIN_ONE            (100u)      /* 1.00 in x100 fixed point. */

/*!
 * @brief S2 unit tests: encoder scaling and the PID step.
 */
void
ss2_motion_test (void)
{
    pid_state_t   pid;
    int32_t const edges = drv_encoder_mm_to_edges(TEST_MM);
    int32_t const back  = drv_encoder_edges_to_mm(edges);

    test_log_begin();

    test_log_check("S2-U01", (edges > 0) && (back >= (TEST_MM - 5)) &&
                             (back <= (TEST_MM + 5)),
                   "500 mm -> edges -> mm within 5 mm");

    pid_reset(&pid);
    test_log_check("S2-U02",
                   (TEST_ERR == pid_step(&pid, GAIN_ONE, 0u, 0u, TEST_ERR)),
                   "P-only step with Kp 1.00 returns the error");

    pid_reset(&pid);
    test_log_check("S2-U03", (0 == pid_step(&pid, GAIN_ONE, GAIN_ONE,
                                            GAIN_ONE, 0)),
                   "zero error from reset gives zero output");

    test_log_check("S2-U04", (motion_odometer_mm() < 10u),
                   "car is stationary at test start (odometer < 10 mm)");

    test_log_summary("subsystem2");
}

#endif /* APP_TEST */

/*** end of file ***/
