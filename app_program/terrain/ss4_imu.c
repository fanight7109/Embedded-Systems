/** @file ss4_imu.c
 *
 * @brief Subsystem 4 entry points: start T4 and run S4's on-target tests.
 *
 * I2C1 and the LSM303 are initialised by T4 itself (its owning task), so
 * a missing sensor raises F09 at run time instead of stopping start-up.
 *
 * Owner: Buddy 4 (GO JUN JIE).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "../integration/app_cfg.h"
#include "../app_main.h"
#include "imu.h"
#include "ss4_imu.h"

#if (1 == APP_TEST)
#include <tm/tmonitor.h>
#include "../integration/test_log.h"
#include "terrain.h"
#endif

ER
ss4_imu_init (void)
{
    return E_OK;
}

/*!
 * @brief Start T4, unless the IMU is not fitted (APP_IMU_FITTED = 0).
 *
 * @return Task ID, or 0 when the IMU is not fitted.
 */
ID
ss4_imu_start (void)
{
    ID tskid = 0;

    if (0 != APP_IMU_FITTED)
    {
        tskid = app_main_start_task(imu_task, APP_PRI_IMU, APP_STK_IMU);
    }

    return tskid;
}

#if (1 == APP_TEST)

#define FLAT_SAMPLES        (50u)
#define ONE_G_MG            (1000)
#define PERIOD_MS           (20u)
#define HUMP_PITCH_DEG      (8u)
#define PITCH_TOL_X10       (10)        /* 1.0 degree. */
#define SETTLE_MS           (1500)

/*!
 * @brief S4 tests: terrain model on a flat, still input; sensor self-test.
 */
void
ss4_imu_test (void)
{
    static terrain_t terr;      /* Static: large for the initial task. */
    terrain_out_t    out;

    test_log_begin();

    terrain_init(&terr);

    for (uint8_t idx = 0u; idx < FLAT_SAMPLES; idx++)
    {
        terrain_update(&terr, 0, 0, ONE_G_MG, 0, 0, PERIOD_MS,
                       HUMP_PITCH_DEG, &out);
    }

    test_log_check("S4-U01", (out.pitch_x10 > -PITCH_TOL_X10) &&
                             (out.pitch_x10 < PITCH_TOL_X10),
                   "flat input settles to 0 +/- 1.0 degree pitch");
    test_log_check("S4-U02", (TERRAIN_CLS_STATIONARY == out.cls) &&
                             (!out.b_tilt),
                   "still input is STATIONARY without tilt");

    if (0 != APP_IMU_FITTED)
    {
        (void)tk_dly_tsk((RELTIM)SETTLE_MS);
        test_log_check("S4-U03", (IMU_TEST_PASS == imu_self_test()),
                       "LSM303 answers on I2C1 and passes self-test");
    }
    else
    {
        (void)tm_printf((UB *)"[TEST] S4-U03 SKIP (APP_IMU_FITTED 0)\n");
    }

    test_log_summary("subsystem4");
}

#endif /* APP_TEST */

/*** end of file ***/
