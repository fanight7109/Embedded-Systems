/** @file app_main.c
 *
 * @brief IS05 start-up: board, kernel objects, drivers, tasks, cyclic
 *        handlers, then (TEST=1) the on-target tests.
 *
 * usermain() runs in the kernel's initial task at priority 1. In this
 * single-core build nothing it creates runs until it blocks, so phase 1
 * (objects and drivers) is always complete before any task starts.
 *
 *   1. board_init(): fixes the timer tick (PF-001) before any timing.
 *   2. config defaults and every kernel object (rtos_obj_create).
 *   3. the shared GPIO interrupt, before any module enables a pin IRQ.
 *   4. ssN_init(): each subsystem sets up the peripherals it owns.
 *   5. ssN_start() and T6, then the cyclic handlers that release
 *      T2, T3 and T4 (they need those task IDs).
 *
 * EVIDENCE=1 builds also start the evidence task (test plan INT-09 ..
 * INT-15), and run the INT-14 PWM/servo bench instead of step 5 when GP21
 * is held down at power-up.
 *
 * Owner: Integration.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "integration/app_cfg.h"
#include "integration/board.h"
#include "integration/config.h"
#include "integration/rtos_obj.h"
#include "integration/probe.h"
#include "integration/gpio_isr.h"
#include "integration/button.h"
#include "integration/nav.h"
#include "communications/ss1_comms.h"
#include "motion_control/ss2_motion.h"
#include "line_barcode/ss3_line.h"
#include "terrain/ss4_imu.h"
#include "obstacles/ss5_obstacle.h"
#include "evidence/evidence.h"
#include "evidence/ev_bench.h"
#include "app_main.h"

/*
 * The design (Design Review 2.3) assumes one core: the ISR-to-task hand-offs
 * and single-writer globals are not safe with two cores running tasks.
 */
#if (1 == TK_SUPPORT_SMP)
#error "IS05 firmware must be built single-core: make SMP=0"
#endif

static ER   start_all(void);
static void report(char const * p_what, ER err);

/*!
 * @brief Kernel entry point. Never returns (returning shuts down).
 */
EXPORT INT
usermain (void)
{
    ER err = E_OK;

    board_init();
    (void)tm_printf((UB *)"\n=== IS05 robot car, micro T-Kernel 3.0 ===\n");

    probe_init();
    config_init_defaults();
    evidence_init();
    err = rtos_obj_create();
    report("rtos_obj_create", err);

    if (E_OK == err)
    {
        err = gpio_isr_init();
        report("gpio_isr_init", err);
    }

    if (E_OK == err)
    {
        /* Phase 1: drivers, each by its owning subsystem. */
        report("ss1_comms_init", ss1_comms_init());
        report("ss2_motion_init", ss2_motion_init());
        report("ss3_line_init", ss3_line_init());
        report("ss4_imu_init", ss4_imu_init());
        report("ss5_obstacle_init", ss5_obstacle_init());
        nav_init();
        button_init();

        if (ev_bench_requested())
        {
            /* INT-14: usermain is still the only writer of every PWM. */
            ev_bench_run();

            for (;;)
            {
                (void)tk_slp_tsk(TMO_FEVR);
            }
        }

        /* Phase 2: tasks and cyclic releases. */
        err = start_all();
    }

    if (E_OK != err)
    {
        /* WARNING: no safe operation possible; motors stay off. */
        rtos_obj_sys_set(RTOS_SYS_FAULT);
        (void)tm_printf((UB *)"start-up failed, system held in FAULT\n");
    }
    else
    {
        (void)tm_printf((UB *)"all subsystems started\n");
    }

#if (1 == APP_TEST)
    (void)tm_printf((UB *)"[TEST] === test run start ===\n");
    ss1_comms_test();
    ss2_motion_test();
    ss3_line_test();
    ss4_imu_test();
    ss5_obstacle_test();
    (void)tm_printf((UB *)"[TEST] === test run end ===\n");
#endif

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }

    return 0;
}

/*!
 * @brief Create and start a task (TA_HLNG, kernel-allocated stack).
 *
 * @return Task ID (> 0) or a negative kernel error code.
 */
ID
app_main_start_task (app_main_task_fn_t p_fn, PRI priority, SZ stack_bytes)
{
    T_CTSK ctsk  = { 0 };
    ID     tskid = 0;

    /* Cast: the kernel stores task entry points as the generic FP type. */
    ctsk.task    = (FP)p_fn;
    ctsk.itskpri = priority;
    ctsk.stksz   = stack_bytes;
    ctsk.tskatr  = TA_HLNG | TA_RNG0;
    tskid        = tk_cre_tsk(&ctsk);

    if (tskid > 0)
    {
        ER const err = tk_sta_tsk(tskid, 0);

        if (E_OK != err)
        {
            tskid = (ID)err;
        }
    }

    return tskid;
}

/*!
 * @brief Start T2..T6 and T1, then the cyclic handlers.
 */
static ER
start_all (void)
{
    ER       err       = E_OK;
    ID const motion_id = ss2_motion_start();
    ID const line_id   = ss3_line_start();
    ID const nav_id    = app_main_start_task(nav_task, APP_PRI_NAV,
                                             APP_STK_NAV);
    ID const obs_id    = ss5_obstacle_start();
    ID const imu_id    = ss4_imu_start();
    ID const tlm_id    = ss1_comms_start();
    ID const ev_id     = evidence_start();

    report("T2 motion", (motion_id > 0) ? E_OK : motion_id);
    report("T3 line", (line_id > 0) ? E_OK : line_id);
    report("T6 nav", (nav_id > 0) ? E_OK : nav_id);
    report("T5 obstacle", (obs_id > 0) ? E_OK : obs_id);
    report("T4 imu", (imu_id > 0) ? E_OK : imu_id);
    report("T1 telemetry", (tlm_id > 0) ? E_OK : tlm_id);
    report("evidence", (ev_id >= 0) ? E_OK : ev_id);

    if (0 == APP_ULTRASONIC_FITTED)
    {
        (void)tm_printf((UB *)"T5 obstacle: not fitted, not started\n");
    }

    if (0 == APP_IMU_FITTED)
    {
        (void)tm_printf((UB *)"T4 imu: not fitted, not started\n");
    }

    /* T1 is optional: the car runs without it. T4 and T5 are required
     * only when their sensor is fitted (app_cfg.h). */
    if ((motion_id <= 0) || (line_id <= 0) || (nav_id <= 0) ||
        ((0 != APP_ULTRASONIC_FITTED) && (obs_id <= 0)) ||
        ((0 != APP_IMU_FITTED) && (imu_id <= 0)))
    {
        err = E_SYS;
    }
    else
    {
        err = rtos_obj_start_cyclic(motion_id, line_id, imu_id);
        report("rtos_obj_start_cyclic", err);
    }

    return err;
}

/*!
 * @brief Print a kernel error code if a start-up step failed.
 */
static void
report (char const * p_what, ER err)
{
    if (err < E_OK)
    {
        (void)tm_printf((UB *)"ERROR %s: %d\n", p_what, (INT)err);
    }
}

/*** end of file ***/
