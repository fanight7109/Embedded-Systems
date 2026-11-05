/** @file imu.c
 *
 * @brief T4: sample the accelerometer every 20 ms (Tribunal 1: period
 *        relaxed from 10 ms), run the terrain estimator, report events.
 *
 * Released by the cyc_imu cyclic handler, not tk_dly_tsk(): with the
 * port's 10 ms tick a 20 ms delay sleeps 20 to 30 ms, which would skew
 * the encoder speed that the estimator divides by the period.
 *
 * Fault handling (Section 1.3): three failed or stale (> 50 ms) reads in a
 * row raise F09 (non-critical). Hump and collision events are disabled,
 * the fault is published and the IMU is re-initialised every 1 s; F09 is
 * cleared on recovery.
 *
 * Owner: Buddy 4 (GO JUN JIE).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "../integration/board.h"
#include "../integration/app_cfg.h"
#include "../integration/app_types.h"
#include "../communications/bitpack.h"
#include "../integration/msg_fmt.h"
#include "../integration/config.h"
#include "../integration/fault.h"
#include "../integration/rtos_obj.h"
#include "../motion_control/drv_encoder.h"
#include "../evidence/evidence.h"
#include "drv_lsm303.h"
#include "terrain.h"
#include "imu.h"

#define STALE_US            (50000u)
#define TLM_DIVIDER         (10u)       /* 20 ms x 10 = 5 Hz.             */
#define REINIT_CYCLES       (APP_IMU_REINIT_MS / APP_IMU_PERIOD_MS)
#define PITCH_OFFSET        (512)
#define ONE_G_MG            (1000)
#define UM_PER_MM           (1000)      /* Edges x 1000 -> um after scaling. */

typedef struct
{
    terrain_t     terr;
    terrain_out_t out;
    int32_t       sum_x;
    int32_t       sum_y;
    int32_t       sum_z;
    int16_t       bias_x;
    int16_t       bias_y;
    int16_t       bias_z;
    uint16_t      last_l;
    uint16_t      last_r;
    uint32_t      last_ok_us;
    uint8_t       state;
    uint8_t       bad_reads;
    uint8_t       samples;
    uint8_t       tlm_div;
    uint8_t       sat_count;
} imu_ctx_t;

static imu_ctx_t g_imu;

/* Published results: single writer T4, read by T6 and T1. */
static volatile uint8_t g_self_test = IMU_TEST_PENDING;
static volatile uint8_t g_hump_peak = 0u;
static volatile uint8_t g_hump_n    = 0u;

/* Hump or slope under the car now (adaptive speed): single writer T4. */
static volatile bool    gb_on_hump  = false;

static void sample_once(imu_ctx_t * p_ctx);
static void calibrate_step(imu_ctx_t * p_ctx, int16_t acc_x, int16_t acc_y,
                           int16_t acc_z);
static void monitor_step(imu_ctx_t * p_ctx, int16_t acc_x, int16_t acc_y,
                         int16_t acc_z);
static void handle_bad_read(imu_ctx_t * p_ctx);
static void try_reinit(imu_ctx_t * p_ctx);
static void send_event(uint8_t event, uint16_t value, uint8_t small_a);
static void send_telemetry(imu_ctx_t * p_ctx);

/*!
 * @brief T4 body: initialise the IMU, then sample every 20 ms.
 */
void
imu_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    terrain_init(&g_imu.terr);
    g_imu.state = TERRAIN_ST_INIT;

    if (drv_lsm303_init())
    {
        /* PASS is published after the bias calibration completes. */
        g_imu.state      = TERRAIN_ST_CALIBRATE;
        g_imu.last_ok_us = board_time_us();
    }
    else
    {
        g_self_test = IMU_TEST_FAIL;
        g_imu.state = TERRAIN_ST_DEGRADED;
    }

    for (;;)
    {
        uint32_t release = 0u;
        uint32_t start   = 0u;

        (void)tk_slp_tsk(TMO_FEVR);   /* Released every 20 ms by cyc_imu. */
        start   = board_time_us();
        release = rtos_obj_release_us(RTOS_REL_IMU);
        sample_once(&g_imu);
        send_telemetry(&g_imu);
        evidence_cycle(EV_LOOP_T4, release, start, board_time_us());
    }
}

uint8_t
imu_self_test (void)
{
    return g_self_test;
}

uint8_t
imu_hump_peak_mm (void)
{
    return g_hump_peak;
}

uint8_t
imu_hump_count (void)
{
    return g_hump_n;
}

/*!
 * @brief true while a hump is being measured or the car is climbing or
 *        descending; false when the IMU is degraded or not calibrated.
 */
bool
imu_on_hump (void)
{
    return gb_on_hump;
}

static void
sample_once (imu_ctx_t * p_ctx)
{
    int16_t acc_x = 0;
    int16_t acc_y = 0;
    int16_t acc_z = 0;

    if ((TERRAIN_ST_DEGRADED == p_ctx->state) ||
        (TERRAIN_ST_REINIT == p_ctx->state))
    {
        try_reinit(p_ctx);
    }
    else if (!drv_lsm303_read_accel(&acc_x, &acc_y, &acc_z) ||
             ((board_time_us() - p_ctx->last_ok_us) > STALE_US))
    {
        p_ctx->last_ok_us = board_time_us();
        handle_bad_read(p_ctx);
    }
    else
    {
        p_ctx->bad_reads  = 0u;
        p_ctx->last_ok_us = board_time_us();

        if (TERRAIN_ST_CALIBRATE == p_ctx->state)
        {
            calibrate_step(p_ctx, acc_x, acc_y, acc_z);
        }
        else
        {
            monitor_step(p_ctx, acc_x, acc_y, acc_z);
        }
    }
}

/*!
 * @brief Average 25 samples at rest: X/Y bias to zero, Z to +1 g.
 *
 * Assumes the robot is still and level during the first 0.5 s (A4).
 */
static void
calibrate_step (imu_ctx_t * p_ctx, int16_t acc_x, int16_t acc_y, int16_t acc_z)
{
    p_ctx->sum_x += acc_x;
    p_ctx->sum_y += acc_y;
    p_ctx->sum_z += acc_z;
    p_ctx->samples++;

    if (p_ctx->samples >= APP_IMU_BIAS_SAMPLES)
    {
        int32_t const count = (int32_t)p_ctx->samples;
        uint32_t const snap = drv_encoder_snapshot();

        p_ctx->bias_x = (int16_t)(p_ctx->sum_x / count);
        p_ctx->bias_y = (int16_t)(p_ctx->sum_y / count);
        p_ctx->bias_z = (int16_t)((p_ctx->sum_z / count) - ONE_G_MG);
        p_ctx->last_l = drv_encoder_left(snap);
        p_ctx->last_r = drv_encoder_right(snap);
        p_ctx->state  = TERRAIN_ST_MONITOR;
        terrain_init(&p_ctx->terr);
        g_self_test   = IMU_TEST_PASS;
    }
}

/*!
 * @brief Run the estimator with the encoder travel over the same period.
 */
static void
monitor_step (imu_ctx_t * p_ctx, int16_t acc_x, int16_t acc_y, int16_t acc_z)
{
    uint32_t const snap  = drv_encoder_snapshot();
    uint16_t const now_l = drv_encoder_left(snap);
    uint16_t const now_r = drv_encoder_right(snap);
    int32_t const  dl_um = drv_encoder_left_to_mm(
        (int32_t)drv_encoder_delta(now_l, p_ctx->last_l) * UM_PER_MM);
    int32_t const  dr_um = drv_encoder_right_to_mm(
        (int32_t)drv_encoder_delta(now_r, p_ctx->last_r) * UM_PER_MM);
    uint8_t const  thr   = config_hump_pitch_deg(config_word_b());

    p_ctx->last_l = now_l;
    p_ctx->last_r = now_r;

    terrain_update(&p_ctx->terr, (int16_t)(acc_x - p_ctx->bias_x),
                   (int16_t)(acc_y - p_ctx->bias_y),
                   (int16_t)(acc_z - p_ctx->bias_z), dl_um, dr_um,
                   (uint16_t)APP_IMU_PERIOD_MS, thr, &p_ctx->out);
    p_ctx->state = (uint8_t)p_ctx->terr.flags.state;
    gb_on_hump   = (TERRAIN_ST_CANDIDATE == p_ctx->state) ||
                   (TERRAIN_ST_ESTIMATE == p_ctx->state) ||
                   (TERRAIN_CLS_CLIMBING == p_ctx->out.cls) ||
                   (TERRAIN_CLS_DESCENDING == p_ctx->out.cls);

    if (p_ctx->out.b_impact)
    {
        fault_set(FAULT_F08_COLLISION);
        send_event(MSG_EV_COLLISION, 0u, 0u);
    }

    if (p_ctx->out.b_hump_done)
    {
        g_hump_peak = (uint8_t)p_ctx->terr.flags.hump_max;
        g_hump_n    = (uint8_t)p_ctx->terr.flags.hump_n;
        send_event(MSG_EV_HUMP, p_ctx->out.hump_last, g_hump_n);
    }
}

static void
handle_bad_read (imu_ctx_t * p_ctx)
{
    p_ctx->bad_reads++;

    if (p_ctx->bad_reads >= APP_IMU_FAULT_READS)
    {
        fault_set(FAULT_F09_IMU);
        send_event(MSG_EV_IMU_FAULT, 0u, 1u);
        gb_on_hump       = false;
        p_ctx->state     = TERRAIN_ST_DEGRADED;
        p_ctx->bad_reads = 0u;
        p_ctx->samples   = 0u;
    }
}

/*!
 * @brief Degraded mode: retry initialisation once per second.
 */
static void
try_reinit (imu_ctx_t * p_ctx)
{
    p_ctx->samples++;

    if (p_ctx->samples >= REINIT_CYCLES)
    {
        p_ctx->samples = 0u;
        p_ctx->state   = TERRAIN_ST_REINIT;

        if (drv_lsm303_init())
        {
            uint32_t const snap = drv_encoder_snapshot();

            fault_clear(FAULT_F09_IMU);
            send_event(MSG_EV_IMU_FAULT, 0u, 0u);
            p_ctx->last_l     = drv_encoder_left(snap);
            p_ctx->last_r     = drv_encoder_right(snap);
            p_ctx->last_ok_us = board_time_us();
            p_ctx->state      = TERRAIN_ST_MONITOR;
            terrain_init(&p_ctx->terr);
        }
        else
        {
            p_ctx->state = TERRAIN_ST_DEGRADED;
        }
    }
}

static void
send_event (uint8_t event, uint16_t value, uint8_t small_a)
{
    msg_nav_t msg = { .event = event, .value = value, .small_a = small_a };
    uint8_t   buf[MSG_NAV_MAX_BYTES];
    uint8_t   len = msg_nav_pack(&msg, buf);

    rtos_obj_nav_send(buf, len);
}

/*!
 * @brief 5 Hz snapshot to T1 (35 bits).
 */
static void
send_telemetry (imu_ctx_t * p_ctx)
{
    p_ctx->tlm_div++;

    if (p_ctx->tlm_div >= TLM_DIVIDER)
    {
        uint8_t  buf[MSG_TLM_MAX_BYTES] = { 0u };
        uint16_t pos = 0u;

        p_ctx->tlm_div = 0u;
        bitpack_put(buf, &pos, MSG_TLM_IMU, MSG_TLM_SRC_BITS);
        bitpack_put(buf, &pos, p_ctx->state, 3u);
        bitpack_put(buf, &pos, p_ctx->out.cls, 3u);
        bitpack_put(buf, &pos, p_ctx->out.b_tilt ? 1u : 0u, 1u);
        bitpack_put(buf, &pos, p_ctx->out.hump_cur, 7u);
        bitpack_put(buf, &pos, p_ctx->terr.flags.hump_max, 7u);
        bitpack_put(buf, &pos, p_ctx->terr.flags.hump_n, 4u);
        bitpack_put(buf, &pos, bitpack_offset(p_ctx->out.pitch_x10,
                                              PITCH_OFFSET, 10u,
                                              &p_ctx->sat_count), 10u);
        rtos_obj_tlm_send(MSG_TLM_IMU, buf, (uint8_t)bitpack_bytes(pos));
    }
}

/*** end of file ***/
