/** @file obstacle.c
 *
 * @brief T5: forward ranging, coarse/fine scans and profile reporting.
 *
 * Timing: the receive timeout on mbf_scan_req (60 ms) paces MONITOR pings
 * at the HC-SR04 minimum cycle. During scans each ping is followed by a
 * delay that completes the 60 ms cycle. All waits are kernel waits, so the
 * task uses almost no CPU (WCET 0.3 ms excluding waits).
 *
 * Owner: Buddy 5 (OON JUN KANG SHAWN).
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
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include "obs_prof.h"
#include "obstacle.h"

#define COARSE_STEP         (6u)            /* 30 degrees between angles. */
#define TLM_PERIOD_US       (200000u)
#define SELF_TEST_PINGS     (3u)

typedef struct
{
    uint32_t state        : 4;
    uint32_t confirm_cnt  : 2;
    uint32_t invalid_cnt  : 2;
    uint32_t servo_idx    : 5;
    uint32_t reserved     : 19;
} obstacle_flags_t;

APP_STATIC_ASSERT(sizeof(obstacle_flags_t) == 4u, obstacle_flags_size);

typedef struct
{
    obstacle_flags_t flags;
    obs_prof_scan_t  scan;
    obs_prof_t       prof;
    uint16_t         last_range;
    uint32_t         last_tlm_us;
} obstacle_ctx_t;

static obstacle_ctx_t g_obs;

/* Self-test result: single writer T5. */
static volatile uint8_t g_self_test = OBSTACLE_TEST_PENDING;

/* Latest forward range in MONITOR (adaptive speed): single writer T5,
 * one aligned 16-bit store, MSG_RANGE_NONE when not ranging forward. */
static volatile uint16_t g_fwd_range = MSG_RANGE_NONE;

static void    run_self_test(obstacle_ctx_t * p_ctx);
static void    handle_request(obstacle_ctx_t * p_ctx, uint8_t req);
static void    monitor_ping(obstacle_ctx_t * p_ctx);
static void    move_servo(obstacle_ctx_t * p_ctx, uint8_t idx);
static void    scan_angle(obstacle_ctx_t * p_ctx, uint8_t idx);
static void    report_profile(obstacle_ctx_t * p_ctx);
static uint8_t paced_ping(uint16_t * p_range);
static void    send_nav(msg_nav_t const * p_msg);
static void    send_telemetry(obstacle_ctx_t * p_ctx);

void
obstacle_init (void)
{
    drv_ultrasonic_init();
    drv_servo_init();
}

/*!
 * @brief T5 body. Starts in SELF_TEST, then HOLD until T6 asks MONITOR.
 */
void
obstacle_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    g_obs.flags.servo_idx = APP_SERVO_FWD_IDX;
    run_self_test(&g_obs);

    for (;;)
    {
        uint8_t req = 0u;

        if (rtos_obj_scan_recv(&req, (TMO)APP_PING_PERIOD_MS))
        {
            handle_request(&g_obs, req);
        }
        else if ((OBSTACLE_ST_MONITOR == g_obs.flags.state) ||
                 (OBSTACLE_ST_CONFIRM == g_obs.flags.state))
        {
            monitor_ping(&g_obs);
        }
        else
        {
            /* HOLD, REPORTED or FAULT: no ranging. */
        }

        send_telemetry(&g_obs);
    }
}

uint8_t
obstacle_self_test (void)
{
    return g_self_test;
}

/*!
 * @brief Latest valid forward range while T5 is in MONITOR or CONFIRM.
 *
 * @return Range in mm, or MSG_RANGE_NONE (clear, no echo, scanning, held).
 */
uint16_t
obstacle_forward_mm (void)
{
    return g_fwd_range;
}

/*!
 * @brief Pass if any of three pings produced an echo pulse (F04 input).
 */
static void
run_self_test (obstacle_ctx_t * p_ctx)
{
    bool b_echo = false;

    p_ctx->flags.state = OBSTACLE_ST_SELF_TEST;

    for (uint8_t ping = 0u; (ping < SELF_TEST_PINGS) && (!b_echo); ping++)
    {
        uint16_t range = MSG_RANGE_NONE;

        b_echo = (DRV_ULTRASONIC_MISSING != paced_ping(&range));
    }

    g_self_test        = b_echo ? OBSTACLE_TEST_PASS : OBSTACLE_TEST_FAIL;
    p_ctx->flags.state = OBSTACLE_ST_HOLD;
}

static void
handle_request (obstacle_ctx_t * p_ctx, uint8_t req)
{
    uint8_t const type   = msg_sr_type(req);
    uint8_t const centre = msg_sr_centre(req);

    /* Scans and HOLD point the sensor away from the path: no forward
     * range until the next MONITOR ping. */
    g_fwd_range = MSG_RANGE_NONE;

    switch (type)
    {
        case MSG_SCAN_COARSE:
            p_ctx->flags.state = OBSTACLE_ST_COARSE;
            obs_prof_scan_reset(&p_ctx->scan);

            /* 60, 90 and 120 degrees only. */
            for (uint8_t idx = APP_SCAN_LO_IDX; idx <= APP_SCAN_HI_IDX;
                 idx = (uint8_t)(idx + COARSE_STEP))
            {
                scan_angle(p_ctx, idx);
            }

            report_profile(p_ctx);
        break;

        case MSG_SCAN_FINE:
        {
            /* Fine scan kept inside the 60 .. 120 degree limits. */
            uint8_t const lo_idx = (centre > (APP_SCAN_LO_IDX +
                                              APP_FINE_HALF_SPAN)) ?
                               (uint8_t)(centre - APP_FINE_HALF_SPAN) :
                               (uint8_t)APP_SCAN_LO_IDX;
            uint8_t const hi_idx = ((centre + APP_FINE_HALF_SPAN) <
                                APP_SCAN_HI_IDX) ?
                               (uint8_t)(centre + APP_FINE_HALF_SPAN) :
                               (uint8_t)APP_SCAN_HI_IDX;

            p_ctx->flags.state = OBSTACLE_ST_FINE;

            for (uint8_t idx = lo_idx; idx <= hi_idx; idx++)
            {
                scan_angle(p_ctx, idx);
            }

            report_profile(p_ctx);
        }
        break;

        case MSG_SCAN_MONITOR:
            move_servo(p_ctx, APP_SERVO_FWD_IDX);
            p_ctx->flags.confirm_cnt = 0u;
            p_ctx->flags.invalid_cnt = 0u;
            p_ctx->flags.state       = OBSTACLE_ST_MONITOR;
        break;

        default:
            move_servo(p_ctx, APP_SERVO_FWD_IDX);
            p_ctx->flags.state = OBSTACLE_ST_HOLD;
        break;
    }
}

/*!
 * @brief One forward ping: confirm after 2 valid echoes within the
 *        threshold; F05 after 3 consecutive missing echoes.
 */
static void
monitor_ping (obstacle_ctx_t * p_ctx)
{
    uint16_t       range  = MSG_RANGE_NONE;
    uint8_t const  result = drv_ultrasonic_ping(&range);
    uint16_t const limit  = config_obstacle_mm(config_word_b());

    p_ctx->last_range = range;
    g_fwd_range = (DRV_ULTRASONIC_VALID == result) ? range : MSG_RANGE_NONE;

    if (DRV_ULTRASONIC_MISSING == result)
    {
        p_ctx->flags.invalid_cnt++;

        if (p_ctx->flags.invalid_cnt >= APP_OBS_INVALID_LIMIT)
        {
            fault_set(FAULT_F05_ULTRASONIC);
            p_ctx->flags.state = OBSTACLE_ST_FAULT;
        }
    }
    else if ((DRV_ULTRASONIC_VALID == result) && (range <= limit))
    {
        p_ctx->flags.invalid_cnt = 0u;
        p_ctx->flags.confirm_cnt++;
        p_ctx->flags.state = OBSTACLE_ST_CONFIRM;

        if (p_ctx->flags.confirm_cnt >= APP_OBS_CONFIRM)
        {
            msg_nav_t const msg = { .event = MSG_EV_OBSTACLE, .value = range };

            send_nav(&msg);
            p_ctx->flags.state = OBSTACLE_ST_REPORTED;
        }
    }
    else
    {
        p_ctx->flags.invalid_cnt = 0u;
        p_ctx->flags.confirm_cnt = 0u;
        p_ctx->flags.state       = OBSTACLE_ST_MONITOR;
    }
}

static void
move_servo (obstacle_ctx_t * p_ctx, uint8_t idx)
{
    uint16_t const settle = drv_servo_settle_ms((uint8_t)p_ctx->flags.servo_idx,
                                            idx);

    drv_servo_set_index(idx);
    p_ctx->flags.servo_idx = idx & 0x1Fu;
    (void)tk_dly_tsk((RELTIM)settle);
}

/*!
 * @brief Up to 3 pings at one angle; the first echo or clear reading wins.
 */
static void
scan_angle (obstacle_ctx_t * p_ctx, uint8_t idx)
{
    uint32_t const bit    = (uint32_t)1u << idx;
    uint8_t        result = DRV_ULTRASONIC_MISSING;
    uint16_t       range  = MSG_RANGE_NONE;

    move_servo(p_ctx, idx);
    p_ctx->scan.scanned_mask |= bit;

    for (uint8_t ping = 0u;
         (ping < APP_SCAN_PINGS) && (DRV_ULTRASONIC_MISSING == result); ping++)
    {
        result = paced_ping(&range);
    }

    if (DRV_ULTRASONIC_MISSING != result)
    {
        p_ctx->scan.valid_mask   |= bit;
        p_ctx->scan.range_mm[idx] = range;
    }
}

static void
report_profile (obstacle_ctx_t * p_ctx)
{
    msg_nav_t msg = { .event = MSG_EV_SCAN_FAULT };

    p_ctx->flags.state = OBSTACLE_ST_PROFILING;

    if (obs_prof_build(&p_ctx->scan, &p_ctx->prof))
    {
        obs_prof_to_msg(&p_ctx->prof, &msg);
        p_ctx->flags.state = OBSTACLE_ST_REPORTED;
    }
    else
    {
        p_ctx->flags.state = OBSTACLE_ST_SCAN_FAIL;
    }

    move_servo(p_ctx, APP_SERVO_FWD_IDX);
    send_nav(&msg);
}

/*!
 * @brief Ping, then wait out the rest of the 60 ms HC-SR04 cycle.
 */
static uint8_t
paced_ping (uint16_t * p_range)
{
    uint32_t const start  = board_time_us();
    uint8_t const  result = drv_ultrasonic_ping(p_range);
    uint32_t const used   = (board_time_us() - start) / 1000u;

    if (used < APP_PING_PERIOD_MS)
    {
        (void)tk_dly_tsk((RELTIM)(APP_PING_PERIOD_MS - used));
    }

    return result;
}

static void
send_nav (msg_nav_t const * p_msg)
{
    uint8_t buf[MSG_NAV_MAX_BYTES];
    uint8_t len = msg_nav_pack(p_msg, buf);

    rtos_obj_nav_send(buf, len);
}

/*!
 * @brief About 5 Hz snapshot to T1 (20 bits).
 */
static void
send_telemetry (obstacle_ctx_t * p_ctx)
{
    uint32_t const now = board_time_us();

    if ((now - p_ctx->last_tlm_us) >= TLM_PERIOD_US)
    {
        uint8_t  buf[MSG_TLM_MAX_BYTES] = { 0u };
        uint16_t pos = 0u;

        p_ctx->last_tlm_us = now;
        bitpack_put(buf, &pos, MSG_TLM_OBST, MSG_TLM_SRC_BITS);
        bitpack_put(buf, &pos, p_ctx->flags.state, 4u);
        bitpack_put(buf, &pos, p_ctx->last_range, 11u);
        bitpack_put(buf, &pos, p_ctx->flags.servo_idx, 5u);
        rtos_obj_tlm_send(MSG_TLM_OBST, buf, (uint8_t)bitpack_bytes(pos));
    }
}

/*** end of file ***/
