/** @file motion.c
 *
 * @brief T2 Motion Control task (Design Review 3, 6.2; FR1, FR3, NFR9).
 *
 * Each 10 ms release: poll flg_sys, read the encoder pair, take the newest
 * command and line error, compute wheel set-points for the active mode,
 * close the speed loop with feed-forward plus PID, check F01/F02/F12/F14,
 * then write the motors. Stops are either commanded (ramp in 300 ms, not
 * latched) or fault stops (PWM zero at once, latched SAFE STOP).
 *
 * Owner: Buddy 2 (PEH JUN WEI JOEL).
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
#include "../integration/probe.h"
#include "drv_encoder.h"
#include "drv_motor.h"
#include "pid.h"
#include "motion.h"

#define PERIOD_US           ((uint32_t)APP_CTRL_PERIOD_MS * 1000u)
#define STALE_LIMIT         (APP_LINE_STALE_MS / APP_CTRL_PERIOD_MS)
#define STALL_LIMIT         (APP_ENC_FAULT_MS / APP_CTRL_PERIOD_MS)
#define TLM_DIVIDER         (20u)       /* 10 ms x 20 = 5 Hz snapshots.   */
#define ACCEL_STEP_MMS      (20)        /* 2 m/s^2 set-point ramp.        */
#define SETTLE_CYCLES       (50u)       /* Ramp allowance for F02.        */
#define DECEL_GAIN          (3)         /* mm/s per mm still to go.       */
#define STOPPED_MMS         (20)
#define SPEED_OFFSET        (512)
#define PI_X10000           (31416)
#define MAX_TURN_DEG        (180u)
#define CYCLES_MAX          (0xFFFFu)

/* Task-private persistent state (rule R4): flags and small counters. */
typedef struct
{
    uint32_t state        : 3;
    uint32_t mode         : 2;
    uint32_t seq          : 4;
    uint32_t miss_cnt     : 2;
    uint32_t stale        : 4;
    uint32_t stall_l      : 5;
    uint32_t stall_r      : 5;
    uint32_t turn_right   : 1;
    uint32_t line_valid   : 1;
    uint32_t tlm_div      : 5;
} motion_flags_t;

APP_STATIC_ASSERT(sizeof(motion_flags_t) == 4u, motion_flags_size);

typedef struct
{
    motion_flags_t flags;
    pid_state_t    pid_l;
    pid_state_t    pid_r;
    uint16_t       hist_l[APP_SPEED_WIN];
    uint16_t       hist_r[APP_SPEED_WIN];
    uint16_t       last_l;
    uint16_t       last_r;
    uint16_t       start_l;
    uint16_t       start_r;
    uint16_t       target_edges;
    uint16_t       elapsed;
    uint16_t       limit;
    uint16_t       cmd_speed;
    int16_t        set_l;
    int16_t        set_r;
    int16_t        meas_l;
    int16_t        meas_r;
    int16_t        line_err;
    int16_t        prev_err;
    int16_t        cruise;
    uint8_t        hist_idx;
    uint8_t        sat_count;
    bool           b_still_l;   /* No left edge in the last cycle.      */
    bool           b_still_r;
    uint32_t       odo_um;
} motion_ctx_t;

static motion_ctx_t g_motion;

/* Published odometer: single writer T2, one aligned 16-bit store. */
static volatile uint16_t g_odometer_mm = 0u;

static void    run_cycle(motion_ctx_t * p_ctx);
static void    enter_safe_stop(motion_ctx_t * p_ctx, UINT sys_flags);
static void    read_encoders(motion_ctx_t * p_ctx);
static void    accept_command(motion_ctx_t * p_ctx, uint32_t cmd);
static void    start_manoeuvre(motion_ctx_t * p_ctx, uint8_t mode,
                               uint32_t distance_mm);
static void    begin_stop(motion_ctx_t * p_ctx);
static void    update_line(motion_ctx_t * p_ctx);
static void    plan_line_follow(motion_ctx_t * p_ctx);
static void    plan_manoeuvre(motion_ctx_t * p_ctx);
static void    plan_stopping(motion_ctx_t * p_ctx);
static void    drive_wheels(motion_ctx_t * p_ctx);
static int16_t wheel_duty(pid_state_t * p_pid, int16_t set, int16_t meas,
                          uint32_t word_a);
static void    check_stall(motion_ctx_t * p_ctx, int16_t duty_l,
                           int16_t duty_r);
static uint8_t stall_step(uint8_t count, bool b_stalled);
static void    send_event(uint8_t event, uint8_t seq);
static void    send_telemetry(motion_ctx_t * p_ctx);
static int16_t approach(int16_t current, int16_t target, int16_t step);
static int16_t abs16(int16_t value);

/*!
 * @brief Configure the motors and encoders. Called before tasks start.
 */
void
motion_init (void)
{
    drv_motor_init();
    drv_encoder_init();
    /* g_motion is zero-initialised as a static. */
    pid_reset(&g_motion.pid_l);
    pid_reset(&g_motion.pid_r);
}

/*!
 * @brief T2 body, released every 10 ms by cyc_ctrl (tk_wup_tsk).
 */
void
motion_task (INT stacd, void * p_exinf)
{
    uint32_t const snap = drv_encoder_snapshot();

    (void)stacd;
    (void)p_exinf;

    g_motion.last_l = drv_encoder_left(snap);
    g_motion.last_r = drv_encoder_right(snap);

    for (uint8_t idx = 0u; idx < APP_SPEED_WIN; idx++)
    {
        g_motion.hist_l[idx] = g_motion.last_l;
        g_motion.hist_r[idx] = g_motion.last_r;
    }

    for (;;)
    {
        uint32_t release = 0u;
        uint32_t start   = 0u;

        (void)tk_slp_tsk(TMO_FEVR);
        probe_high();
        start   = board_time_us();
        release = rtos_obj_release_us(RTOS_REL_MOTION);

        run_cycle(&g_motion);

        if (rtos_obj_missed_deadline(release, start, board_time_us(),
                                     PERIOD_US))
        {
            g_motion.flags.miss_cnt++;

            if (g_motion.flags.miss_cnt >= APP_OVERRUN_LIMIT)
            {
                fault_set(FAULT_F12_DEADLINE);
            }
        }
        else
        {
            g_motion.flags.miss_cnt = 0u;
        }

        probe_low();
    }
}

/*!
 * @brief Latest odometer reading in mm (wraps after 65.5 m, A8).
 */
uint16_t
motion_odometer_mm (void)
{
    return g_odometer_mm;
}

static void
run_cycle (motion_ctx_t * p_ctx)
{
    UINT const sys = rtos_obj_sys_peek();
    uint32_t   cmd = 0u;

    if ((0u != (sys & (RTOS_SYS_ESTOP | RTOS_SYS_FAULT))) ||
        (MOTION_ST_SAFE_STOP == p_ctx->flags.state))
    {
        enter_safe_stop(p_ctx, sys);
    }
    else
    {
        read_encoders(p_ctx);

        if (rtos_obj_motion_recv(&cmd))
        {
            accept_command(p_ctx, cmd);
        }

        if (0u != (sys & RTOS_SYS_STOP_REQ))
        {
            rtos_obj_sys_clear(RTOS_SYS_STOP_REQ);
            begin_stop(p_ctx);
        }

        update_line(p_ctx);

        switch (p_ctx->flags.state)
        {
            case MOTION_ST_EXECUTE:
                if (MOTION_MODE_LINE == p_ctx->flags.mode)
                {
                    plan_line_follow(p_ctx);
                }
                else
                {
                    plan_manoeuvre(p_ctx);
                }
            break;

            case MOTION_ST_STOPPING:
                plan_stopping(p_ctx);
            break;

            default:
                /* IDLE and COMPLETE hold the wheels at zero. */
                p_ctx->flags.state = MOTION_ST_IDLE;
                p_ctx->set_l = 0;
                p_ctx->set_r = 0;
            break;
        }

        drive_wheels(p_ctx);
    }

    send_telemetry(p_ctx);
}

/*!
 * @brief Fault or ESTOP: PWM to zero now and latch (NFR9, FR8).
 */
static void
enter_safe_stop (motion_ctx_t * p_ctx, UINT sys_flags)
{
    drv_motor_stop_now();

    if (MOTION_ST_SAFE_STOP != p_ctx->flags.state)
    {
        p_ctx->flags.state = MOTION_ST_SAFE_STOP;
        p_ctx->set_l = 0;
        p_ctx->set_r = 0;

        if (0u != (sys_flags & RTOS_SYS_ESTOP))
        {
            fault_set(FAULT_F13_ESTOP);
        }
    }
}

/*!
 * @brief One aligned load of both counters; speeds over the window.
 */
static void
read_encoders (motion_ctx_t * p_ctx)
{
    uint32_t const snap   = drv_encoder_snapshot();
    uint16_t const now_l  = drv_encoder_left(snap);
    uint16_t const now_r  = drv_encoder_right(snap);
    int16_t const  d_left     = drv_encoder_delta(now_l, p_ctx->last_l);
    int16_t const  d_right     = drv_encoder_delta(now_r, p_ctx->last_r);
    uint8_t const  oldest = p_ctx->hist_idx;
    int32_t const  win_ms = (int32_t)(APP_SPEED_WIN * APP_CTRL_PERIOD_MS);
    int32_t        span_l = drv_encoder_delta(now_l, p_ctx->hist_l[oldest]);
    int32_t        span_r = drv_encoder_delta(now_r, p_ctx->hist_r[oldest]);

    /* Edges -> um over the window -> um/ms, which is mm/s. */
    span_l = drv_encoder_edges_to_mm(span_l * 1000) / win_ms;
    span_r = drv_encoder_edges_to_mm(span_r * 1000) / win_ms;
    p_ctx->meas_l = (int16_t)span_l;
    p_ctx->meas_r = (int16_t)span_r;

    p_ctx->hist_l[oldest] = now_l;
    p_ctx->hist_r[oldest] = now_r;
    p_ctx->hist_idx = (uint8_t)((oldest + 1u) % APP_SPEED_WIN);

    p_ctx->odo_um += (uint32_t)drv_encoder_edges_to_mm(
                         ((int32_t)abs16(d_left) + (int32_t)abs16(d_right)) *
                         500);
    g_odometer_mm = (uint16_t)(p_ctx->odo_um / 1000u);

    p_ctx->b_still_l = (0 == d_left);
    p_ctx->b_still_r = (0 == d_right);
    p_ctx->last_l = now_l;
    p_ctx->last_r = now_r;
}

/*!
 * @brief Validate and start a command; invalid ones are rejected to T6
 *        and do not stop the robot (Design Review 6.2).
 */
static void
accept_command (motion_ctx_t * p_ctx, uint32_t cmd)
{
    uint8_t const  type  = msg_mc_type(cmd);
    uint16_t const param = msg_mc_param(cmd);
    bool           b_ok  = true;

    p_ctx->flags.seq  = msg_mc_seq(cmd) & 0xFu;
    p_ctx->cmd_speed  = msg_mc_speed(cmd);

    switch (type)
    {
        case MSG_MC_STOP:
            begin_stop(p_ctx);
        break;

        case MSG_MC_LINE_FOLLOW:
            p_ctx->flags.mode  = MOTION_MODE_LINE;
            p_ctx->flags.state = MOTION_ST_EXECUTE;
            p_ctx->flags.stale = 0u;
        break;

        case MSG_MC_MOVE_DIST:
        case MSG_MC_REVERSE:
            b_ok = (0u != param);

            if (b_ok)
            {
                start_manoeuvre(p_ctx, (MSG_MC_REVERSE == type) ?
                                MOTION_MODE_REVERSE : MOTION_MODE_MOVE,
                                param);
            }
        break;

        case MSG_MC_TURN_ANGLE:
            b_ok = ((0u != param) && (param <= MAX_TURN_DEG));

            if (b_ok)
            {
                uint32_t const arc_mm = ((uint32_t)PI_X10000 *
                                         APP_TRACK_WIDTH_MM * param) /
                                        (360u * 10000u);

                p_ctx->flags.turn_right = msg_mc_dir(cmd) & 0x1u;
                start_manoeuvre(p_ctx, MOTION_MODE_TURN, arc_mm);
            }
        break;

        default:
            b_ok = false;
        break;
    }

    if (!b_ok)
    {
        send_event(MSG_EV_REJECT, msg_mc_seq(cmd));
    }
}

/*!
 * @brief Begin a distance-based manoeuvre and arm the F02 timer
 *        (2 x expected duration plus ramp allowance).
 */
static void
start_manoeuvre (motion_ctx_t * p_ctx, uint8_t mode, uint32_t distance_mm)
{
    uint32_t const expected = ((distance_mm * 1000u) /
                               ((uint32_t)p_ctx->cmd_speed *
                                APP_CTRL_PERIOD_MS)) + SETTLE_CYCLES;
    uint32_t const limit    = 2u * expected;

    p_ctx->flags.mode   = mode & 0x3u;
    p_ctx->flags.state  = MOTION_ST_EXECUTE;
    p_ctx->target_edges =
        (uint16_t)drv_encoder_mm_to_edges((int32_t)distance_mm);
    p_ctx->start_l      = p_ctx->last_l;
    p_ctx->start_r      = p_ctx->last_r;
    p_ctx->elapsed      = 0u;
    p_ctx->limit        = (uint16_t)((limit > CYCLES_MAX) ? CYCLES_MAX
                                                          : limit);
}

static void
begin_stop (motion_ctx_t * p_ctx)
{
    if ((MOTION_ST_EXECUTE == p_ctx->flags.state) ||
        (MOTION_ST_STOPPING == p_ctx->flags.state))
    {
        p_ctx->flags.state = MOTION_ST_STOPPING;
    }
    else
    {
        p_ctx->flags.state = MOTION_ST_IDLE;
        send_event(MSG_EV_STOPPED, p_ctx->flags.seq);
    }
}

/*!
 * @brief Take the newest line error and age it for F14.
 */
static void
update_line (motion_ctx_t * p_ctx)
{
    uint16_t msg = 0u;

    if (rtos_obj_line_recv_latest(&msg))
    {
        p_ctx->prev_err         = p_ctx->line_err;
        p_ctx->line_err         = msg_le_err(msg);
        p_ctx->flags.line_valid = msg_le_valid(msg) ? 1u : 0u;
        p_ctx->flags.stale      = 0u;
    }
    else if (p_ctx->flags.stale < 15u)
    {
        p_ctx->flags.stale++;
    }
    else
    {
        /* Saturated. */
    }
}

/*!
 * @brief LINE_FOLLOW: PD steering on the line error around the cruise
 *        speed. Positive error means the line is to the right.
 */
static void
plan_line_follow (motion_ctx_t * p_ctx)
{
    int32_t steer = 0;

    if (p_ctx->flags.stale >= STALE_LIMIT)
    {
        fault_set(FAULT_F14_CMD_TIMEOUT);
    }

    p_ctx->cruise = approach(p_ctx->cruise, (int16_t)p_ctx->cmd_speed,
                             ACCEL_STEP_MMS);

    if (0u != p_ctx->flags.line_valid)
    {
        steer = (((int32_t)APP_LINE_KP_X10 * p_ctx->line_err) +
                 ((int32_t)APP_LINE_KD_X10 *
                  (p_ctx->line_err - p_ctx->prev_err))) / 10;
    }

    p_ctx->set_l = (int16_t)(p_ctx->cruise + steer);
    p_ctx->set_r = (int16_t)(p_ctx->cruise - steer);
}

/*!
 * @brief MOVE, REVERSE and TURN: encoder progress with deceleration,
 *        straight-line correction and the F02 timeout.
 */
static void
plan_manoeuvre (motion_ctx_t * p_ctx)
{
    int32_t const prog_l   = drv_encoder_delta(p_ctx->last_l, p_ctx->start_l);
    int32_t const prog_r   = drv_encoder_delta(p_ctx->last_r, p_ctx->start_r);
    int32_t const done = (abs16((int16_t)prog_l) + abs16((int16_t)prog_r)) / 2;
    int32_t const left = (int32_t)p_ctx->target_edges - done;
    int32_t       vel  = 0;

    p_ctx->elapsed++;

    if (p_ctx->elapsed > p_ctx->limit)
    {
        fault_set(FAULT_F02_MANOEUVRE);
    }

    if (left <= 0)
    {
        p_ctx->flags.state = MOTION_ST_COMPLETE;
        p_ctx->set_l = 0;
        p_ctx->set_r = 0;
        p_ctx->cruise = 0;
        send_event(MSG_EV_MOVE_DONE, p_ctx->flags.seq);
    }
    else
    {
        vel = APP_MIN_MOVE_SPEED + (drv_encoder_edges_to_mm(left) * DECEL_GAIN);
        vel = (vel > (int32_t)p_ctx->cmd_speed) ? (int32_t)p_ctx->cmd_speed
                                                : vel;
        p_ctx->cruise = approach(p_ctx->cruise, (int16_t)vel,
                                 ACCEL_STEP_MMS);

        if (MOTION_MODE_TURN == p_ctx->flags.mode)
        {
            int16_t const sign = (0u != p_ctx->flags.turn_right) ? 1 : -1;

            p_ctx->set_l = (int16_t)(sign * p_ctx->cruise);
            p_ctx->set_r = (int16_t)(-sign * p_ctx->cruise);
        }
        else
        {
            int16_t const sign = (MOTION_MODE_REVERSE == p_ctx->flags.mode)
                                 ? -1 : 1;
            int32_t const skew = drv_encoder_edges_to_mm(
                abs16((int16_t)prog_l) - abs16((int16_t)prog_r));
            int32_t const corr = (APP_STRAIGHT_K_X10 * skew) / 10;

            p_ctx->set_l = (int16_t)(sign * (p_ctx->cruise - corr));
            p_ctx->set_r = (int16_t)(sign * (p_ctx->cruise + corr));
        }
    }
}

/*!
 * @brief Commanded stop: ramp both wheels to zero within 300 ms.
 */
static void
plan_stopping (motion_ctx_t * p_ctx)
{
    int16_t const step = (int16_t)(((uint32_t)MSG_SPEED_MAX *
                                    APP_CTRL_PERIOD_MS) / APP_STOP_RAMP_MS);

    p_ctx->set_l  = approach(p_ctx->set_l, 0, step);
    p_ctx->set_r  = approach(p_ctx->set_r, 0, step);
    p_ctx->cruise = 0;

    if ((0 == p_ctx->set_l) && (0 == p_ctx->set_r) &&
        (abs16(p_ctx->meas_l) < STOPPED_MMS) &&
        (abs16(p_ctx->meas_r) < STOPPED_MMS))
    {
        p_ctx->flags.state = MOTION_ST_IDLE;
        send_event(MSG_EV_STOPPED, p_ctx->flags.seq);
    }
}

/*!
 * @brief Close both speed loops and write the motors.
 */
static void
drive_wheels (motion_ctx_t * p_ctx)
{
    uint32_t const word_a = config_word_a();
    int16_t const  duty_l = wheel_duty(&p_ctx->pid_l, p_ctx->set_l,
                                       p_ctx->meas_l, word_a);
    int16_t const  duty_r = wheel_duty(&p_ctx->pid_r, p_ctx->set_r,
                                       p_ctx->meas_r, word_a);

    check_stall(p_ctx, duty_l, duty_r);
    drv_encoder_set_direction_hint(duty_l >= 0, duty_r >= 0);
    drv_motor_drive((uint16_t)abs16(duty_l), duty_l >= 0,
                (uint16_t)abs16(duty_r), duty_r >= 0);
}

/*!
 * @brief Feed-forward plus PID for one wheel; signed duty out.
 */
static int16_t
wheel_duty (pid_state_t * p_pid, int16_t set, int16_t meas, uint32_t word_a)
{
    int32_t duty = 0;

    if (0 == set)
    {
        pid_reset(p_pid);
    }
    else
    {
        duty = (((int32_t)set * (int32_t)APP_FF_DUTY_PER_MMS_X10) / 10) +
               pid_step(p_pid,
                        config_gain_x100(word_a, CONFIG_KP_SHIFT),
                        config_gain_x100(word_a, CONFIG_KI_SHIFT),
                        config_gain_x100(word_a, CONFIG_KD_SHIFT),
                        (int32_t)set - meas);

        if (duty > (int32_t)APP_DUTY_MAX)
        {
            duty = (int32_t)APP_DUTY_MAX;
        }
        else if (duty < -(int32_t)APP_DUTY_MAX)
        {
            duty = -(int32_t)APP_DUTY_MAX;
        }
        else
        {
            /* In range. */
        }
    }

    return (int16_t)duty;
}

/*!
 * @brief F01: |PWM| above 30 % with no count change for 200 ms.
 */
static void
check_stall (motion_ctx_t * p_ctx, int16_t duty_l, int16_t duty_r)
{
    int16_t const limit = (int16_t)APP_ENC_FAULT_DUTY;

    p_ctx->flags.stall_l = stall_step((uint8_t)p_ctx->flags.stall_l,
                                      p_ctx->b_still_l &&
                                      (abs16(duty_l) > limit)) & 0x1Fu;
    p_ctx->flags.stall_r = stall_step((uint8_t)p_ctx->flags.stall_r,
                                      p_ctx->b_still_r &&
                                      (abs16(duty_r) > limit)) & 0x1Fu;

    if ((p_ctx->flags.stall_l >= STALL_LIMIT) ||
        (p_ctx->flags.stall_r >= STALL_LIMIT))
    {
        fault_set(FAULT_F01_ENCODER);
    }
}

static uint8_t
stall_step (uint8_t count, bool b_stalled)
{
    uint8_t next = 0u;

    if (b_stalled)
    {
        next = (count < STALL_LIMIT) ? (uint8_t)(count + 1u) : count;
    }

    return next;
}

static void
send_event (uint8_t event, uint8_t seq)
{
    msg_nav_t msg = { .event = event, .small_a = seq };
    uint8_t   buf[MSG_NAV_MAX_BYTES];
    uint8_t   len = msg_nav_pack(&msg, buf);

    rtos_obj_nav_send(buf, len);
}

/*!
 * @brief 5 Hz status snapshot to T1 (73 bits).
 */
static void
send_telemetry (motion_ctx_t * p_ctx)
{
    p_ctx->flags.tlm_div++;

    if (p_ctx->flags.tlm_div >= TLM_DIVIDER)
    {
        uint8_t  buf[MSG_TLM_MAX_BYTES] = { 0u };
        uint16_t pos = 0u;

        p_ctx->flags.tlm_div = 0u;
        bitpack_put(buf, &pos, MSG_TLM_MOTION, MSG_TLM_SRC_BITS);
        bitpack_put(buf, &pos, p_ctx->flags.state, 3u);
        bitpack_put(buf, &pos, p_ctx->flags.mode, 2u);
        bitpack_put(buf, &pos, bitpack_offset(p_ctx->meas_l, SPEED_OFFSET,
                                              10u, &p_ctx->sat_count), 10u);
        bitpack_put(buf, &pos, bitpack_offset(p_ctx->meas_r, SPEED_OFFSET,
                                              10u, &p_ctx->sat_count), 10u);
        bitpack_put(buf, &pos, p_ctx->last_l, 16u);
        bitpack_put(buf, &pos, p_ctx->last_r, 16u);
        bitpack_put(buf, &pos, g_odometer_mm, 16u);
        rtos_obj_tlm_send(MSG_TLM_MOTION, buf,
                          (uint8_t)bitpack_bytes(pos));
    }
}

static int16_t
approach (int16_t current, int16_t target, int16_t step)
{
    int16_t next = target;

    if (current < (target - step))
    {
        next = (int16_t)(current + step);
    }
    else if (current > (target + step))
    {
        next = (int16_t)(current - step);
    }
    else
    {
        next = target;
    }

    return next;
}

static int16_t
abs16 (int16_t value)
{
    return (value < 0) ? (int16_t)(-value) : value;
}

/*** end of file ***/
