/** @file nav.c
 *
 * @brief T6 Navigation Coordinator (system statechart, Section 5).
 *
 * INIT -> CALIBRATE -> READY -> ACTIVE { LINE_FOLLOW, BARCODE_ACTION,
 * LINE_SEARCH, OBSTACLE } -> STOPPED (commanded, restartable) or
 * SAFE_STOP (any critical fault or ESTOP, latched until reset).
 *
 * Manoeuvres are lists of motion steps (TURN, MOVE, REVERSE) executed in
 * order: each step is sent to T2 with a 4-bit sequence number and the next
 * one starts on the matching MOVE_DONE. A step marked "search" ends early
 * when T3 reports LINE_FOUND.
 *
 * Barcode policy: U-TURN executes at once; LEFT/RIGHT are held until the
 * next junction (or line end), then the robot drives 60 mm to put the
 * axle on the junction and turns 90 degrees; if no junction appears within
 * 400 mm the turn is made there. STRAIGHT is logged and the line followed.
 *
 * CALIBRATE first waits for the boot settle (MQTT connected or
 * APP_BOOT_SETTLE_MS) and arms the F12 check, then judges the IR
 * contrast and both wheel encoders during a +/-25 degree spin. The IMU
 * and ultrasonic self-tests are added only for sensors marked fitted in
 * app_cfg.h (APP_IMU_FITTED, APP_ULTRASONIC_FITTED). In single-edge mode
 * it then pivots until the left sensor sits on the line's left edge.
 *
 * Speed: in LINE_FOLLOW the cruise speed is re-sent every 100 ms, slewed
 * towards base_speed (MQTT-settable while driving, with kp/ki/kd), less
 * near an obstacle (HC-SR04 forward range) or on a hump (IMU). After a
 * bypass the car pivots onto the line edge before following again.
 *
 * Barcode test drive (MQTT "start" with val = N mm > 0, e.g. dashboard
 * "drive N"): from READY or STOPPED, one straight MOVE_DIST of N mm at
 * base_speed with the line ignored, shown as BARCODE_ACTION. T3 keeps
 * scanning; BARCODE events are published but not acted on. Forward
 * ranging stays armed and an obstacle stops the car. At the end (or on
 * STOP) the car goes to STOPPED and MISSION_END is published.
 *
 * Corners: a junction with no barcode pending arms a corner check; if the
 * line is lost within APP_CORNER_MM after it, the search sweeps
 * APP_CORNER_SEARCH_DEG each way (right first). Any other loss while
 * following is a bend: the search turns APP_LOST_SEARCH_DEG towards the
 * side the line went (LINE_LOST dir), then 30 degrees past the start the
 * other way, then back. Other searches (rejoin) sweep +/-30 degrees.
 *
 * Owner: Integration (all buddies).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "app_cfg.h"
#include "app_types.h"
#include "board.h"
#include "button.h"
#include "../communications/bitpack.h"
#include "msg_fmt.h"
#include "config.h"
#include "fault.h"
#include "rtos_obj.h"
#include "sys_state.h"
#include "../motion_control/motion.h"
#include "../motion_control/drv_encoder.h"
#include "../communications/mqtt_app.h"
#include "../line_barcode/ir_line.h"
#include "../terrain/imu.h"
#include "../obstacles/obstacle.h"
#include "../obstacles/obs_prof.h"
#include "../evidence/evidence.h"
#include "nav.h"

/* Sub-phases inside a system state (4 bits). */
#define PH_NONE             (0u)
#define PH_CAL_TESTS        (1u)
#define PH_CAL_SPIN         (2u)
#define PH_CAL_IR           (3u)
#define PH_OBS_STOPPING     (4u)
#define PH_OBS_COARSE       (5u)
#define PH_OBS_FINE         (6u)
#define PH_STEPS            (7u)
#define PH_STOPPING         (8u)
#define PH_CAL_SETTLE       (9u)
#define PH_CAL_ALIGN        (10u)
#define PH_OBS_ALIGN        (11u)

#define SELF_TEST_TMO_MS    (3000u)
#define TLM_PERIOD_MS       (200u)
#define SEQ_MASK            (0xFu)
#define UTURN_DEG           (180u)
#define TURN_90_DEG         (90u)
#define SEARCH_SWEEP_DEG    (30u)
#define STEP_SPEED          (150u)
#define RETRY_STEPS         (1u)        /* Reverse plan: one step.      */

typedef struct
{
    uint32_t sys          : 4;
    uint32_t phase        : 4;
    uint32_t retries      : 2;
    uint32_t step_idx     : 4;
    uint32_t step_count   : 4;
    uint32_t seq          : 4;
    uint32_t b_pending    : 1;
    uint32_t pending_cmd  : 2;
    uint32_t b_line_seen  : 1;
    uint32_t b_reacquire  : 1;
    uint32_t b_hold_scan  : 1;
    uint32_t b_mission_end: 1;
    uint32_t b_corner     : 1;          /* Junction passed, no barcode.   */
    uint32_t b_test_drive : 1;          /* Straight barcode test run.     */
    uint32_t reserved     : 1;
} nav_flags_t;

APP_STATIC_ASSERT(sizeof(nav_flags_t) == 4u, nav_flags_size);

typedef struct
{
    nav_flags_t     flags;
    obs_prof_step_t steps[OBS_PROF_MAX_STEPS];
    uint32_t        phase_ms;      /* Phase start (timeouts).           */
    uint32_t        timer_ms;      /* F03 / F07 timer start.            */
    uint32_t        last_tlm_ms;
    uint16_t        pending_odo;
    uint16_t        hold_odo;
    uint16_t        speed;         /* base_speed, mm/s.                 */
    uint16_t        cruise_set;    /* Line-follow speed last sent, mm/s.*/
    uint32_t        speed_ms;      /* Last adaptive speed update.       */
    uint16_t        cal_mark_l;    /* Encoder counts at spin-step start. */
    uint16_t        cal_mark_r;
    uint16_t        cal_edges_l;   /* Edges counted over the whole spin. */
    uint16_t        cal_edges_r;
    uint16_t        jct_odo;       /* Odometer at the last junction.    */
} nav_ctx_t;

static nav_ctx_t g_nav;

static void     handle_event(nav_ctx_t * p_ctx, msg_nav_t const * p_msg);
static void     handle_active(nav_ctx_t * p_ctx, msg_nav_t const * p_msg);
static void     handle_obstacle(nav_ctx_t * p_ctx, msg_nav_t const * p_msg);
static void     periodic(nav_ctx_t * p_ctx, uint32_t now_ms);
static void     calibrate_step(nav_ctx_t * p_ctx, msg_nav_t const * p_msg,
                               uint32_t now_ms);
static void     start_cal_spin(nav_ctx_t * p_ctx);
static bool     cal_sensors_done(void);
static bool     cal_sensors_pass(void);
static void     cal_mark(nav_ctx_t * p_ctx);
static void     cal_count(nav_ctx_t * p_ctx);
static uint16_t add_edges(uint16_t total, int16_t delta);
static void     start_mission(nav_ctx_t * p_ctx);
static void     start_test_drive(nav_ctx_t * p_ctx, uint16_t drive_mm);
static void     request_stop(nav_ctx_t * p_ctx);
static bool     is_active(uint8_t sys);
static void     resume_line_follow(nav_ctx_t * p_ctx);
static void     live_set(nav_ctx_t * p_ctx, msg_nav_t const * p_msg);
static uint16_t cruise_target(nav_ctx_t const * p_ctx);
static void     adapt_speed(nav_ctx_t * p_ctx, uint32_t now);
static void     obstacle_align_step(nav_ctx_t * p_ctx, uint32_t now);
static void     begin_line_search(nav_ctx_t * p_ctx);
static void     begin_corner_search(nav_ctx_t * p_ctx);
static void     begin_lost_search(nav_ctx_t * p_ctx, uint8_t dir);
static void     execute_barcode(nav_ctx_t * p_ctx, bool b_at_junction);
static void     on_profile(nav_ctx_t * p_ctx, msg_nav_t const * p_msg);
static void     retry_or_stop(nav_ctx_t * p_ctx);
static void     steps_done(nav_ctx_t * p_ctx);
static void     add_step(nav_ctx_t * p_ctx, uint8_t type, uint16_t param,
                         uint8_t dir, bool b_search);
static void     run_steps(nav_ctx_t * p_ctx, uint8_t state);
static void     send_step(nav_ctx_t * p_ctx);
static void     send_motion(nav_ctx_t * p_ctx, uint8_t type, uint16_t speed,
                            uint16_t param, uint8_t dir);
static void     set_state(nav_ctx_t * p_ctx, uint8_t sys, uint8_t phase);
static void     enter_safe_stop(nav_ctx_t * p_ctx);
static void     publish_event(uint8_t mq_ev, msg_nav_t const * p_msg);
static void     publish_mission_end(nav_ctx_t * p_ctx);
static void     send_state(nav_ctx_t * p_ctx);
static uint32_t now_ms(void);

/*!
 * @brief Called once before tasks run. The GP20 START and GP21 STOP
 *        buttons are configured by button_init(); g_nav is
 *        zero-initialised.
 */
void
nav_init (void)
{
    /* Nothing to do. */
}

/*!
 * @brief T6 body: event-driven with a 100 ms receive timeout.
 */
void
nav_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    g_nav.flags.b_line_seen = 1u;
    set_state(&g_nav, SYS_ST_CALIBRATE, PH_CAL_SETTLE);

    for (;;)
    {
        uint8_t   buf[MSG_NAV_MAX_BYTES];
        msg_nav_t msg;
        INT const len = rtos_obj_nav_recv(buf, (TMO)APP_NAV_RCV_TMO_MS);

        if ((len > 0) && msg_nav_unpack(buf, (uint8_t)len, &msg))
        {
            handle_event(&g_nav, &msg);
        }

        periodic(&g_nav, now_ms());
    }
}

/*!
 * @brief Route one event by system state.
 */
static void
handle_event (nav_ctx_t * p_ctx, msg_nav_t const * p_msg)
{
    evidence_nav_event(p_msg->event, p_msg->small_a, p_msg->small_b,
                       p_msg->value);                   /* INT-11..13 */

    /* Track line presence in every state, for resume decisions. */
    if (MSG_EV_LINE_LOST == p_msg->event)
    {
        p_ctx->flags.b_line_seen = 0u;
    }
    else if (MSG_EV_LINE_FOUND == p_msg->event)
    {
        p_ctx->flags.b_line_seen = 1u;
    }
    else if (MSG_EV_HUMP == p_msg->event)
    {
        publish_event(MSG_MQEV_HUMP, p_msg);
    }
    else
    {
        /* Handled per state below. */
    }

    switch (p_ctx->flags.sys)
    {
        case SYS_ST_CALIBRATE:
            calibrate_step(p_ctx, p_msg, now_ms());
        break;

        case SYS_ST_READY:
        case SYS_ST_STOPPED:
            if ((MSG_EV_CMD_START == p_msg->event) && (0u != p_msg->value))
            {
                start_test_drive(p_ctx, p_msg->value);
            }
            else if (MSG_EV_CMD_START == p_msg->event)
            {
                start_mission(p_ctx);
            }
            else if ((MSG_EV_CMD_SET == p_msg->event) &&
                     (SYS_ST_READY == p_ctx->flags.sys))
            {
                (void)config_set_encoded(p_msg->small_a, p_msg->value);
            }
            else if (MSG_EV_CMD_SET == p_msg->event)
            {
                /* STOPPED: base_speed and PID only; START reloads speed. */
                live_set(p_ctx, p_msg);
            }
            else
            {
                /* Ignored while idle. */
            }
        break;

        case SYS_ST_LINE_FOLLOW:
        case SYS_ST_BARCODE_ACTION:
        case SYS_ST_LINE_SEARCH:
        case SYS_ST_OBSTACLE:
            handle_active(p_ctx, p_msg);
        break;

        default:
            /* INIT and SAFE_STOP ignore events. */
        break;
    }
}

/*!
 * @brief Events inside the composite ACTIVE state.
 */
static void
handle_active (nav_ctx_t * p_ctx, msg_nav_t const * p_msg)
{
    uint8_t const sys = (uint8_t)p_ctx->flags.sys;

    if (MSG_EV_CMD_STOP == p_msg->event)
    {
        evidence_stop_trigger(EV_SRC_STOP_MQTT);        /* INT-10 */
        request_stop(p_ctx);
    }
    else if (MSG_EV_CMD_SET == p_msg->event)
    {
        live_set(p_ctx, p_msg);
    }
    else if (PH_STOPPING == p_ctx->flags.phase)
    {
        if (MSG_EV_STOPPED == p_msg->event)
        {
            p_ctx->flags.b_test_drive = 0u;
            set_state(p_ctx, SYS_ST_STOPPED, PH_NONE);
            publish_mission_end(p_ctx);
        }
    }
    else if (SYS_ST_OBSTACLE == sys)
    {
        handle_obstacle(p_ctx, p_msg);
    }
    else if ((MSG_EV_OBSTACLE == p_msg->event) &&
             (0u != p_ctx->flags.b_test_drive))
    {
        publish_event(MSG_MQEV_OBSTACLE, p_msg);
        evidence_stop_trigger(EV_SRC_OBSTACLE);         /* INT-10 */
        request_stop(p_ctx);                /* Test drive: just stop.    */
    }
    else if ((MSG_EV_OBSTACLE == p_msg->event) && (SYS_ST_LINE_FOLLOW == sys))
    {
        publish_event(MSG_MQEV_OBSTACLE, p_msg);
        p_ctx->flags.retries = 0u;
        evidence_stop_trigger(EV_SRC_OBSTACLE);         /* INT-10, 12 */
        rtos_obj_sys_set(RTOS_SYS_STOP_REQ);
        set_state(p_ctx, SYS_ST_OBSTACLE, PH_OBS_STOPPING);
    }
    else if (MSG_EV_BARCODE == p_msg->event)
    {
        /* Every decoded code is published; only A..D (nocmd = 0) act. */
        publish_event(MSG_MQEV_BARCODE, p_msg);

        if ((SYS_ST_LINE_FOLLOW == sys) && (0u == p_msg->small_d))
        {
            p_ctx->flags.b_pending   = 1u;
            p_ctx->flags.pending_cmd = p_msg->small_a & 0x3u;
            p_ctx->pending_odo       = motion_odometer_mm();

            if (MSG_BC_UTURN == p_msg->small_a)
            {
                execute_barcode(p_ctx, false);
            }
        }
    }
    else if (MSG_EV_JUNCTION == p_msg->event)
    {
        publish_event(MSG_MQEV_JUNCTION, p_msg);

        if ((SYS_ST_LINE_FOLLOW == sys) && (0u != p_ctx->flags.b_pending))
        {
            execute_barcode(p_ctx, true);
        }
        else if (SYS_ST_LINE_FOLLOW == sys)
        {
            /* No barcode: go straight on; a corner shows up as a loss. */
            p_ctx->flags.b_corner = 1u;
            p_ctx->jct_odo        = motion_odometer_mm();
        }
        else
        {
            /* Junctions met while turning or searching are ignored. */
        }
    }
    else if ((MSG_EV_LINE_LOST == p_msg->event) && (SYS_ST_LINE_FOLLOW == sys))
    {
        bool const b_corner = (0u != p_ctx->flags.b_corner) &&
            ((uint16_t)(motion_odometer_mm() - p_ctx->jct_odo) <=
             APP_CORNER_MM);

        p_ctx->flags.b_corner = 0u;

        if (0u != p_ctx->flags.b_pending)
        {
            execute_barcode(p_ctx, true);   /* Line ends at a T junction. */
        }
        else if (b_corner)
        {
            begin_corner_search(p_ctx);     /* Line turned at the corner. */
        }
        else
        {
            begin_lost_search(p_ctx, p_msg->small_a);   /* Bend. */
        }
    }
    else if ((MSG_EV_LINE_FOUND == p_msg->event) && (SYS_ST_LINE_SEARCH == sys))
    {
        resume_line_follow(p_ctx);
    }
    else if (MSG_EV_MOVE_DONE == p_msg->event)
    {
        if ((PH_STEPS == p_ctx->flags.phase) &&
            (p_msg->small_a == p_ctx->flags.seq))
        {
            p_ctx->flags.step_idx++;
            steps_done(p_ctx);
        }
    }
    else if (MSG_EV_REJECT == p_msg->event)
    {
        fault_set(FAULT_F02_MANOEUVRE);     /* A step T2 cannot execute. */
    }
    else
    {
        /* Other events need no action in this state. */
    }
}

/*!
 * @brief OBSTACLE: stop, coarse scan, fine scan, decide, bypass or retry,
 *        then reacquire the line (FR5, FR6, Section 6.3).
 */
static void
handle_obstacle (nav_ctx_t * p_ctx, msg_nav_t const * p_msg)
{
    switch (p_ctx->flags.phase)
    {
        case PH_OBS_STOPPING:
            if (MSG_EV_STOPPED == p_msg->event)
            {
                rtos_obj_scan_send(msg_sr_pack(MSG_SCAN_COARSE, 0u));
                set_state(p_ctx, SYS_ST_OBSTACLE, PH_OBS_COARSE);
            }
        break;

        case PH_OBS_COARSE:
        case PH_OBS_FINE:
            if (MSG_EV_PROFILE == p_msg->event)
            {
                on_profile(p_ctx, p_msg);
            }
            else if (MSG_EV_SCAN_FAULT == p_msg->event)
            {
                retry_or_stop(p_ctx);   /* Failed scan counts as a retry. */
            }
            else
            {
                /* Waiting for the scan. */
            }
        break;

        case PH_STEPS:
            if ((MSG_EV_LINE_FOUND == p_msg->event) &&
                (0u != p_ctx->flags.b_reacquire) &&
                p_ctx->steps[p_ctx->flags.step_idx].b_search)
            {
                /* Heading is the rejoin angle, inside the 45 deg gate. */
                p_ctx->flags.b_reacquire = 0u;
                p_ctx->flags.step_idx++;
                send_step(p_ctx);
            }
            else if ((MSG_EV_MOVE_DONE == p_msg->event) &&
                     (p_msg->small_a == p_ctx->flags.seq))
            {
                if (p_ctx->steps[p_ctx->flags.step_idx].b_search)
                {
                    fault_set(FAULT_F07_REACQUIRE);
                }
                else
                {
                    p_ctx->flags.step_idx++;
                    steps_done(p_ctx);
                }
            }
            else
            {
                /* Other events are ignored during the manoeuvre. */
            }
        break;

        default:
            /* No other obstacle phases. */
        break;
    }
}

/*!
 * @brief Timers, safety checks, START and STOP buttons and telemetry
 *        (every pass). A press is consumed even when a higher branch
 *        wins, so a press in a state that does not use it (CALIBRATE,
 *        SAFE_STOP, START while ACTIVE, STOP while READY) is ignored,
 *        not deferred.
 */
static void
periodic (nav_ctx_t * p_ctx, uint32_t now)
{
    uint8_t const sys     = (uint8_t)p_ctx->flags.sys;
    bool const    b_start = button_take(BUTTON_START);
    bool const    b_stop  = button_take(BUTTON_STOP);

    if ((SYS_ST_SAFE_STOP != sys) &&
        (fault_is_critical() ||
         (0u != (rtos_obj_sys_peek() & (RTOS_SYS_ESTOP | RTOS_SYS_FAULT)))))
    {
        if (SYS_ST_CALIBRATE == sys)
        {
            fault_set(FAULT_F04_SELF_TEST);
        }

        enter_safe_stop(p_ctx);
    }
    else if (SYS_ST_CALIBRATE == sys)
    {
        calibrate_step(p_ctx, NULL, now);
    }
    else if (b_start &&
             ((SYS_ST_READY == sys) || (SYS_ST_STOPPED == sys)))
    {
        start_mission(p_ctx);
    }
    else if (b_stop && is_active(sys) &&
             (PH_STOPPING != p_ctx->flags.phase))
    {
        evidence_stop_trigger(EV_SRC_STOP_BUTTON);      /* INT-10 */
        request_stop(p_ctx);
    }
    else if ((SYS_ST_LINE_SEARCH == sys) &&
             ((now - p_ctx->timer_ms) > APP_LINE_SEARCH_MS))
    {
        fault_set(FAULT_F03_LINE_SEARCH);
    }
    else if ((0u != p_ctx->flags.b_reacquire) &&
             ((now - p_ctx->timer_ms) > APP_REACQUIRE_MS))
    {
        fault_set(FAULT_F07_REACQUIRE);
    }
    else if ((SYS_ST_OBSTACLE == sys) &&
             (PH_OBS_ALIGN == p_ctx->flags.phase))
    {
        obstacle_align_step(p_ctx, now);
    }
    else if ((SYS_ST_LINE_FOLLOW == sys) && (0u != p_ctx->flags.b_pending) &&
             ((uint16_t)(motion_odometer_mm() - p_ctx->pending_odo) >
              APP_BC_JUNCTION_MM))
    {
        execute_barcode(p_ctx, false);   /* No junction appeared. */
    }
    else if ((SYS_ST_LINE_FOLLOW == sys) && (0u != p_ctx->flags.b_hold_scan) &&
             ((uint16_t)(motion_odometer_mm() - p_ctx->hold_odo) >
              APP_CONTINUE_HOLD_MM))
    {
        p_ctx->flags.b_hold_scan = 0u;
        rtos_obj_scan_send(msg_sr_pack(MSG_SCAN_MONITOR, 0u));
    }
    else
    {
        /* Nothing due. */
    }

    adapt_speed(p_ctx, now);

    if ((now - p_ctx->last_tlm_ms) >= TLM_PERIOD_MS)
    {
        p_ctx->last_tlm_ms = now;
        send_state(p_ctx);
    }
}

/*!
 * @brief CALIBRATE (FR10): IR contrast and motor/encoder spin, plus the
 *        IMU and ultrasonic self-tests when those sensors are fitted.
 *
 * The spin (left 25, right 50, left 25 degrees) sweeps the IR pair across
 * the line while T3 records each sensor's min/max, and turns both wheels.
 * Pass needs T3's contrast PASS and at least APP_CAL_MIN_EDGES encoder
 * edges on each wheel. A wheel that did not count raises F01 as well as
 * F04, so the dashboard shows which half failed.
 */
static void
calibrate_step (nav_ctx_t * p_ctx, msg_nav_t const * p_msg, uint32_t now)
{
    bool const b_done = (NULL != p_msg) && (MSG_EV_MOVE_DONE == p_msg->event) &&
                        (p_msg->small_a == p_ctx->flags.seq);

    switch (p_ctx->flags.phase)
    {
        case PH_CAL_SETTLE:
            /* WiFi start-up over (or given up): F12 on, then self-tests. */
            if (mqtt_app_is_connected() || (now >= APP_BOOT_SETTLE_MS))
            {
                rtos_obj_arm_deadlines();
                set_state(p_ctx, SYS_ST_CALIBRATE, PH_CAL_TESTS);
            }
        break;

        case PH_CAL_TESTS:
            if (cal_sensors_done())
            {
                start_cal_spin(p_ctx);
            }
            else if ((now - p_ctx->phase_ms) > SELF_TEST_TMO_MS)
            {
                fault_set(FAULT_F04_SELF_TEST);
            }
            else
            {
                /* Waiting for the fitted sensors' self-tests. */
            }
        break;

        case PH_CAL_SPIN:
            if (b_done)
            {
                cal_count(p_ctx);
                p_ctx->flags.step_idx++;

                if (p_ctx->flags.step_idx < p_ctx->flags.step_count)
                {
                    cal_mark(p_ctx);
                    send_step(p_ctx);
                }
                else
                {
                    ir_line_set_calibration(false);
                    set_state(p_ctx, SYS_ST_CALIBRATE, PH_CAL_IR);
                }
            }
        break;

        case PH_CAL_IR:
            if (IR_CAL_PENDING != ir_line_cal_result())
            {
                bool const b_motors_ok =
                    (p_ctx->cal_edges_l >= APP_CAL_MIN_EDGES) &&
                    (p_ctx->cal_edges_r >= APP_CAL_MIN_EDGES);

                if (!b_motors_ok)
                {
                    fault_set(FAULT_F01_ENCODER);
                }

                if (b_motors_ok && (IR_CAL_PASS == ir_line_cal_result()) &&
                    cal_sensors_pass())
                {
#if (1 == APP_LINE_EDGE_FOLLOW)
                    send_motion(p_ctx, MSG_MC_EDGE_ALIGN, APP_CAL_SPIN_SPEED,
                                0u, 0u);
                    set_state(p_ctx, SYS_ST_CALIBRATE, PH_CAL_ALIGN);
#else
                    set_state(p_ctx, SYS_ST_READY, PH_NONE);
#endif
                }
                else
                {
                    fault_set(FAULT_F04_SELF_TEST);
                }
            }
        break;

        case PH_CAL_ALIGN:
            /* Pivot until the left sensor holds the edge; not a fault if
             * it cannot (line out of reach): START aligns it anyway. */
            if (ir_line_edge_aligned() ||
                ((now - p_ctx->phase_ms) > APP_CAL_ALIGN_TMO_MS))
            {
                send_motion(p_ctx, MSG_MC_STOP, MSG_SPEED_MIN, 0u, 0u);
                set_state(p_ctx, SYS_ST_READY, PH_NONE);
            }
        break;

        default:
            set_state(p_ctx, SYS_ST_CALIBRATE, PH_CAL_SETTLE);
        break;
    }
}

/*!
 * @brief Queue the three calibration turns and send the first one.
 */
static void
start_cal_spin (nav_ctx_t * p_ctx)
{
    ir_line_set_calibration(true);
    p_ctx->cal_edges_l      = 0u;
    p_ctx->cal_edges_r      = 0u;
    p_ctx->flags.step_count = 0u;
    add_step(p_ctx, MSG_MC_TURN_ANGLE, APP_CAL_SPIN_DEG, MSG_DIR_LEFT, false);
    add_step(p_ctx, MSG_MC_TURN_ANGLE, 2u * APP_CAL_SPIN_DEG, MSG_DIR_RIGHT,
             false);
    add_step(p_ctx, MSG_MC_TURN_ANGLE, APP_CAL_SPIN_DEG, MSG_DIR_LEFT, false);
    p_ctx->flags.step_idx = 0u;
    set_state(p_ctx, SYS_ST_CALIBRATE, PH_CAL_SPIN);
    cal_mark(p_ctx);
    send_step(p_ctx);
}

/*!
 * @brief true once every fitted sensor has finished its self-test.
 */
static bool
cal_sensors_done (void)
{
    bool const b_imu = (0 == APP_IMU_FITTED) ||
                       (IMU_TEST_PENDING != imu_self_test());
    bool const b_obs = (0 == APP_ULTRASONIC_FITTED) ||
                       (OBSTACLE_TEST_PENDING != obstacle_self_test());

    return (b_imu && b_obs);
}

/*!
 * @brief true if every fitted sensor passed its self-test.
 */
static bool
cal_sensors_pass (void)
{
    bool const b_imu = (0 == APP_IMU_FITTED) ||
                       (IMU_TEST_PASS == imu_self_test());
    bool const b_obs = (0 == APP_ULTRASONIC_FITTED) ||
                       (OBSTACLE_TEST_PASS == obstacle_self_test());

    return (b_imu && b_obs);
}

/*!
 * @brief Remember both encoder counts at the start of a spin step.
 */
static void
cal_mark (nav_ctx_t * p_ctx)
{
    uint32_t const snap = drv_encoder_snapshot();

    p_ctx->cal_mark_l = drv_encoder_left(snap);
    p_ctx->cal_mark_r = drv_encoder_right(snap);
}

/*!
 * @brief Add each wheel's travel in the finished step, in either
 *        direction, to the spin totals.
 */
static void
cal_count (nav_ctx_t * p_ctx)
{
    uint32_t const snap = drv_encoder_snapshot();

    p_ctx->cal_edges_l = add_edges(p_ctx->cal_edges_l,
        drv_encoder_delta(drv_encoder_left(snap), p_ctx->cal_mark_l));
    p_ctx->cal_edges_r = add_edges(p_ctx->cal_edges_r,
        drv_encoder_delta(drv_encoder_right(snap), p_ctx->cal_mark_r));
}

/*!
 * @brief total + |delta|, saturating at UINT16_MAX.
 */
static uint16_t
add_edges (uint16_t total, int16_t delta)
{
    int32_t const mag = (delta < 0) ? -(int32_t)delta : (int32_t)delta;
    int32_t const sum = (int32_t)total + mag;

    return (uint16_t)((sum > (int32_t)UINT16_MAX) ? (int32_t)UINT16_MAX : sum);
}

static void
start_mission (nav_ctx_t * p_ctx)
{
    p_ctx->flags.b_pending     = 0u;
    p_ctx->flags.b_hold_scan   = 0u;
    p_ctx->flags.b_reacquire   = 0u;
    p_ctx->flags.b_mission_end = 0u;
    p_ctx->flags.b_corner      = 0u;
    p_ctx->flags.b_test_drive  = 0u;
    p_ctx->speed = config_base_speed(config_word_b());
    resume_line_follow(p_ctx);
}

/*!
 * @brief Barcode test drive: straight for drive_mm at base_speed, line
 *        ignored (MOVE_DIST holds both wheels on the encoders).
 */
static void
start_test_drive (nav_ctx_t * p_ctx, uint16_t drive_mm)
{
    p_ctx->flags.b_pending     = 0u;
    p_ctx->flags.b_hold_scan   = 0u;
    p_ctx->flags.b_reacquire   = 0u;
    p_ctx->flags.b_mission_end = 0u;
    p_ctx->flags.b_corner      = 0u;
    p_ctx->flags.b_test_drive  = 1u;
    p_ctx->speed = config_base_speed(config_word_b());
    rtos_obj_scan_send(msg_sr_pack(MSG_SCAN_MONITOR, 0u));
    p_ctx->flags.step_count = 0u;
    add_step(p_ctx, MSG_MC_MOVE_DIST, drive_mm, 0u, false);
    run_steps(p_ctx, SYS_ST_BARCODE_ACTION);
}

/*!
 * @brief Commanded stop (MQTT STOP or the GP21 button): hold the scanner,
 *        ask T2 to ramp down, and wait for STOPPED in PH_STOPPING.
 */
static void
request_stop (nav_ctx_t * p_ctx)
{
    rtos_obj_scan_send(msg_sr_pack(MSG_SCAN_HOLD, 0u));
    rtos_obj_sys_set(RTOS_SYS_STOP_REQ);
    set_state(p_ctx, (uint8_t)p_ctx->flags.sys, PH_STOPPING);
}

/*!
 * @brief true in the composite ACTIVE state (the mission is running).
 */
static bool
is_active (uint8_t sys)
{
    return (SYS_ST_LINE_FOLLOW == sys) || (SYS_ST_BARCODE_ACTION == sys) ||
           (SYS_ST_LINE_SEARCH == sys) || (SYS_ST_OBSTACLE == sys);
}

/*!
 * @brief Back to LINE_FOLLOW and re-arm forward ranging.
 *
 * T5 stops pinging after it reports an obstacle. If that report arrived
 * during a manoeuvre, T6 ignored it, so MONITOR is always re-armed here
 * unless a CONTINUE decision asked T5 to hold past the same obstacle.
 */
static void
resume_line_follow (nav_ctx_t * p_ctx)
{
    if (0u == p_ctx->flags.b_hold_scan)
    {
        rtos_obj_scan_send(msg_sr_pack(MSG_SCAN_MONITOR, 0u));
    }

    p_ctx->flags.b_reacquire = 0u;
    p_ctx->flags.b_line_seen = 1u;
    p_ctx->cruise_set        = cruise_target(p_ctx);
    p_ctx->speed_ms          = now_ms();
    set_state(p_ctx, SYS_ST_LINE_FOLLOW, PH_NONE);
    send_motion(p_ctx, MSG_MC_LINE_FOLLOW, p_ctx->cruise_set, 0u, 0u);
}

/*!
 * @brief MQTT "set" while the mission runs or in STOPPED: base_speed and
 *        the speed PID gains only (T1 rejects the other keys outside READY).
 *
 * T2 copies config word A once per 10 ms cycle, so a new gain takes
 * effect on the next cycle; a new base_speed is reached through the
 * 100 ms speed slew in adapt_speed().
 */
static void
live_set (nav_ctx_t * p_ctx, msg_nav_t const * p_msg)
{
    uint8_t const key = p_msg->small_a;

    if ((CONFIG_KEY_BASE_SPEED == key) || (CONFIG_KEY_KP == key) ||
        (CONFIG_KEY_KI == key) || (CONFIG_KEY_KD == key))
    {
        if (config_set_encoded(key, p_msg->value) &&
            (CONFIG_KEY_BASE_SPEED == key))
        {
            p_ctx->speed = config_base_speed(config_word_b());
        }
    }
}

/*!
 * @brief Line-follow speed wanted now: base_speed, slowed linearly from
 *        APP_SLOW_ZONE_MM to APP_APPROACH_SPEED at the obstacle threshold,
 *        and capped at APP_HUMP_SPEED on a hump or slope.
 */
static uint16_t
cruise_target (nav_ctx_t const * p_ctx)
{
    uint16_t const base   = p_ctx->speed;
    uint16_t       target = base;

    if ((0 != APP_ULTRASONIC_FITTED) && (base > APP_APPROACH_SPEED))
    {
        uint16_t const range = obstacle_forward_mm();
        uint16_t const limit = config_obstacle_mm(config_word_b());

        if (range <= limit)
        {
            target = APP_APPROACH_SPEED;
        }
        else if (range < APP_SLOW_ZONE_MM)
        {
            target = (uint16_t)(APP_APPROACH_SPEED +
                                (((uint32_t)(base - APP_APPROACH_SPEED) *
                                  (uint32_t)(range - limit)) /
                                 (uint32_t)(APP_SLOW_ZONE_MM - limit)));
        }
        else
        {
            /* Clear ahead (MSG_RANGE_NONE is above the slow zone). */
        }
    }

    if ((0 != APP_IMU_FITTED) && imu_on_hump() && (target > APP_HUMP_SPEED))
    {
        target = APP_HUMP_SPEED;
    }

    return target;
}

/*!
 * @brief Every APP_SPEED_ADJ_MS in LINE_FOLLOW, step the cruise speed by at
 *        most APP_SPEED_STEP_MMS towards cruise_target() and re-send it.
 *
 * T2 keeps its line PD and speed PID state across a re-sent LINE_FOLLOW
 * and ramps its own cruise to the new value.
 */
static void
adapt_speed (nav_ctx_t * p_ctx, uint32_t now)
{
    if ((SYS_ST_LINE_FOLLOW == p_ctx->flags.sys) &&
        (PH_NONE == p_ctx->flags.phase) &&
        ((now - p_ctx->speed_ms) >= APP_SPEED_ADJ_MS))
    {
        uint16_t const target = cruise_target(p_ctx);
        uint16_t       next   = target;

        p_ctx->speed_ms = now;

        if (target > (p_ctx->cruise_set + APP_SPEED_STEP_MMS))
        {
            next = (uint16_t)(p_ctx->cruise_set + APP_SPEED_STEP_MMS);
        }
        else if ((target + APP_SPEED_STEP_MMS) < p_ctx->cruise_set)
        {
            next = (uint16_t)(p_ctx->cruise_set - APP_SPEED_STEP_MMS);
        }
        else
        {
            /* Within one step: go straight to the target. */
        }

        if (next != p_ctx->cruise_set)
        {
            p_ctx->cruise_set = next;
            send_motion(p_ctx, MSG_MC_LINE_FOLLOW, next, 0u, 0u);
        }
    }
}

/*!
 * @brief After a bypass: pivot (EDGE_ALIGN) until the left sensor holds the
 *        edge, then follow the line. The reached flag is held for
 *        APP_EDGE_ALIGN_MS, so it is only trusted after that long here (a
 *        crossing during the last turn does not count). On timeout follow
 *        the line if it is seen, otherwise search for it.
 */
static void
obstacle_align_step (nav_ctx_t * p_ctx, uint32_t now)
{
    uint32_t const held = now - p_ctx->phase_ms;

    if ((held >= APP_EDGE_ALIGN_MS) && ir_line_edge_aligned())
    {
        resume_line_follow(p_ctx);
    }
    else if (held > APP_OBS_ALIGN_TMO_MS)
    {
        if (0u != p_ctx->flags.b_line_seen)
        {
            resume_line_follow(p_ctx);
        }
        else
        {
            begin_line_search(p_ctx);
        }
    }
    else
    {
        /* Still pivoting. */
    }
}

/*!
 * @brief Sweep left 30, right 60, left 30 until LINE_FOUND (F03 at 3 s).
 */
static void
begin_line_search (nav_ctx_t * p_ctx)
{
    p_ctx->flags.step_count = 0u;
    add_step(p_ctx, MSG_MC_TURN_ANGLE, SEARCH_SWEEP_DEG, MSG_DIR_LEFT, true);
    add_step(p_ctx, MSG_MC_TURN_ANGLE, 2u * SEARCH_SWEEP_DEG, MSG_DIR_RIGHT,
             true);
    add_step(p_ctx, MSG_MC_TURN_ANGLE, SEARCH_SWEEP_DEG, MSG_DIR_LEFT, true);
    p_ctx->timer_ms = now_ms();
    run_steps(p_ctx, SYS_ST_LINE_SEARCH);
}

/*!
 * @brief Search after losing the line while following it (a bend).
 *
 * Sweeps APP_LOST_SEARCH_DEG towards the side the line went (LINE_LOST
 * dir from T3), then back past the start by SEARCH_SWEEP_DEG to check the
 * other side, then back to the start. Each step ends early on LINE_FOUND.
 *
 * @param[in] dir  MSG_DIR_LEFT or MSG_DIR_RIGHT: side to try first.
 */
static void
begin_lost_search (nav_ctx_t * p_ctx, uint8_t dir)
{
    uint8_t const first = (MSG_DIR_RIGHT == dir) ? MSG_DIR_RIGHT :
                                                    MSG_DIR_LEFT;
    uint8_t const other = (MSG_DIR_RIGHT == first) ? MSG_DIR_LEFT :
                                                      MSG_DIR_RIGHT;

    p_ctx->flags.step_count = 0u;
    add_step(p_ctx, MSG_MC_TURN_ANGLE, APP_LOST_SEARCH_DEG, first, true);
    add_step(p_ctx, MSG_MC_TURN_ANGLE,
             (uint16_t)(APP_LOST_SEARCH_DEG + SEARCH_SWEEP_DEG), other, true);
    add_step(p_ctx, MSG_MC_TURN_ANGLE, SEARCH_SWEEP_DEG, first, true);
    p_ctx->timer_ms = now_ms();
    run_steps(p_ctx, SYS_ST_LINE_SEARCH);
}

/*!
 * @brief Corner search: right APP_CORNER_SEARCH_DEG, left twice that, back.
 *
 * Right first: the left-edge follower already bends round left corners,
 * so a line lost just after a junction has usually turned right. Each
 * step ends early on LINE_FOUND (search step), like begin_line_search().
 */
static void
begin_corner_search (nav_ctx_t * p_ctx)
{
    p_ctx->flags.step_count = 0u;
    add_step(p_ctx, MSG_MC_TURN_ANGLE, APP_CORNER_SEARCH_DEG, MSG_DIR_RIGHT,
             true);
    add_step(p_ctx, MSG_MC_TURN_ANGLE, 2u * APP_CORNER_SEARCH_DEG,
             MSG_DIR_LEFT, true);
    add_step(p_ctx, MSG_MC_TURN_ANGLE, APP_CORNER_SEARCH_DEG, MSG_DIR_RIGHT,
             true);
    p_ctx->timer_ms = now_ms();
    run_steps(p_ctx, SYS_ST_LINE_SEARCH);
}

/*!
 * @brief Carry out the held barcode command (FR3).
 *
 * @param[in] b_at_junction  true to drive the axle onto the junction first.
 */
static void
execute_barcode (nav_ctx_t * p_ctx, bool b_at_junction)
{
    uint8_t const cmd = (uint8_t)p_ctx->flags.pending_cmd;

    p_ctx->flags.b_pending  = 0u;
    p_ctx->flags.step_count = 0u;

    if (MSG_BC_STRAIGHT == cmd)
    {
        if (0u == p_ctx->flags.b_line_seen)
        {
            begin_line_search(p_ctx);
        }
    }
    else
    {
        if (b_at_junction && (MSG_BC_UTURN != cmd))
        {
            add_step(p_ctx, MSG_MC_MOVE_DIST, APP_BC_ALIGN_MM, 0u, false);
        }

        add_step(p_ctx, MSG_MC_TURN_ANGLE,
                 (MSG_BC_UTURN == cmd) ? UTURN_DEG : TURN_90_DEG,
                 (uint8_t)((MSG_BC_LEFT == cmd) ? MSG_DIR_LEFT : MSG_DIR_RIGHT),
                 false);
        run_steps(p_ctx, SYS_ST_BARCODE_ACTION);
    }
}

/*!
 * @brief Coarse profile -> fine scan; fine profile -> decision (FR6).
 */
static void
on_profile (nav_ctx_t * p_ctx, msg_nav_t const * p_msg)
{
    obs_prof_t prof;
    uint16_t   dev = 0u;
    uint8_t    decision = OBS_DEC_CONTINUE;

    obs_prof_from_msg(p_msg, &prof);

    if ((PH_OBS_COARSE == p_ctx->flags.phase) && prof.b_found)
    {
        rtos_obj_scan_send(msg_sr_pack(MSG_SCAN_FINE,
                                       obs_prof_coarse_centre(&prof)));
        set_state(p_ctx, SYS_ST_OBSTACLE, PH_OBS_FINE);
    }
    else
    {
        publish_event(MSG_MQEV_PROFILE, p_msg);
        decision = obs_prof_decide(&prof, (uint8_t)p_ctx->flags.retries,
                                   &dev);
        evidence_nav_event(EV_NAV_DECISION, decision,
                           (uint8_t)p_ctx->flags.retries, dev);  /* INT-12 */
        p_ctx->flags.step_count = obs_prof_plan(decision, dev, prof.d_min_mm,
                                                p_ctx->steps) & 0xFu;

        switch (decision)
        {
            case OBS_DEC_CONTINUE:
                p_ctx->flags.b_hold_scan = 1u;
                p_ctx->hold_odo = motion_odometer_mm();
                resume_line_follow(p_ctx);
            break;

            case OBS_DEC_BYPASS_LEFT:
            case OBS_DEC_BYPASS_RIGHT:
                rtos_obj_scan_send(msg_sr_pack(MSG_SCAN_HOLD, 0u));
                run_steps(p_ctx, SYS_ST_OBSTACLE);
            break;

            case OBS_DEC_REVERSE:
                p_ctx->flags.retries++;
                run_steps(p_ctx, SYS_ST_OBSTACLE);
            break;

            default:
                fault_set(FAULT_F06_NO_BYPASS);
            break;
        }
    }
}

static void
retry_or_stop (nav_ctx_t * p_ctx)
{
    if (p_ctx->flags.retries < APP_OBS_MAX_RETRIES)
    {
        p_ctx->flags.retries++;
        p_ctx->flags.step_count = obs_prof_plan(OBS_DEC_REVERSE, 0u, 0u,
                                                p_ctx->steps) & 0xFu;
        run_steps(p_ctx, SYS_ST_OBSTACLE);
    }
    else
    {
        fault_set(FAULT_F06_NO_BYPASS);
    }
}

/*!
 * @brief Start the next step, or finish the list for the current state.
 */
static void
steps_done (nav_ctx_t * p_ctx)
{
    if (p_ctx->flags.step_idx < p_ctx->flags.step_count)
    {
        if (p_ctx->steps[p_ctx->flags.step_idx].b_search &&
            (SYS_ST_OBSTACLE == p_ctx->flags.sys))
        {
            p_ctx->flags.b_reacquire = 1u;
            p_ctx->timer_ms = now_ms();
        }

        send_step(p_ctx);
    }
    else if (0u != p_ctx->flags.b_test_drive)
    {
        request_stop(p_ctx);        /* Drive done: STOPPED, MISSION_END. */
    }
    else if (SYS_ST_LINE_SEARCH == p_ctx->flags.sys)
    {
        fault_set(FAULT_F03_LINE_SEARCH);
    }
    else if ((SYS_ST_OBSTACLE == p_ctx->flags.sys) &&
             (RETRY_STEPS == p_ctx->flags.step_count))
    {
        rtos_obj_scan_send(msg_sr_pack(MSG_SCAN_COARSE, 0u));
        set_state(p_ctx, SYS_ST_OBSTACLE, PH_OBS_COARSE);
    }
    else if (SYS_ST_OBSTACLE == p_ctx->flags.sys)
    {
        /* Bypass done, heading parallel to the line: realign on its edge. */
        set_state(p_ctx, SYS_ST_OBSTACLE, PH_OBS_ALIGN);
        send_motion(p_ctx, MSG_MC_EDGE_ALIGN, APP_CAL_SPIN_SPEED, 0u, 0u);
    }
    else
    {
        if (0u == p_ctx->flags.b_line_seen)
        {
            begin_line_search(p_ctx);
        }
        else
        {
            resume_line_follow(p_ctx);
        }
    }
}

static void
add_step (nav_ctx_t * p_ctx, uint8_t type, uint16_t param, uint8_t dir,
          bool b_search)
{
    if (p_ctx->flags.step_count < OBS_PROF_MAX_STEPS)
    {
        obs_prof_step_t * const p_step = &p_ctx->steps[p_ctx->flags.step_count];

        p_step->type     = type;
        p_step->param    = param;
        p_step->dir      = dir;
        p_step->b_search = b_search;
        p_ctx->flags.step_count++;
    }
}

static void
run_steps (nav_ctx_t * p_ctx, uint8_t state)
{
    p_ctx->flags.step_idx = 0u;
    set_state(p_ctx, state, PH_STEPS);
    steps_done(p_ctx);
}

static void
send_step (nav_ctx_t * p_ctx)
{
    obs_prof_step_t const * const p_step =
        &p_ctx->steps[p_ctx->flags.step_idx];
    uint16_t       speed = p_step->b_search ? APP_SEARCH_SPEED : STEP_SPEED;

    if (SYS_ST_CALIBRATE == p_ctx->flags.sys)
    {
        speed = APP_CAL_SPIN_SPEED;     /* Slow self-test spin. */
    }
    else if (0u != p_ctx->flags.b_test_drive)
    {
        speed = p_ctx->speed;           /* Barcode test: base_speed. */
    }
    else
    {
        /* Manoeuvre and search speeds as set above. */
    }

    send_motion(p_ctx, p_step->type, speed, p_step->param, p_step->dir);
}

static void
send_motion (nav_ctx_t * p_ctx, uint8_t type, uint16_t speed, uint16_t param,
             uint8_t dir)
{
    p_ctx->flags.seq = (p_ctx->flags.seq + 1u) & SEQ_MASK;
    rtos_obj_motion_send(msg_mc_pack(type, speed, param, dir,
                                     (uint8_t)p_ctx->flags.seq));
}

static void
set_state (nav_ctx_t * p_ctx, uint8_t sys, uint8_t phase)
{
    bool const b_changed = (sys != p_ctx->flags.sys);

    p_ctx->flags.sys   = sys & 0xFu;
    p_ctx->flags.phase = phase & 0xFu;
    p_ctx->phase_ms    = now_ms();
    evidence_state(sys, phase);                         /* INT-09..12 */

    if (b_changed)
    {
        send_state(p_ctx);
    }
}

/*!
 * @brief Latched SAFE STOP: T2 has already zeroed the PWM (NFR9).
 */
static void
enter_safe_stop (nav_ctx_t * p_ctx)
{
    rtos_obj_sys_set(RTOS_SYS_FAULT);
    rtos_obj_scan_send(msg_sr_pack(MSG_SCAN_HOLD, 0u));
    set_state(p_ctx, SYS_ST_SAFE_STOP, PH_NONE);
    publish_mission_end(p_ctx);
}

/*!
 * @brief Forward an event to T1 as an mbf_tlm EVENT (MQTT event data).
 */
static void
publish_event (uint8_t mq_ev, msg_nav_t const * p_msg)
{
    uint8_t  buf[MSG_TLM_MAX_BYTES] = { 0u };
    uint16_t pos = 0u;

    bitpack_put(buf, &pos, MSG_TLM_EVENT, MSG_TLM_SRC_BITS);
    bitpack_put(buf, &pos, mq_ev, 3u);

    switch (mq_ev)
    {
        case MSG_MQEV_BARCODE:
            bitpack_put(buf, &pos, p_msg->small_a, 2u);
            bitpack_put(buf, &pos, p_msg->small_b, 1u);
            bitpack_put(buf, &pos, p_msg->small_c, MSG_BC_SYM_BITS);
            bitpack_put(buf, &pos, p_msg->small_d, 1u);
        break;

        case MSG_MQEV_JUNCTION:
            bitpack_put(buf, &pos, p_msg->value, 16u);
        break;

        case MSG_MQEV_HUMP:
            bitpack_put(buf, &pos, p_msg->value, 7u);
            bitpack_put(buf, &pos, p_msg->small_a, 4u);
        break;

        case MSG_MQEV_OBSTACLE:
            bitpack_put(buf, &pos, p_msg->value, 11u);
        break;

        case MSG_MQEV_PROFILE:
            bitpack_put(buf, &pos, p_msg->small_a, 5u);
            bitpack_put(buf, &pos, p_msg->value, 11u);
            bitpack_put(buf, &pos, p_msg->small_b, 8u);
            bitpack_put(buf, &pos, p_msg->small_c, 8u);
            bitpack_put(buf, &pos, p_msg->small_d, 8u);
        break;

        default:
            /* MISSION_END has its own builder; FAULT is built by T1. */
        break;
    }

    rtos_obj_tlm_send(MSG_TLM_EVENT, buf, (uint8_t)bitpack_bytes(pos));
}

/*!
 * @brief MISSION_END: highest hump peak, hump count, distance, rejects.
 */
static void
publish_mission_end (nav_ctx_t * p_ctx)
{
    if (0u == p_ctx->flags.b_mission_end)
    {
        uint8_t  buf[MSG_TLM_MAX_BYTES] = { 0u };
        uint16_t pos = 0u;

        p_ctx->flags.b_mission_end = 1u;
        bitpack_put(buf, &pos, MSG_TLM_EVENT, MSG_TLM_SRC_BITS);
        bitpack_put(buf, &pos, MSG_MQEV_MISSION_END, 3u);
        bitpack_put(buf, &pos, imu_hump_peak_mm(), 7u);
        bitpack_put(buf, &pos, imu_hump_count(), 4u);
        bitpack_put(buf, &pos, motion_odometer_mm(), 16u);
        bitpack_put(buf, &pos, ir_line_rejects(), 4u);
        rtos_obj_tlm_send(MSG_TLM_EVENT, buf, (uint8_t)bitpack_bytes(pos));
    }
}

static void
send_state (nav_ctx_t * p_ctx)
{
    uint8_t  buf[MSG_TLM_MAX_BYTES] = { 0u };
    uint16_t pos = 0u;

    bitpack_put(buf, &pos, MSG_TLM_NAV, MSG_TLM_SRC_BITS);
    bitpack_put(buf, &pos, p_ctx->flags.sys, 4u);
    rtos_obj_tlm_send(MSG_TLM_NAV, buf, (uint8_t)bitpack_bytes(pos));
}

static uint32_t
now_ms (void)
{
    return board_time_ms();
}

/*** end of file ***/
