/** @file ir_line.c
 *
 * @brief T3 Line + Barcode task (FR1, FR2, FR11).
 *
 * Every 20 ms (Tribunal 1 change from 10 ms): sample IR_L and IR_C,
 * compute the line error, run the Region A line state machine, send the
 * line error to T2, then run Region B: bc_sensor_poll() drains the
 * barcode edge ring into the Code 39 capture (bc_sensor.c, barcode.c).
 *
 * Owner: Buddy 3 (LIM JUN HAO).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "../integration/board.h"
#include "../integration/app_cfg.h"
#include "../integration/app_types.h"
#include "../communications/bitpack.h"
#include "../integration/msg_fmt.h"
#include "../integration/fault.h"
#include "../integration/rtos_obj.h"
#include "../integration/probe.h"
#include "../motion_control/motion.h"
#include "../evidence/evidence.h"
#include "bc_sensor.h"
#include "drv_adc.h"
#include "ir_line.h"

#define PERIOD_US           ((uint32_t)APP_LINE_PERIOD_MS * 1000u)
#define LOST_SAMPLES        (APP_LINE_LOST_MS / APP_LINE_PERIOD_MS)
#define ALIGN_SAMPLES       (APP_EDGE_ALIGN_MS / APP_LINE_PERIOD_MS)
#define NORM_FULL           (1000)
#define NORM_HALF           (500)
#define SEEN_SUM            (300)
#define ERR_FULL_HALF_MM    (63)
#define ADC_MAX             (4095u)
#define TLM_DIVIDER         (10u)       /* 20 ms x 10 = 5 Hz.             */
#define IR_BIT_L            (0x4u)
#define IR_BIT_C            (0x2u)
#define IR_BIT_BC           (0x1u)

/* Task-private context (rule R4). */
typedef struct
{
    uint32_t line_state  : 3;
    uint32_t white_cnt   : 4;
    uint32_t black_cnt   : 2;
    uint32_t jct_cnt     : 2;
    uint32_t seq         : 4;
    uint32_t miss_cnt    : 2;
    uint32_t b_cal_prev  : 1;
} ir_flags_t;

APP_STATIC_ASSERT(sizeof(ir_flags_t) == 4u, ir_flags_size);

typedef struct
{
    ir_flags_t flags;
    uint16_t   min_l;
    uint16_t   max_l;
    uint16_t   min_c;
    uint16_t   max_c;
    int16_t    err;
    uint8_t    ir_bits;
    uint8_t    tlm_div;
    uint8_t    align_cnt;              /* Edge-reached hold, samples.    */
    int16_t    prev_off;               /* Last left-sensor offset.       */
    uint8_t    b_jct_dark;             /* Both sensors junction-dark.    */
} ir_ctx_t;

static ir_ctx_t g_ir;

/* Calibration handshake: request written by T6, result by T3. */
static volatile bool    gb_cal_request = false;
static volatile uint8_t g_cal_result   = IR_CAL_PENDING;

/* Left sensor held on the edge (single-edge mode): single writer T3. */
static volatile bool gb_edge_aligned = false;

/* Contrast (max - min) per sensor from the last sweep: single writer T3. */
static volatile uint16_t g_cal_contrast[2] = { 0u, 0u };

static void     run_cycle(ir_ctx_t * p_ctx);
static uint16_t read_adc(uint8_t channel);
static int16_t  normalise(uint16_t raw, uint16_t low, uint16_t high);
static void     sample_line(ir_ctx_t * p_ctx, bool * p_black_l,
                            bool * p_black_c, bool * p_seen);
static void     update_calibration(ir_ctx_t * p_ctx);
static void     line_state_machine(ir_ctx_t * p_ctx, bool b_black_l,
                                   bool b_black_c, bool b_seen);
static void     send_simple(uint8_t event, uint16_t value, uint8_t small_a,
                            uint8_t small_b);
static void     send_telemetry(ir_ctx_t * p_ctx);

/*!
 * @brief Configure the ADC inputs (the barcode sampler: bc_sensor_init()).
 */
void
ir_line_init (void)
{
    drv_adc_init();

    /* g_ir is zero-initialised as a static; only the minima need setting. */
    g_ir.min_l = (uint16_t)ADC_MAX;
    g_ir.min_c = (uint16_t)ADC_MAX;
}

/*!
 * @brief T3 body, released every 20 ms (phase 5 ms) by cyc_line.
 */
void
ir_line_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        uint32_t release = 0u;
        uint32_t start   = 0u;
        uint32_t end     = 0u;

        (void)tk_slp_tsk(TMO_FEVR);
        probe_high();
        start   = board_time_us();
        release = rtos_obj_release_us(RTOS_REL_LINE);

        run_cycle(&g_ir);
        end = board_time_us();

        if (rtos_obj_missed_deadline(release, start, end, PERIOD_US))
        {
            g_ir.flags.miss_cnt++;

            if (g_ir.flags.miss_cnt >= APP_OVERRUN_LIMIT)
            {
                fault_set(FAULT_F12_DEADLINE);
            }
        }
        else
        {
            g_ir.flags.miss_cnt = 0u;
        }

        evidence_cycle(EV_LOOP_T3, release, start, end);    /* INT-09 */
        probe_low();
    }
}

/*!
 * @brief Start (true) or finish (false) the CALIBRATE min/max sweep.
 *        Called only by T6.
 */
void
ir_line_set_calibration (bool b_active)
{
    if (b_active)
    {
        g_cal_result = IR_CAL_PENDING;
    }

    gb_cal_request = b_active;
}

uint8_t
ir_line_cal_result (void)
{
    return g_cal_result;
}

/*!
 * @brief Contrast (max - min, ADC counts) of one sensor in the last
 *        calibration sweep; 0 before the first one.
 */
/*!
 * @brief true for APP_EDGE_ALIGN_MS after the left sensor reached the edge
 *        (within APP_EDGE_ALIGN_TOL of APP_EDGE_TARGET, or crossed it
 *        between two samples), single-edge mode.
 */
bool
ir_line_edge_aligned (void)
{
    return gb_edge_aligned;
}

uint16_t
ir_line_cal_contrast (uint8_t sensor)
{
    return (sensor <= IR_SENSOR_C) ? g_cal_contrast[sensor] : 0u;
}

/*!
 * @brief Rejected barcode captures (0..15), for MISSION_END.
 */
uint8_t
ir_line_rejects (void)
{
    return bc_sensor_rejects();
}

static void
run_cycle (ir_ctx_t * p_ctx)
{
    bool b_black_l = false;
    bool b_black_c = false;
    bool b_seen    = false;

    sample_line(p_ctx, &b_black_l, &b_black_c, &b_seen);
    update_calibration(p_ctx);

    if (IR_LINE_ST_CALIBRATE != p_ctx->flags.line_state)
    {
#if (1 == APP_LINE_EDGE_FOLLOW)
        /* An all-white left sensor still means "edge to the right", so
         * the error stays valid for steering; T6 decides the line is lost
         * (LINE_LOST after APP_LINE_LOST_MS without it). */
        bool const b_valid = true;
#else
        bool const b_valid = b_seen;
#endif

        line_state_machine(p_ctx, b_black_l, b_black_c, b_seen);
        rtos_obj_line_send(msg_le_pack(p_ctx->err, p_ctx->ir_bits, b_valid,
                                       (IR_LINE_ST_JUNCTION ==
                                        p_ctx->flags.line_state),
                                       (uint8_t)p_ctx->flags.seq));
        p_ctx->flags.seq++;
    }

    bc_sensor_poll();
    send_telemetry(p_ctx);
}

static uint16_t
read_adc (uint8_t channel)
{
    uint32_t sum = 0u;

    for (uint8_t idx = 0u; idx < APP_IR_OVERSAMPLE; idx++)
    {
        uint16_t sample = 0u;

        /* A failed conversion reads as 0; the next sample corrects it. */
        (void)drv_adc_read(channel, &sample);
        sum += sample;
    }

    return (uint16_t)(sum / APP_IR_OVERSAMPLE);
}

/*!
 * @brief Map a raw reading to 0 (white) .. 1000 (black).
 */
static int16_t
normalise (uint16_t raw, uint16_t low, uint16_t high)
{
    int32_t value = NORM_HALF;

    if (high > low)
    {
        value = (((int32_t)raw - (int32_t)low) * NORM_FULL) /
                ((int32_t)high - (int32_t)low);
        value = (value < 0) ? 0 : ((value > NORM_FULL) ? NORM_FULL : value);
    }

#if (0 == APP_IR_BLACK_IS_HIGH)
    value = NORM_FULL - value;
#endif

    return (int16_t)value;
}

/*!
 * @brief Read both line sensors; derive error, colours and line presence.
 *
 * Single-edge mode (APP_LINE_EDGE_FOLLOW 1): the left sensor tracks the
 * line's right edge (APP_EDGE_RIGHT 1) or left edge (0), so the error is
 * how far it is from half black; positive means the line is to the right.
 * The centre sensor sits right of the left one; only when both read above
 * APP_JCT_BLACK is the car over a junction or corner, and the error is
 * then held at 0 so the cross bar does not steer it.
 *
 * Straddle mode (0): the error is proportional to the difference in
 * blackness of the pair: positive means the line is to the right.
 */
static void
sample_line (ir_ctx_t * p_ctx, bool * p_black_l, bool * p_black_c,
             bool * p_seen)
{
    uint16_t const raw_l = read_adc(APP_ADC_CH_IR_L);
    uint16_t const raw_c = read_adc(APP_ADC_CH_IR_C);
    int16_t const  n_l   = normalise(raw_l, p_ctx->min_l, p_ctx->max_l);
    int16_t const  n_c   = normalise(raw_c, p_ctx->min_c, p_ctx->max_c);
    bool const     b_bc  = bc_sensor_is_black();

    if (gb_cal_request)
    {
        p_ctx->min_l = (raw_l < p_ctx->min_l) ? raw_l : p_ctx->min_l;
        p_ctx->max_l = (raw_l > p_ctx->max_l) ? raw_l : p_ctx->max_l;
        p_ctx->min_c = (raw_c < p_ctx->min_c) ? raw_c : p_ctx->min_c;
        p_ctx->max_c = (raw_c > p_ctx->max_c) ? raw_c : p_ctx->max_c;
    }

    *p_black_l = (n_l > NORM_HALF);
    *p_black_c = (n_c > NORM_HALF);

#if (1 == APP_LINE_EDGE_FOLLOW)
    {
        bool const    b_dark = (n_l > APP_JCT_BLACK) &&
                               (n_c > APP_JCT_BLACK);
#if (1 == APP_EDGE_RIGHT)
        /* Right edge: blacker than the target = line further right. The
         * centre sensor black (not a junction) with the left one whitish
         * means the car drifted left past the line: full right. */
        int16_t const off = (*p_black_c && (!b_dark) &&
                             (n_l < APP_EDGE_TARGET)) ?
                            (int16_t)APP_EDGE_TARGET :
                            (int16_t)(n_l - APP_EDGE_TARGET);
#else
        int16_t const off = (int16_t)(APP_EDGE_TARGET - n_l);
#endif
        bool const    b_on_edge = (off <= APP_EDGE_ALIGN_TOL) &&
                                  (off >= -APP_EDGE_ALIGN_TOL);

        p_ctx->b_jct_dark = b_dark ? 1u : 0u;
        *p_seen    = (n_l > APP_EDGE_SEEN) || *p_black_c;
        p_ctx->err = b_dark ? 0 :
                     (int16_t)(((int32_t)off * ERR_FULL_HALF_MM) /
                               APP_EDGE_TARGET);
        bool const    b_crossed = ((p_ctx->prev_off > 0) && (off <= 0)) ||
                                  ((p_ctx->prev_off < 0) && (off >= 0));

        if ((b_on_edge || b_crossed) && (!b_dark))
        {
            p_ctx->align_cnt = (uint8_t)ALIGN_SAMPLES;
        }
        else if (p_ctx->align_cnt > 0u)
        {
            p_ctx->align_cnt--;
        }
        else
        {
            /* Not at the edge. */
        }

        p_ctx->prev_off = off;
        gb_edge_aligned = (p_ctx->align_cnt > 0u);
    }
#else
    p_ctx->b_jct_dark = (*p_black_l && *p_black_c) ? 1u : 0u;
    *p_seen    = (*p_black_l || *p_black_c || ((n_l + n_c) > SEEN_SUM));
    p_ctx->err = (int16_t)(((int32_t)(n_c - n_l) * ERR_FULL_HALF_MM) /
                           NORM_FULL);
#endif
    p_ctx->ir_bits = (uint8_t)((*p_black_l ? IR_BIT_L : 0u) |
                               (*p_black_c ? IR_BIT_C : 0u) |
                               (b_bc ? IR_BIT_BC : 0u));
}

/*!
 * @brief Enter CALIBRATE on request; on exit judge the contrast (F04).
 */
static void
update_calibration (ir_ctx_t * p_ctx)
{
    bool const b_req = gb_cal_request;

    if (b_req && (0u == p_ctx->flags.b_cal_prev))
    {
        p_ctx->min_l = (uint16_t)ADC_MAX;
        p_ctx->min_c = (uint16_t)ADC_MAX;
        p_ctx->max_l = 0u;
        p_ctx->max_c = 0u;
        p_ctx->flags.line_state = IR_LINE_ST_CALIBRATE;
        bc_sensor_set_calibration(true);
    }
    else if ((!b_req) && (0u != p_ctx->flags.b_cal_prev))
    {
        uint16_t const con_l = (p_ctx->max_l > p_ctx->min_l) ?
            (uint16_t)(p_ctx->max_l - p_ctx->min_l) : 0u;
        uint16_t const con_c = (p_ctx->max_c > p_ctx->min_c) ?
            (uint16_t)(p_ctx->max_c - p_ctx->min_c) : 0u;
        bool const     b_ok  = (con_l >= APP_IR_MIN_CONTRAST) &&
                               (con_c >= APP_IR_MIN_CONTRAST);

        g_cal_contrast[IR_SENSOR_L] = con_l;
        g_cal_contrast[IR_SENSOR_C] = con_c;
        g_cal_result = b_ok ? IR_CAL_PASS : IR_CAL_FAIL;
        (void)tm_printf((UB *)"ir cal: L %d..%d (%d) C %d..%d (%d) need %d"
                        " -> %s\n", (INT)p_ctx->min_l, (INT)p_ctx->max_l,
                        (INT)con_l, (INT)p_ctx->min_c, (INT)p_ctx->max_c,
                        (INT)con_c, (INT)APP_IR_MIN_CONTRAST,
                        b_ok ? "PASS" : "FAIL");
        bc_sensor_set_calibration(false);
        p_ctx->flags.line_state = IR_LINE_ST_TRACKING;
        p_ctx->flags.white_cnt  = 0u;
    }
    else
    {
        /* No change. */
    }

    p_ctx->flags.b_cal_prev = b_req ? 1u : 0u;
}

/*!
 * @brief Region A: LINE_LOST after APP_LINE_LOST_MS without the line,
 *        LINE_FOUND after 3 black samples, JUNCTION after 3 samples with
 *        both sensors junction-dark (b_jct_dark, set in sample_line()).
 */
static void
line_state_machine (ir_ctx_t * p_ctx, bool b_black_l, bool b_black_c,
                    bool b_seen)
{
    bool const b_both = (0u != p_ctx->b_jct_dark);

    p_ctx->flags.jct_cnt = b_both ?
        ((p_ctx->flags.jct_cnt < APP_JUNCTION_SAMPLES) ?
         (p_ctx->flags.jct_cnt + 1u) : p_ctx->flags.jct_cnt) : 0u;

    switch (p_ctx->flags.line_state)
    {
        case IR_LINE_ST_TRACKING:
        case IR_LINE_ST_LOSING:
            if (APP_JUNCTION_SAMPLES == p_ctx->flags.jct_cnt)
            {
                p_ctx->flags.line_state = IR_LINE_ST_JUNCTION;
                send_simple(MSG_EV_JUNCTION, motion_odometer_mm(), 0u, 0u);
            }
            else if (b_seen)
            {
                p_ctx->flags.line_state = IR_LINE_ST_TRACKING;
                p_ctx->flags.white_cnt  = 0u;
            }
            else if ((p_ctx->flags.white_cnt + 1u) >= LOST_SAMPLES)
            {
                p_ctx->flags.line_state = IR_LINE_ST_LOST;
                p_ctx->flags.black_cnt  = 0u;
                send_simple(MSG_EV_LINE_LOST, 0u, 0u, 0u);
            }
            else
            {
                p_ctx->flags.line_state = IR_LINE_ST_LOSING;
                p_ctx->flags.white_cnt++;
            }
        break;

        case IR_LINE_ST_JUNCTION:
            if (!b_both)
            {
                p_ctx->flags.line_state = IR_LINE_ST_TRACKING;
            }
        break;

        case IR_LINE_ST_LOST:
            p_ctx->flags.black_cnt = (b_black_l || b_black_c) ?
                ((p_ctx->flags.black_cnt < APP_LINE_FOUND_SAMPLES) ?
                 (p_ctx->flags.black_cnt + 1u) : p_ctx->flags.black_cnt) : 0u;

            if (APP_LINE_FOUND_SAMPLES == p_ctx->flags.black_cnt)
            {
                p_ctx->flags.line_state = IR_LINE_ST_TRACKING;
                p_ctx->flags.white_cnt  = 0u;
                send_simple(MSG_EV_LINE_FOUND, 0u, 0u, 0u);
            }
        break;

        default:
            p_ctx->flags.line_state = IR_LINE_ST_TRACKING;
        break;
    }
}

static void
send_simple (uint8_t event, uint16_t value, uint8_t small_a, uint8_t small_b)
{
    msg_nav_t msg = { .event = event, .value = value, .small_a = small_a,
                      .small_b = small_b };
    uint8_t   buf[MSG_NAV_MAX_BYTES];
    uint8_t   len = msg_nav_pack(&msg, buf);

    rtos_obj_nav_send(buf, len);
}

/*!
 * @brief 5 Hz snapshot to T1 (23 bits).
 */
static void
send_telemetry (ir_ctx_t * p_ctx)
{
    p_ctx->tlm_div++;

    if (p_ctx->tlm_div >= TLM_DIVIDER)
    {
        uint8_t            buf[MSG_TLM_MAX_BYTES] = { 0u };
        uint16_t           pos = 0u;
        bc_sensor_report_t bc  = { 0u, 0u, 0u, 0u, 0u };

        bc_sensor_report(&bc);

        p_ctx->tlm_div = 0u;
        bitpack_put(buf, &pos, MSG_TLM_LINE, MSG_TLM_SRC_BITS);
        bitpack_put(buf, &pos, p_ctx->flags.line_state, 3u);
        bitpack_put(buf, &pos, bc.state, 2u);
        bitpack_put(buf, &pos, p_ctx->ir_bits, 3u);
        bitpack_put(buf, &pos, bitpack_offset(p_ctx->err, MSG_LE_ERR_OFFSET,
                                              7u, NULL), 7u);
        bitpack_put(buf, &pos, bc.last_cmd, 2u);
        bitpack_put(buf, &pos, bc.last_dir, 1u);
        bitpack_put(buf, &pos, bc.b_new, 1u);
        bitpack_put(buf, &pos, bc.rejects, 4u);
        rtos_obj_tlm_send(MSG_TLM_LINE, buf, (uint8_t)bitpack_bytes(pos));
    }
}

/*** end of file ***/
