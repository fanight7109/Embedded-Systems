/** @file bc_sensor.c
 *
 * @brief Barcode sensor sampler ISR, edge ring and T3 capture driver
 *        (FR2, FR3, NFR8, NFR10).
 *
 * Sampler: TIMER alarm 0 fires every APP_BC_SAMPLE_US. The ISR averages
 * APP_BC_OVERSAMPLE conversions of ADC2 (GP28, sensor AO) and passes the
 * "blackness" to the slicer (barcode_slice(), barcode.c), which compares
 * it with thresholds that ride on the paper: bar at white + APP_BC_RISE,
 * space at white + APP_BC_FALL. The printed bars read only ~350 against
 * ~160 for paper (29 Oct), far lighter than the tape line, so the old
 * fixed and CALIBRATE-derived thresholds (set by the line) missed them.
 * CALIBRATE now only records the range the sensor saw, for DIAG.
 *
 * Trace: each sample's blackness is also averaged into distance bins of
 * APP_BC_TRACE_BIN_COUNTS encoder edges; every finished bin goes into the
 * ISR's ring g_bc_bins (blackness:12 | black:1). T3 copies the ring, oldest
 * first and with interrupts masked, into g_bc_trace_out when a capture has
 * ended and the car is APP_BC_TRACE_TAIL_MM past it (or after
 * APP_BC_TRACE_TAIL_MS), or when T1 passes on "diag bc_trace". Handshake:
 * T3 alone writes the copy and g_bc_trace_seq, and only while T1 has
 * released the previous copy (g_bc_trace_ack == g_bc_trace_seq); T1 alone
 * writes g_bc_trace_ack and g_bc_trace_req.
 *
 * Ring (NFR12): g_bc_ring[APP_BC_RING_SIZE] of uint32_t entries,
 *   ticks:15 (16 us units, saturating) | counts:16 (encoder edges L+R,
 *   saturating) | level:1 (1 = black after the edge).
 * The ISR writes the entry, then publishes the head; it is the only
 * writer of the head, the overflow count, the last-edge stamps, the
 * level, the last sample, the slicer and the published thresholds. T3 is
 * the only writer of the tail. Each entry is one aligned 32-bit store.
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
#include "../integration/msg_fmt.h"
#include "../integration/rtos_obj.h"
#include "../motion_control/drv_encoder.h"
#include "barcode.h"
#include "drv_adc.h"
#include "bc_sensor.h"

#define RING_MASK           (APP_BC_RING_SIZE - 1u)
#define TICK_US             (16u)
#define TICKS_MAX           (0x7FFFu)       /* 15 bits = 524 ms.          */
#define COUNTS_MAX          (0xFFFFu)
#define TICK_SHIFT          (17u)
#define COUNT_SHIFT         (1u)
#define COUNT_MASK          (0xFFFFu)
#define LEVEL_MASK          (0x1u)
#define SIGN_BIT            (0x8000u)
#define WHEELS              (2)             /* Counts are L + R.          */
#define REJECT_MAX          (15u)
#define DIR_MASK            (0x1u)
#define CMD_MASK            (0x3u)
#define ADC_MAX             (4095u)
#define THR_BLACK_SHIFT     (16u)
#define THR_WHITE_MASK      (0xFFFFu)
#define BC_ALARM_BIT        (0x1u)          /* TIMER alarm 0.             */
#define BC_IRQ_TIMER0       (0u)            /* NVIC line of TIMER_IRQ_0.  */
#define BC_INT_LEVEL        (2)             /* Same level as gpio_isr.c.  */
#define BC_ALARM_LEAD_US    (20u)           /* Re-arm at least this ahead.*/
#define TRACE_MASK          (APP_BC_TRACE_BINS - 1u)
#define TRACE_LEVEL_MASK    (0xFFFu)
#define TRACE_BLACK_BIT     (0x1000u)
#define TRACE_MAX_PER_SAMPLE (4u)           /* Bins one sample may close. */
#define UM_PER_COUNT_X2     (APP_WHEEL_CIRC_UM / APP_ENC_EDGES_PER_REV)
#define PROF_QUIET_BINS     ((APP_BC_PROF_QUIET_MM * 1000u * 2u) / \
                             (APP_BC_TRACE_BIN_COUNTS * UM_PER_COUNT_X2))

APP_STATIC_ASSERT(0u == (APP_BC_RING_SIZE & RING_MASK), bc_ring_pow2);
APP_STATIC_ASSERT(APP_BC_RING_SIZE <= 128u, bc_ring_fits_uint8);
APP_STATIC_ASSERT(APP_BC_FALL < APP_BC_RISE, bc_hysteresis);
APP_STATIC_ASSERT((APP_BC_OVERSAMPLE >= 1u) && (APP_BC_OVERSAMPLE <= 16u),
                  bc_oversample);
APP_STATIC_ASSERT(APP_BC_SAMPLE_US > (2u * BC_ALARM_LEAD_US), bc_sample);
APP_STATIC_ASSERT(0u == (APP_BC_TRACE_BINS & TRACE_MASK), bc_trace_pow2);
APP_STATIC_ASSERT(APP_BC_TRACE_BIN_COUNTS > 0u, bc_trace_bin);
APP_STATIC_ASSERT((PROF_QUIET_BINS > 0u) &&
                  (PROF_QUIET_BINS < APP_BC_TRACE_BINS), bc_prof_quiet);

/* Task-private context, T3 only (rule R4). */
typedef struct
{
    barcode_scan_t scan;
    uint8_t        seen_ovf;
    uint8_t        rejects;
    uint8_t        last_cmd;
    uint8_t        last_dir;
    uint8_t        b_new;
    uint8_t        b_cal;           /* Calibration sweep in progress.    */
    uint16_t       cal_min;         /* Blackness range seen in the sweep.*/
    uint16_t       cal_max;
    uint8_t        b_trace_due;     /* Capture ended: copy after tail.   */
    uint8_t        seen_req;        /* Last g_bc_trace_req acted on.     */
    uint16_t       trace_mark_sum;  /* Encoders when the capture ended.  */
    uint32_t       trace_mark_ms;
    uint16_t       prof_from;       /* Bin head at the last read.        */
    uint16_t       watch_head;      /* Bin head at the last flat check.  */
    uint8_t        b_active;        /* Profile moved since the last read.*/
} bc_ctx_t;

static bc_ctx_t g_bc;

/* ISR-owned ring (single producer, single consumer). */
static volatile uint32_t g_bc_ring[APP_BC_RING_SIZE];
static volatile uint8_t  g_bc_head      = 0u;   /* Written only by ISR.   */
static volatile uint8_t  g_bc_tail      = 0u;   /* Written only by T3.    */
static volatile uint8_t  g_bc_overflows = 0u;   /* Written only by ISR.   */
static volatile uint32_t g_bc_last_us   = 0u;   /* Written only by ISR.   */
static volatile uint16_t g_bc_last_sum  = 0u;   /* Written only by ISR.   */

/* Sampler state: single writer the alarm ISR. */
static volatile uint32_t g_bc_next_us   = 0u;   /* Next alarm time.       */
static volatile uint16_t g_bc_raw       = 0u;   /* Last blackness sample. */
static volatile uint8_t  g_bc_level     = 0u;   /* 1 = over a black bar.  */
static volatile uint16_t g_bc_slice_sum = 0u;   /* Encoders at last sample*/

/* Slicer: set up by bc_sensor_init() before the alarm is enabled, then
 * written only by the sampler ISR. */
static barcode_slicer_t  g_bc_slicer;

/* Slicer thresholds in use, black_on:16 | white_on:16, for DIAG: single
 * writer the ISR, one aligned 32-bit store so a reader never sees a mixed
 * pair. */
static volatile uint32_t g_bc_thresholds = 0u;

/* Range seen in the last CALIBRATE (read by T1): single writer T3. */
static volatile uint16_t g_bc_cal_white = 0u;
static volatile uint16_t g_bc_cal_black = 0u;

/* Reject count (0..15) for MISSION_END (read by T6): single writer T3. */
static volatile uint8_t  g_bc_rejects   = 0u;

/* Trace ring and bin accumulator: single writer the sampler ISR. */
static volatile uint16_t g_bc_bins[APP_BC_TRACE_BINS];
static volatile uint16_t g_bc_bin_head  = 0u;   /* Next slot (wraps).     */
static uint32_t          g_bc_bin_sum   = 0u;
static uint16_t          g_bc_bin_n     = 0u;
static uint16_t          g_bc_bin_travel = 0u;

/* Profile decode buffer and its start point: T3 only. */
static uint16_t          g_bc_profile[APP_BC_TRACE_BINS];

/* Trace copy for T1: written by T3 only while T1 has released it. */
static uint16_t          g_bc_trace_out[APP_BC_TRACE_BINS];
static volatile uint8_t  g_bc_trace_seq = 0u;   /* Written only by T3.    */
static volatile uint8_t  g_bc_trace_ack = 0u;   /* Written only by T1.    */
static volatile uint8_t  g_bc_trace_req = 0u;   /* Written only by T1.    */

static void     handle_outcome(bc_ctx_t * p_ctx, uint8_t out,
                               barcode_result_t const * p_result);
static void     count_reject(bc_ctx_t * p_ctx);
static void     send_barcode(uint8_t cmd, uint8_t dir);
static uint16_t encoder_sum(void);
static uint16_t abs_diff(uint16_t now, uint16_t before);
static uint16_t mm_to_counts(uint32_t distance_mm);
static void     set_thresholds(uint16_t black_on, uint16_t white_on);
static uint16_t blackness(uint16_t raw);
static void     push_edge(uint32_t now, uint32_t level);
static void     bin_sample(uint16_t raw, uint16_t travel, uint8_t level);
static void     update_trace(bc_ctx_t * p_ctx);
static void     mark_trace_due(bc_ctx_t * p_ctx);
static void     copy_trace(void);
static bool     profile_decode(bc_ctx_t * p_ctx, barcode_result_t * p_result);
static void     profile_watch(bc_ctx_t * p_ctx);
static void     bc_sample_isr(UINT intno);

/*!
 * @brief Thresholds, capture and the AO sampler (TIMER alarm 0). Called
 *        once from ss3_line_init(), after ir_line_init() has set up the
 *        ADC and the GP28 analogue input.
 *
 * @return E_OK or the tk_def_int() error.
 */
ER
bc_sensor_init (void)
{
    barcode_cfg_t const cfg =
    {
        .quiet_counts = mm_to_counts(APP_BC_QUIET_MM),
        .still_counts = mm_to_counts(APP_BC_STILL_MM),
        .quiet_ticks  = ((uint32_t)APP_BC_QUIET_MS * 1000u) / TICK_US,
        .glitch_ticks = (uint16_t)(APP_BC_GLITCH_US / TICK_US)
    };
    barcode_slicer_cfg_t const slicer_cfg =
    {
        .rise         = APP_BC_RISE,
        .fall         = APP_BC_FALL,
        .delta        = APP_BC_DELTA,
        .creep_counts = APP_BC_CREEP_COUNTS
    };
    T_DINT dint = { .intatr = TA_HLNG, .inthdr = NULL };
    ER     err  = E_OK;

    barcode_slicer_init(&g_bc_slicer, &slicer_cfg);
    set_thresholds(g_bc_slicer.black_on, g_bc_slicer.white_on);
    g_bc_last_us   = board_time_us();
    g_bc_last_sum  = encoder_sum();
    g_bc_slice_sum = g_bc_last_sum;
    barcode_scan_init(&g_bc.scan, &cfg);

    /* Cast: the kernel stores handlers as the generic FP type. */
    dint.inthdr = (FP)bc_sample_isr;
    err = tk_def_int(BC_IRQ_TIMER0, &dint);

    if (E_OK == err)
    {
        g_bc_next_us = board_time_us() + APP_BC_SAMPLE_US;
        out_w(TIMER_INTR, BC_ALARM_BIT);        /* Drop a stale match.  */
        set_w(TIMER_INTE, BC_ALARM_BIT);
        out_w(TIMER_ALARM0, g_bc_next_us);
        EnableInt(BC_IRQ_TIMER0, BC_INT_LEVEL);
    }

    return err;
}

/*!
 * @brief Drain the edge ring into the capture and check for the quiet
 *        zone. Called once per T3 cycle (20 ms).
 */
void
bc_sensor_poll (void)
{
    bc_ctx_t * const p_ctx  = &g_bc;
    uint8_t const    ovf    = g_bc_overflows;
    uint8_t const    head   = g_bc_head;
    uint8_t          tail   = g_bc_tail;
    barcode_result_t result = { '\0', false, false, BARCODE_OK, 0u };

    if (0u != p_ctx->b_cal)
    {
        uint16_t const raw = g_bc_raw;

        p_ctx->cal_min = (raw < p_ctx->cal_min) ? raw : p_ctx->cal_min;
        p_ctx->cal_max = (raw > p_ctx->cal_max) ? raw : p_ctx->cal_max;
    }

    if (ovf != p_ctx->seen_ovf)
    {
        /* Edges were dropped: the capture in progress cannot be trusted. */
        p_ctx->seen_ovf = ovf;

        if (BARCODE_ST_CAPTURE == p_ctx->scan.state)
        {
            count_reject(p_ctx);
        }

        barcode_scan_abort(&p_ctx->scan);
    }

    while (tail != head)
    {
        uint32_t const entry  = g_bc_ring[tail];
        /* Casts: the masks/shifts bound each field to 15 and 16 bits. */
        uint16_t const ticks  = (uint16_t)(entry >> TICK_SHIFT);
        uint16_t const counts = (uint16_t)((entry >> COUNT_SHIFT) &
                                           COUNT_MASK);
        uint8_t        out    = BARCODE_OUT_NONE;

        out = barcode_scan_edge(&p_ctx->scan, ticks, counts,
                                (0u != (entry & LEVEL_MASK)), &result);
        handle_outcome(p_ctx, out, &result);
        tail      = (uint8_t)((tail + 1u) & RING_MASK);
        g_bc_tail = tail;
    }

    {
        uint32_t const now      = board_time_us();
        uint32_t const last_us  = g_bc_last_us;
        uint16_t const last_sum = g_bc_last_sum;
        bool const     b_black  = bc_sensor_is_black();

        /* An edge after the drain makes these stamps inconsistent: skip
         * the check; the next cycle picks the edge up. */
        if (g_bc_head == tail)
        {
            uint8_t const out = barcode_scan_idle(&p_ctx->scan,
                                                  (now - last_us) / TICK_US,
                                                  abs_diff(encoder_sum(),
                                                           last_sum),
                                                  b_black, &result);

            /* Edge path: only a decode counts. Its quiet zone comes too
             * early when the slicer misses thin bars (29 Oct 23:59: cut
             * after 12 of 15 bars), so rejects and the trace are left to
             * profile_watch(), which waits for the profile to go flat. */
            if (BARCODE_OUT_DECODED == out)
            {
                handle_outcome(p_ctx, out, &result);
            }
        }
    }

    profile_watch(p_ctx);
    update_trace(p_ctx);
}

/*!
 * @brief Start (true) or finish (false) the barcode sensor's part of the
 *        CALIBRATE sweep. Called only by T3, from ir_line.c on the edges
 *        of the line calibration.
 *
 * Records the range the sensor saw (DIAG bc_white / bc_black) and prints
 * it with the slicer thresholds now in use. The thresholds are NOT taken
 * from this range any more: the spin crosses the tape line (~2500), which
 * is far blacker than the printed bars (~350), so a split of that range
 * sat above every bar. Never fails the calibration: the line sensors
 * alone decide F04.
 */
void
bc_sensor_set_calibration (bool b_active)
{
    bc_ctx_t * const p_ctx = &g_bc;

    if (b_active)
    {
        p_ctx->cal_min = (uint16_t)ADC_MAX;
        p_ctx->cal_max = 0u;
        p_ctx->b_cal   = 1u;
    }
    else if (0u != p_ctx->b_cal)
    {
        uint32_t const thr = g_bc_thresholds;

        p_ctx->b_cal   = 0u;
        g_bc_cal_white = p_ctx->cal_min;
        g_bc_cal_black = p_ctx->cal_max;

        (void)tm_printf((UB *)"bc cal: saw %d..%d; slicer: black above %d,"
                        " white below %d (paper + %d / + %d)\n",
                        (INT)p_ctx->cal_min, (INT)p_ctx->cal_max,
                        (INT)(thr >> THR_BLACK_SHIFT),
                        (INT)(thr & THR_WHITE_MASK), (INT)APP_BC_RISE,
                        (INT)APP_BC_FALL);
    }
    else
    {
        /* Not calibrating: nothing to finish. */
    }
}

/*!
 * @brief Barcode sensor currently over a black bar (last sampler result).
 */
bool
bc_sensor_is_black (void)
{
    return (0u != g_bc_level);
}

/*!
 * @brief Rejected captures so far (0..15); any task may read.
 */
uint8_t
bc_sensor_rejects (void)
{
    return g_bc_rejects;
}

/*!
 * @brief Fill the telemetry snapshot and clear the new-code flag. Called
 *        only by T3, once per telemetry frame.
 */
void
bc_sensor_report (bc_sensor_report_t * p_report)
{
    p_report->state    = g_bc.scan.state;
    p_report->last_cmd = g_bc.last_cmd;
    p_report->last_dir = g_bc.last_dir;
    p_report->b_new    = g_bc.b_new;
    p_report->rejects  = g_bc.rejects;
    g_bc.b_new         = 0u;
}

/*!
 * @brief One diagnostic value for the MQTT DIAG command; any task may
 *        call (each value is one 16- or 32-bit load), except
 *        BC_DIAG_TRACE, which only T1 may pass (it requests a trace).
 *
 * @param[in] item  BC_DIAG_*.
 *
 * @return Blackness in ADC counts (0..4095); 0 for an unknown item, and
 *         0 for the CALIBRATE range before the first sweep.
 */
uint16_t
bc_sensor_diag (uint8_t item)
{
    uint32_t const thr   = g_bc_thresholds;
    uint16_t       value = 0u;

    switch (item)
    {
        case BC_DIAG_RAW:
            value = g_bc_raw;
        break;

        case BC_DIAG_CAL_WHITE:
            value = g_bc_cal_white;
        break;

        case BC_DIAG_CAL_BLACK:
            value = g_bc_cal_black;
        break;

        case BC_DIAG_BLACK_ON:
            /* Cast: the field is 16 bits wide by construction. */
            value = (uint16_t)(thr >> THR_BLACK_SHIFT);
        break;

        case BC_DIAG_WHITE_ON:
            value = (uint16_t)(thr & THR_WHITE_MASK);
        break;

        case BC_DIAG_TRACE:
            /* T1 is the only caller of this item (single writer). */
            g_bc_trace_req = (uint8_t)(g_bc_trace_req + 1u);
            value          = (uint16_t)APP_BC_TRACE_BINS;
        break;

        default:
            /* Unknown item: mqtt_fmt_check_cmd() already rejects it. */
        break;
    }

    return value;
}

/*!
 * @brief The trace copy waiting to be published (T1 only).
 *
 * @param[out] p_trace_id  Trace number (wraps at 256).
 *
 * @return APP_BC_TRACE_BINS samples (blackness:12 | black:1, oldest
 *         first), or NULL when none is waiting. Valid until
 *         bc_sensor_trace_done().
 */
uint16_t const *
bc_sensor_trace_get (uint8_t * p_trace_id)
{
    uint8_t const          seq    = g_bc_trace_seq;
    uint16_t const * const p_data = (seq != g_bc_trace_ack) ?
                                    g_bc_trace_out : NULL;

    *p_trace_id = seq;

    return p_data;
}

/*!
 * @brief T1 has published the trace copy: T3 may overwrite it.
 */
void
bc_sensor_trace_done (void)
{
    g_bc_trace_ack = g_bc_trace_seq;
}

/*!
 * @brief Travel per trace sample, micrometres.
 */
uint16_t
bc_sensor_trace_bin_um (void)
{
    /* Cast: 6 edges x 161 um / 2 wheels, far below 65535. */
    return (uint16_t)((APP_BC_TRACE_BIN_COUNTS * UM_PER_COUNT_X2) / 2u);
}

/*!
 * @brief Copy the trace for T1 when a capture has ended and the car has
 *        driven the tail past it (or waited), or when T1 asked for one.
 *        A copy is skipped while T1 still holds the previous one.
 */
static void
update_trace (bc_ctx_t * p_ctx)
{
    uint8_t const req      = g_bc_trace_req;
    bool const    b_free   = (g_bc_trace_seq == g_bc_trace_ack);
    bool          b_copy   = false;

    if (req != p_ctx->seen_req)
    {
        p_ctx->seen_req = req;
        b_copy          = true;
    }
    else if (0u != p_ctx->b_trace_due)
    {
        b_copy = (abs_diff(encoder_sum(), p_ctx->trace_mark_sum) >=
                  mm_to_counts(APP_BC_TRACE_TAIL_MM)) ||
                 ((board_time_ms() - p_ctx->trace_mark_ms) >=
                  APP_BC_TRACE_TAIL_MS);
    }
    else
    {
        /* Nothing to send. */
    }

    if (b_copy)
    {
        p_ctx->b_trace_due = 0u;

        if (b_free)
        {
            copy_trace();
            g_bc_trace_seq = (uint8_t)(g_bc_trace_seq + 1u);
        }
    }
}

/*!
 * @brief Decode the bins since the last read by their profile (T3 only).
 *
 * The ring is copied oldest first with interrupts masked (as for the
 * trace); bins from before the last read are blanked so the same code is
 * not reported twice.
 */
static bool
profile_decode (bc_ctx_t * p_ctx, barcode_result_t * p_result)
{
    barcode_peaks_cfg_t const cfg =
    {
        .delta        = APP_BC_PROF_DELTA,
        .min_rise     = APP_BC_PROF_MIN_RISE,
        .space_margin = APP_BC_PROF_SPACE_GAP
    };
    uint16_t head   = 0u;
    UINT     intsts = 0u;
    bool     b_ok   = false;

    DI(intsts);
    head = g_bc_bin_head;

    for (uint16_t idx = 0u; idx < APP_BC_TRACE_BINS; idx++)
    {
        /* Cast: masked to the ring size. */
        g_bc_profile[idx] = g_bc_bins[(uint16_t)((head + idx) & TRACE_MASK)];
    }

    EI(intsts);

    {
        /* Cast: wrapping 16-bit bin counts. */
        uint16_t const fresh = (uint16_t)(head - p_ctx->prof_from);

        if (fresh < APP_BC_TRACE_BINS)
        {
            for (uint16_t idx = 0u;
                 idx < (uint16_t)(APP_BC_TRACE_BINS - fresh); idx++)
            {
                g_bc_profile[idx] = 0u;
            }
        }
    }

    b_ok = barcode_decode_peaks(g_bc_profile, APP_BC_TRACE_BINS, &cfg,
                                  p_result);

#if (1 == APP_BC_LOG)
    (void)tm_printf((UB *)"bc: profile %s *%c* (%d bars)\n",
                    b_ok ? "read" : "no code",
                    b_ok ? (INT)p_result->symbol : (INT)'-',
                    (INT)((p_result->elements + 1u) / 2u));
#endif

    return b_ok;
}

/*!
 * @brief End of a pass, judged on the profile itself: once the last
 *        APP_BC_PROF_QUIET_MM of bins are flat (range below
 *        APP_BC_PROF_FLAT: paper or floor, no bar) after something moved,
 *        decode the bins since the last read. A failure with at least
 *        one character's worth of bars counts as a reject. Runs only when
 *        new bins arrived, so a car stopped mid-code just waits.
 */
static void
profile_watch (bc_ctx_t * p_ctx)
{
    uint16_t const head = g_bc_bin_head;

    if (head != p_ctx->watch_head)
    {
        uint16_t low  = (uint16_t)TRACE_LEVEL_MASK;
        uint16_t high = 0u;

        p_ctx->watch_head = head;

        for (uint16_t back = 1u; back <= PROF_QUIET_BINS; back++)
        {
            /* Cast: masked to the ring size. One 16-bit load each. */
            uint16_t const level = (uint16_t)(
                g_bc_bins[(uint16_t)((head - back) & TRACE_MASK)] &
                TRACE_LEVEL_MASK);

            low  = (level < low) ? level : low;
            high = (level > high) ? level : high;
        }

        if ((uint16_t)(high - low) >= APP_BC_PROF_FLAT)
        {
            p_ctx->b_active = 1u;
        }
        else if ((0u != p_ctx->b_active) &&
                 ((uint16_t)(head - p_ctx->prof_from) > PROF_QUIET_BINS))
        {
            barcode_result_t result = { '\0', false, false, BARCODE_OK, 0u };

            p_ctx->b_active = 0u;

            if (profile_decode(p_ctx, &result))
            {
                handle_outcome(p_ctx, BARCODE_OUT_DECODED, &result);
            }
            else if (result.elements >= BARCODE_MIN_ELEMENTS)
            {
                handle_outcome(p_ctx, BARCODE_OUT_REJECTED, &result);
            }
            else
            {
                mark_trace_due(p_ctx);  /* Not a code; still show it.   */
            }

            p_ctx->prof_from = head;    /* These bins are done.         */
        }
        else
        {
            /* Flat and nothing new: waiting. */
        }
    }
}

/*!
 * @brief A capture ended: send its trace once the tail is driven (the
 *        first end since the last copy sets the mark).
 */
static void
mark_trace_due (bc_ctx_t * p_ctx)
{
    if (0u == p_ctx->b_trace_due)
    {
        p_ctx->b_trace_due    = 1u;
        p_ctx->trace_mark_sum = encoder_sum();
        p_ctx->trace_mark_ms  = board_time_ms();
    }
}

/*!
 * @brief Copy the ISR's bin ring, oldest first (T3 only).
 *
 * Interrupts are masked for the copy (~1024 loads, tens of us) so the
 * ring cannot move under it; one sample is delayed at most that long.
 */
static void
copy_trace (void)
{
    UINT intsts = 0u;

    DI(intsts);

    {
        uint16_t const head = g_bc_bin_head;

        for (uint16_t idx = 0u; idx < APP_BC_TRACE_BINS; idx++)
        {
            /* Cast: masked to the ring size. */
            g_bc_trace_out[idx] =
                g_bc_bins[(uint16_t)((head + idx) & TRACE_MASK)];
        }
    }

    EI(intsts);
}

/*!
 * @brief Report a decoded code to T6, or count a reject.
 */
static void
handle_outcome (bc_ctx_t * p_ctx, uint8_t out,
                barcode_result_t const * p_result)
{
    uint8_t cmd = 0u;

    if (BARCODE_OUT_NONE != out)
    {
        mark_trace_due(p_ctx);
    }

    if (BARCODE_OUT_DECODED == out)
    {
        /* A later profile read must not find this code again. */
        p_ctx->prof_from = g_bc_bin_head;
    }

    if (BARCODE_OUT_DECODED == out)
    {
        bool const    b_cmd = barcode_to_command(p_result->symbol, &cmd);
        uint8_t const dir   = p_result->b_reversed ? 1u : 0u;

        if (b_cmd)
        {
            p_ctx->last_cmd = (uint8_t)(cmd & CMD_MASK);
            p_ctx->last_dir = (uint8_t)(dir & DIR_MASK);
            p_ctx->b_new    = 1u;
            send_barcode(p_ctx->last_cmd, p_ctx->last_dir);
        }

#if (1 == APP_BC_LOG)
        (void)tm_printf((UB *)"bc: *%c* %s, %s, by %s (%d el)\n",
                        (INT)p_result->symbol,
                        b_cmd ? "command sent" : "no command",
                        p_result->b_reversed ? "reverse" : "forward",
                        p_result->b_by_distance ? "distance" : "time",
                        (INT)p_result->elements);
#endif
    }
    else if (BARCODE_OUT_REJECTED == out)
    {
        count_reject(p_ctx);

#if (1 == APP_BC_LOG)
        (void)tm_printf((UB *)"bc: reject, reason %d (%d el, by %s)\n",
                        (INT)p_result->reason, (INT)p_result->elements,
                        p_result->b_by_distance ? "distance" : "time");
#endif
    }
    else
    {
        /* Nothing finished this call. */
    }
}

static void
count_reject (bc_ctx_t * p_ctx)
{
    if (p_ctx->rejects < REJECT_MAX)
    {
        p_ctx->rejects++;
        g_bc_rejects = p_ctx->rejects;
    }
}

static void
send_barcode (uint8_t cmd, uint8_t dir)
{
    msg_nav_t msg = { .event = MSG_EV_BARCODE, .value = 0u, .small_a = cmd,
                      .small_b = dir };
    uint8_t   buf[MSG_NAV_MAX_BYTES];
    uint8_t   len = msg_nav_pack(&msg, buf);

    rtos_obj_nav_send(buf, len);
}

/*!
 * @brief Left + right encoder counts, modulo 2^16 (only differences are
 *        used). Safe in the ISR: one aligned load of the counter pair.
 */
static uint16_t
encoder_sum (void)
{
    uint32_t const snap = drv_encoder_snapshot();

    /* Cast: modular 16-bit sum is intended. */
    return (uint16_t)(drv_encoder_left(snap) + drv_encoder_right(snap));
}

/*!
 * @brief |now - before| for wrapping 16-bit counters (travel either way).
 */
static uint16_t
abs_diff (uint16_t now, uint16_t before)
{
    /* Casts: modular 16-bit differences are intended. */
    uint16_t diff = (uint16_t)(now - before);

    if (0u != (diff & SIGN_BIT))
    {
        diff = (uint16_t)(0u - (uint32_t)diff);
    }

    return diff;
}

/*!
 * @brief Distance to encoder edges of both wheels added together.
 */
static uint16_t
mm_to_counts (uint32_t distance_mm)
{
    /* Cast: distance_mm is a small app_cfg.h constant (< 1000 mm). */
    int32_t const counts = drv_encoder_mm_to_edges((int32_t)distance_mm) *
                           WHEELS;

    /* Cast: clamped to 0..COUNTS_MAX first. */
    return (uint16_t)((counts < 0) ? 0 :
                      ((counts > (int32_t)COUNTS_MAX) ?
                       (int32_t)COUNTS_MAX : counts));
}

/*!
 * @brief Publish the slicer's threshold pair in one 32-bit store (sampler
 *        ISR, and bc_sensor_init() before the ISR is enabled).
 */
static void
set_thresholds (uint16_t black_on, uint16_t white_on)
{
    g_bc_thresholds = ((uint32_t)black_on << THR_BLACK_SHIFT) |
                      (uint32_t)white_on;
}

/*!
 * @brief Raw ADC reading to blackness (higher = blacker).
 */
static uint16_t
blackness (uint16_t raw)
{
#if (1 == APP_IR_BLACK_IS_HIGH)
    return raw;
#else
    /* Cast: raw is a 12-bit result, so the difference is 0..4095. */
    return (uint16_t)(ADC_MAX - raw);
#endif
}

/*!
 * @brief TIMER alarm 0: sample the barcode AO, slice it, and stamp an
 *        edge on each change of colour.
 *
 * WARNING: interrupt context, every APP_BC_SAMPLE_US. APP_BC_OVERSAMPLE
 * ADC conversions (about 2 us each); never waits on the kernel; no
 * floating point (NFR10).
 */
static void
bc_sample_isr (UINT intno)
{
    uint32_t const now   = board_time_us();
    uint32_t       next  = g_bc_next_us + APP_BC_SAMPLE_US;
    uint32_t       total = 0u;
    bool           b_ok  = true;

    (void)intno;
    out_w(TIMER_INTR, BC_ALARM_BIT);

    /* The alarm matches one exact count, so a time already passed would
     * not fire for 71 minutes. Cast: wrapping difference read signed. */
    if ((int32_t)(next - now) < (int32_t)BC_ALARM_LEAD_US)
    {
        next = now + APP_BC_SAMPLE_US;
    }

    g_bc_next_us = next;
    out_w(TIMER_ALARM0, next);

    for (uint8_t idx = 0u; (idx < APP_BC_OVERSAMPLE) && b_ok; idx++)
    {
        uint16_t one = 0u;

        b_ok   = drv_adc_read(APP_ADC_CH_IR_BC, &one);
        total += one;
    }

    if (b_ok)
    {
        uint16_t const sum    = encoder_sum();
        /* Cast: the mean of 12-bit results fits 12 bits. */
        uint16_t const raw    = blackness((uint16_t)(total /
                                                     APP_BC_OVERSAMPLE));
        uint16_t const travel = abs_diff(sum, g_bc_slice_sum);
        uint8_t const  level  = barcode_slice(&g_bc_slicer, raw, travel) ?
                                1u : 0u;

        g_bc_slice_sum = sum;
        g_bc_raw       = raw;
        set_thresholds(g_bc_slicer.black_on, g_bc_slicer.white_on);
        bin_sample(raw, travel, level);

        if (level != g_bc_level)
        {
            g_bc_level = level;
            push_edge(now, level);
        }
    }
}

/*!
 * @brief Average samples into distance bins for the trace (ISR only).
 *
 * WARNING: interrupt context; integer only, never waits.
 */
static void
bin_sample (uint16_t raw, uint16_t travel, uint8_t level)
{
    uint32_t const acc    = (uint32_t)g_bc_bin_travel + travel;
    uint8_t        closed = 0u;

    /* A car stopped for 30 s fills the count: keep the mean so far. */
    if (g_bc_bin_n < COUNTS_MAX)
    {
        g_bc_bin_sum += raw;
        g_bc_bin_n    = (uint16_t)(g_bc_bin_n + 1u);
    }

    /* Cast: clamped to COUNTS_MAX first. */
    g_bc_bin_travel = (uint16_t)((acc > COUNTS_MAX) ? COUNTS_MAX : acc);

    while ((g_bc_bin_travel >= APP_BC_TRACE_BIN_COUNTS) &&
           (closed < TRACE_MAX_PER_SAMPLE))
    {
        uint16_t const head = g_bc_bin_head;
        /* Cast: the mean of 12-bit samples fits 12 bits. */
        uint16_t const mean = (uint16_t)(g_bc_bin_sum / g_bc_bin_n);

        g_bc_bins[head & TRACE_MASK] = (uint16_t)(
            (mean & TRACE_LEVEL_MASK) |
            ((0u != level) ? TRACE_BLACK_BIT : 0u));
        g_bc_bin_head   = (uint16_t)(head + 1u);
        g_bc_bin_travel = (uint16_t)(g_bc_bin_travel -
                                     APP_BC_TRACE_BIN_COUNTS);
        closed++;
    }

    if (0u != closed)
    {
        /* A jump (encoder glitch) is not drawn as hundreds of bins. */
        g_bc_bin_travel = (g_bc_bin_travel >= APP_BC_TRACE_BIN_COUNTS) ?
                          0u : g_bc_bin_travel;
        g_bc_bin_sum    = 0u;
        g_bc_bin_n      = 0u;
    }
}

/*!
 * @brief Stamp one edge into the ring (sampler ISR only).
 *
 * WARNING: interrupt context. Writes the entry before publishing the head
 * index; never waits; no floating point (NFR10).
 */
static void
push_edge (uint32_t now, uint32_t level)
{
    uint16_t const sum    = encoder_sum();
    uint32_t       ticks  = (now - g_bc_last_us) / TICK_US;
    uint16_t const counts = abs_diff(sum, g_bc_last_sum);
    uint8_t const  head   = g_bc_head;
    uint8_t const  next   = (uint8_t)((head + 1u) & RING_MASK);

    ticks         = (ticks > TICKS_MAX) ? TICKS_MAX : ticks;
    g_bc_last_us  = now;
    g_bc_last_sum = sum;

    if (next == g_bc_tail)
    {
        g_bc_overflows++;
    }
    else
    {
        g_bc_ring[head] = (ticks << TICK_SHIFT) |
                          ((uint32_t)counts << COUNT_SHIFT) |
                          (level & LEVEL_MASK);
        g_bc_head       = next;
    }
}

/*** end of file ***/
