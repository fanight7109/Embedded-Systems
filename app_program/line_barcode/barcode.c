/** @file barcode.c
 *
 * @brief Code 39 capture state machine and decoder (FR2, FR3, NFR8, A7).
 *
 * Pattern bits are ordered bar, space, bar, ... bar (9 elements), most
 * significant first, 1 = wide. Example: '*' = 010010100, 'A' = 100001001,
 * which matches the supplied "Barcode Sample" sheet (*A* and *Z*, wide to
 * narrow 3:1).
 *
 * Capture (see barcode.h for the measurement scheme):
 *   IDLE    --white->black edge-->  CAPTURE (the quiet zone is dropped)
 *   CAPTURE --newest bar completes *X*-->  HOLD   (reported at once)
 *   CAPTURE --quiet zone-->  IDLE   (final decode, or reject)
 *   HOLD    --quiet zone-->  IDLE
 * A quiet zone is white for cfg.quiet_counts of travel, or, for a capture
 * the car did not drive across, white for cfg.quiet_ticks.
 *
 * The A4 sheet has almost no white before the first bar on one side (the
 * code is printed a few mm from the paper edge), so the capture may begin
 * on the paper edge, its shadow, a darker floor or the line. The decoder
 * therefore looks for a code ending at the newest bar (the last 29, 39 or
 * 49 elements) instead of requiring the capture to start on the code, and
 * the element buffer slides: the oldest bar/space pair is dropped when it
 * is full.
 *
 * Slicer (one call per analogue sample, from the sampler ISR):
 *   blackness is low-passed first (1/4 per sample: 2 ms at 2 kHz, the
 *   same delay on every edge, so widths are unchanged)
 *   white reference W = lowest recent blackness (the paper)
 *   space -> bar    when blackness >= W + cfg.rise and has risen
 *                   cfg.delta above the lowest point of the space
 *   bar   -> space  when blackness <= W + cfg.fall, or has fallen
 *                   cfg.delta below the highest point of the bar
 * The swing test (29 Oct) reads thin bars that only reach ~200 (paper
 * ~150..190, thick bars ~300+) and narrow spaces between thick bars that
 * never get back down to the paper level.
 * Relative thresholds keep working when the print is faint for the IR
 * sensor (laser and inkjet black can read far lighter than the tape
 * line), and they do not depend on CALIBRATE crossing the line.
 *
 * Owner: Buddy 3 (LIM JUN HAO).
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "../integration/app_cfg.h"
#include "../integration/msg_fmt.h"
#include "barcode.h"

#define CHAR_ELEMENTS       (9u)
#define CHAR_STRIDE         (10u)   /* 9 elements + inter-character gap. */
#define CHAR_BARS           (5u)
#define CHAR_SPACES         (4u)
#define MIN_CHARS           (3u)    /* Start, one data character, stop.  */
#define MAX_CHARS           ((APP_BC_MAX_ELEMENTS + 1u) / CHAR_STRIDE)
#define FIRST_DECODE_COUNT  ((MIN_CHARS * CHAR_STRIDE) - 1u)
#define DROP_ELEMENTS       (2u)    /* One bar + one space: keeps order. */
#define NUM_SYMBOLS         (44u)
#define FRAME_SYMBOL        ('*')
#define PATTERN_TOP_BIT     (8u)
#define WIDE_BARS           (2u)    /* All symbols except $ / + %.       */
#define WIDE_SPACES         (1u)
#define WIDE_SPACES_NO_BAR  (3u)    /* $ / + %: no wide bar.             */
#define RATIO_SCALE         (10u)
#define WIDTH_MAX           (0xFFFFu)
#define LEVEL_MAX           (0xFFFFu)
#define FOLLOW_SHIFT        (3u)    /* White reference: 1/8 per sample.  */
#define FOLLOW_ROUND        ((1u << FOLLOW_SHIFT) - 1u)
#define SMOOTH_SHIFT        (2u)    /* Low-pass: 1/4 per sample (2 ms).  */
#define PROF_MAX_PEAKS      (64u)   /* Bars kept from one profile.       */
#define PROF_LEVEL_MASK     (0xFFFu)
#define PROF_SEEK_NONE      (0u)
#define PROF_SEEK_PEAK      (1u)    /* Rising: waiting for the top.      */
#define PROF_SEEK_VALLEY    (2u)    /* Falling: waiting for the bottom.  */
#define PROF_MAX_EXT        (128u)  /* Newest peaks/valleys kept.        */
#define PROF_MAX_EL         (PROF_MAX_EXT - 1u)
#define PROF_FRAC_SHIFT     (4u)    /* Edges in 1/16 bin.                */
#define PROF_CODE_LENGTHS   (3u)    /* 29, 39, 49 elements.              */

/* Profile decoder working set (one caller, T3; never re-entered). */
typedef struct
{
    uint16_t pos[PROF_MAX_EXT];
    uint16_t val[PROF_MAX_EXT];
    uint16_t count;
} prof_ext_t;

static prof_ext_t g_prof_ext;
static uint16_t   g_prof_width[PROF_MAX_EL];

typedef struct
{
    char     symbol;
    uint16_t pattern;
} code39_entry_t;

/* Widest/narrowest split of one character's bars or spaces. */
typedef struct
{
    uint16_t min_wide;
    uint16_t max_narrow;
    uint16_t min_all;
    uint16_t max_all;
    uint8_t  wide_mask;    /* Bit n set: value n is wide.                */
} split_t;

/* Standard Code 39 table (ISO/IEC 16388). */
static code39_entry_t const g_code39[NUM_SYMBOLS] =
{
    { '0', 0x034u }, { '1', 0x121u }, { '2', 0x061u }, { '3', 0x160u },
    { '4', 0x031u }, { '5', 0x130u }, { '6', 0x070u }, { '7', 0x025u },
    { '8', 0x124u }, { '9', 0x064u }, { 'A', 0x109u }, { 'B', 0x049u },
    { 'C', 0x148u }, { 'D', 0x019u }, { 'E', 0x118u }, { 'F', 0x058u },
    { 'G', 0x00Du }, { 'H', 0x10Cu }, { 'I', 0x04Cu }, { 'J', 0x01Cu },
    { 'K', 0x103u }, { 'L', 0x043u }, { 'M', 0x142u }, { 'N', 0x013u },
    { 'O', 0x112u }, { 'P', 0x052u }, { 'Q', 0x007u }, { 'R', 0x106u },
    { 'S', 0x046u }, { 'T', 0x016u }, { 'U', 0x181u }, { 'V', 0x0C1u },
    { 'W', 0x1C0u }, { 'X', 0x091u }, { 'Y', 0x190u }, { 'Z', 0x0D0u },
    { '-', 0x085u }, { '.', 0x184u }, { ' ', 0x0C4u }, { '*', 0x094u },
    { '$', 0x0A8u }, { '/', 0x0A2u }, { '+', 0x08Au }, { '%', 0x02Au }
};

static void     add_element(barcode_scan_t * p_scan, uint16_t ticks,
                            uint16_t counts, bool b_was_bar);
static void     drop_oldest(barcode_scan_t * p_scan);
static uint8_t  finish(barcode_scan_t * p_scan, barcode_result_t * p_result);
static bool     decode_capture(barcode_scan_t const * p_scan,
                               barcode_result_t * p_result);
static bool     decode_pass(uint16_t const * p_widths, uint8_t count,
                            bool b_reverse, barcode_result_t * p_result);
static uint8_t  classify_char(uint16_t const * p_widths, uint8_t count,
                              uint8_t first, bool b_reverse,
                              uint16_t * p_pattern, uint16_t * p_wide_space);
static void     split_widest(uint16_t const * p_vals, uint8_t num,
                             uint8_t num_wide, split_t * p_split);
static bool     is_separated(uint16_t wide, uint16_t narrow);
static uint16_t width_at(uint16_t const * p_widths, uint8_t count,
                         uint8_t idx, bool b_reverse);
static uint16_t add_sat(uint16_t lhs, uint16_t rhs);
static void     prof_push(prof_ext_t * p_ext, uint16_t pos, uint16_t val);
static void     prof_find(uint16_t const * p_bins, uint16_t first,
                          uint16_t count, uint16_t delta, prof_ext_t * p_ext);
static void     prof_floor(prof_ext_t * p_ext, uint16_t bar_min);
static uint32_t prof_cross(uint16_t const * p_bins, prof_ext_t const * p_ext,
                           uint16_t from_ext);
static char     lookup(uint16_t pattern);

/* Peaks (height above paper) and the valley after each (level). */
typedef struct
{
    uint16_t peak[PROF_MAX_PEAKS];
    uint16_t valley[PROF_MAX_PEAKS];
    uint16_t paper;
    uint16_t valley_open;      /* Lowest point since the last peak.     */
    uint8_t  peaks;
    bool     b_open;           /* A valley follows the last peak.       */
} prof_t;

static void     prof_extrema(uint16_t const * p_bins, uint16_t count,
                             barcode_peaks_cfg_t const * p_cfg,
                             prof_t * p_prof);
static void     prof_add_peak(prof_t * p_prof, uint16_t level,
                              uint16_t min_rise);
static void     prof_add_valley(prof_t * p_prof, uint16_t level);
static bool     prof_window(prof_t const * p_prof, uint8_t first,
                            uint8_t chars, bool b_reverse,
                            barcode_peaks_cfg_t const * p_cfg,
                            char * p_symbol);
static char     prof_char(prof_t const * p_prof, uint8_t first,
                          bool b_reverse,
                          barcode_peaks_cfg_t const * p_cfg);

/*!
 * @brief Clear a capture context and store its thresholds.
 */
void
barcode_scan_init (barcode_scan_t * p_scan, barcode_cfg_t const * p_cfg)
{
    p_scan->cfg        = *p_cfg;
    p_scan->travel     = 0u;
    p_scan->count      = 0u;
    p_scan->state      = BARCODE_ST_IDLE;
    p_scan->b_merge    = false;
}

/*!
 * @brief One sensor edge: the element that just ended is added.
 *
 * @param[in,out] p_scan    Capture context.
 * @param[in]     ticks     Element duration, 16 us ticks (saturated).
 * @param[in]     counts    Element length, encoder edges L+R (saturated).
 * @param[in]     b_black   Sensor on black AFTER the edge.
 * @param[out]    p_result  Valid when BARCODE_OUT_DECODED is returned.
 *
 * @return BARCODE_OUT_DECODED as soon as a framed code is complete,
 *         otherwise BARCODE_OUT_NONE.
 */
uint8_t
barcode_scan_edge (barcode_scan_t * p_scan, uint16_t ticks, uint16_t counts,
                   bool b_black, barcode_result_t * p_result)
{
    uint8_t out = BARCODE_OUT_NONE;

    switch (p_scan->state)
    {
        case BARCODE_ST_IDLE:
            /* White to black: the first bar starts. The element that just
             * ended is the quiet zone, which is not part of the code. */
            if (b_black)
            {
                p_scan->travel     = 0u;
                p_scan->count      = 0u;
                p_scan->b_merge    = false;
                p_scan->state      = BARCODE_ST_CAPTURE;
            }
        break;

        case BARCODE_ST_CAPTURE:
            add_element(p_scan, ticks, counts, !b_black);

            /* A bar just ended: try now, so the car can act before it
             * reaches the quiet zone. */
            if ((BARCODE_ST_CAPTURE == p_scan->state) &&
                (!p_scan->b_merge) &&
                (p_scan->count >= FIRST_DECODE_COUNT) &&
                (1u == (p_scan->count % 2u)) &&
                decode_capture(p_scan, p_result))
            {
                p_scan->state = BARCODE_ST_HOLD;
                out           = BARCODE_OUT_DECODED;
            }
        break;

        case BARCODE_ST_HOLD:
        default:
            /* Ignore edges until the quiet zone after the code. */
        break;
    }

    return out;
}

/*!
 * @brief Periodic check for the quiet zone that ends a code.
 *
 * @param[in,out] p_scan        Capture context.
 * @param[in]     ticks_since   Time since the last edge, 16 us ticks.
 * @param[in]     counts_since  Travel since the last edge, edges L+R.
 * @param[in]     b_black       Sensor on black now.
 * @param[out]    p_result      Valid unless BARCODE_OUT_NONE is returned.
 *
 * @return BARCODE_OUT_DECODED or _REJECTED when a capture ends on its
 *         quiet zone; BARCODE_OUT_NONE otherwise (also for noise shorter
 *         than one character).
 */
uint8_t
barcode_scan_idle (barcode_scan_t * p_scan, uint32_t ticks_since,
                   uint16_t counts_since, bool b_black,
                   barcode_result_t * p_result)
{
    bool const b_still = ((p_scan->travel + counts_since) <
                          p_scan->cfg.still_counts);
    bool const b_quiet = (!b_black) &&
                         ((counts_since >= p_scan->cfg.quiet_counts) ||
                          (b_still &&
                           (ticks_since >= p_scan->cfg.quiet_ticks)));
    uint8_t    out     = BARCODE_OUT_NONE;

    if (b_quiet)
    {
        if (BARCODE_ST_CAPTURE == p_scan->state)
        {
            out = finish(p_scan, p_result);
        }

        p_scan->state = BARCODE_ST_IDLE;
    }

    return out;
}

/*!
 * @brief Give up on the current capture (edges were lost in the ring);
 *        wait for the next quiet zone.
 */
void
barcode_scan_abort (barcode_scan_t * p_scan)
{
    p_scan->state = BARCODE_ST_HOLD;
}

/*!
 * @brief Decode a capture, trying the forward order first.
 *
 * @param[in]  p_widths  Element widths in any consistent unit.
 * @param[in]  count     Number of elements.
 * @param[out] p_result  Symbol, direction, reject reason, element count.
 *
 * @return true if a framed *X* code was found.
 */
bool
barcode_decode (uint16_t const * p_widths, uint8_t count,
                barcode_result_t * p_result)
{
    bool b_ok = false;

    p_result->b_by_distance = false;
    b_ok = decode_pass(p_widths, count, false, p_result);

    if (!b_ok)
    {
        barcode_result_t reverse = *p_result;

        if (decode_pass(p_widths, count, true, &reverse))
        {
            *p_result = reverse;
            b_ok      = true;
        }
    }

    return b_ok;
}

/*!
 * @brief Decode a whole pass by its peaks and valleys (see barcode.h).
 *
 * @param[in]  p_bins    blackness:12 (higher bits ignored) per distance
 *                       bin, oldest first; 0 = bin not filled yet.
 * @param[in]  count     Number of bins.
 * @param[in]  p_cfg     Thresholds.
 * @param[out] p_result  Symbol and direction when true is returned;
 *                       elements = 2 x bars - 1 of the profile.
 *
 * @return true if a framed *X* (3 to 5 characters) was found; the one
 *         nearest the end of the profile wins.
 */
bool
barcode_decode_peaks (uint16_t const * p_bins, uint16_t count,
                      barcode_peaks_cfg_t const * p_cfg,
                      barcode_result_t * p_result)
{
    static prof_t prof;          /* T3 only; kept off the task stack.    */
    bool          b_ok   = false;
    char          symbol = '\0';

    prof_extrema(p_bins, count, p_cfg, &prof);

    for (uint8_t chars = MIN_CHARS; (chars <= MAX_CHARS) && (!b_ok);
         chars++)
    {
        /* Cast: chars <= 5, so 5 x chars <= 25. */
        uint8_t const bars = (uint8_t)(chars * CHAR_BARS);

        for (uint8_t back = 0u; ((uint16_t)back + bars <= prof.peaks) &&
             (!b_ok); back++)
        {
            /* Cast: back + bars <= peaks <= 64. */
            uint8_t const first = (uint8_t)(prof.peaks - bars - back);

            if (prof_window(&prof, first, chars, false, p_cfg, &symbol))
            {
                b_ok                = true;
                p_result->b_reversed = false;
            }
            else if (prof_window(&prof, first, chars, true, p_cfg,
                                 &symbol))
            {
                b_ok                = true;
                p_result->b_reversed = true;
            }
            else
            {
                /* Try the next window back. */
            }
        }
    }

    p_result->symbol        = b_ok ? symbol : '\0';
    p_result->reason        = b_ok ? BARCODE_OK : BARCODE_ERR_PATTERN;
    p_result->b_by_distance = true;
    /* Cast: peaks <= 64, so 2 x peaks - 1 <= 127. */
    p_result->elements      = (prof.peaks > 0u) ?
                              (uint8_t)((2u * prof.peaks) - 1u) : 0u;

    return b_ok;
}

/*!
 * @brief Map a decoded symbol to a navigation command (FR3).
 *
 * @return false for any symbol other than A to D (no command generated).
 */
bool
barcode_to_command (char symbol, uint8_t * p_cmd)
{
    bool b_ok = true;

    switch (symbol)
    {
        case 'A':
            *p_cmd = MSG_BC_LEFT;
        break;

        case 'B':
            *p_cmd = MSG_BC_RIGHT;
        break;

        case 'C':
            *p_cmd = MSG_BC_STRAIGHT;
        break;

        case 'D':
            *p_cmd = MSG_BC_UTURN;
        break;

        default:
            b_ok = false;
        break;
    }

    return b_ok;
}

/*!
 * @brief Decode the newest framed code in a distance profile.
 *
 * @param[in]  p_bins    Blackness per distance bin, oldest first (bits
 *                       above BARCODE_PROF_LEVEL_MASK ignored; 0 = empty).
 * @param[in]  count     Bins.
 * @param[in]  p_cfg     Swing and bar floor, ADC counts.
 * @param[out] p_result  Symbol etc. (b_by_distance set).
 *
 * @return true if a framed code was found. Every window of 29, 39 or 49
 *         elements ending on a bar is tried, newest first.
 */
bool
barcode_decode_profile (uint16_t const * p_bins, uint16_t count,
                        barcode_prof_cfg_t const * p_cfg,
                        barcode_result_t * p_result)
{
    prof_ext_t * const p_ext  = &g_prof_ext;
    uint16_t           first  = 0u;
    uint16_t           paper  = WIDTH_MAX;
    uint16_t           widths = 0u;
    bool               b_ok   = false;

    p_result->reason   = BARCODE_ERR_LENGTH;
    p_result->elements = 0u;

    /* Skip the empty start of a ring that has not filled yet. */
    while ((first < count) &&
           (0u == (p_bins[first] & BARCODE_PROF_LEVEL_MASK)))
    {
        first++;
    }

    for (uint16_t idx = first; idx < count; idx++)
    {
        uint16_t const level = (uint16_t)(p_bins[idx] &
                                          BARCODE_PROF_LEVEL_MASK);

        paper = (level < paper) ? level : paper;
    }

    prof_find(p_bins, first, count, p_cfg->delta, p_ext);
    prof_floor(p_ext, add_sat(paper, p_cfg->floor));

    /* Extremes alternate valley, peak, ..., valley: 2k+1 of them give k
     * bars and k-1 spaces (edge to edge). */
    if (p_ext->count >= 3u)
    {
        uint32_t prev_edge = prof_cross(p_bins, p_ext, 0u);

        for (uint16_t ext = 1u; (ext + 1u) < p_ext->count; ext++)
        {
            uint32_t const edge = prof_cross(p_bins, p_ext, ext);
            uint32_t const span = edge - prev_edge;

            /* Cast: clamped to 16 bits. */
            g_prof_width[widths] = (uint16_t)((span > WIDTH_MAX) ?
                                              WIDTH_MAX : span);
            widths++;
            prev_edge = edge;
        }
    }

    /* widths is odd (bar, space, ..., bar). Newest window first. */
    for (uint16_t end = widths; (end >= FIRST_DECODE_COUNT) && (!b_ok);
         end = (uint16_t)(end - 2u))
    {
        for (uint8_t len_idx = 0u; (len_idx < PROF_CODE_LENGTHS) && (!b_ok);
             len_idx++)
        {
            /* Cast: 29 + 10 x 2 = 49 at most. */
            uint8_t const len = (uint8_t)(FIRST_DECODE_COUNT +
                                          (CHAR_STRIDE * len_idx));

            if (len <= end)
            {
                barcode_result_t attempt = *p_result;

                b_ok = barcode_decode(&g_prof_width[end - len], len,
                                      &attempt);

                if (b_ok || (widths == end))
                {
                    *p_result = attempt;
                }
            }
        }
    }

    p_result->b_by_distance = true;

    return b_ok;
}

/*!
 * @brief Clear a slicer and store its thresholds.
 *
 * The white reference starts at the top of the range and falls to the
 * paper within the first few tens of samples.
 */
void
barcode_slicer_init (barcode_slicer_t * p_slicer,
                     barcode_slicer_cfg_t const * p_cfg)
{
    p_slicer->cfg      = *p_cfg;
    p_slicer->smooth   = 0u;
    p_slicer->white    = LEVEL_MAX;
    p_slicer->black_on = LEVEL_MAX;
    p_slicer->white_on = LEVEL_MAX;
    p_slicer->creep    = 0u;
    p_slicer->extreme  = LEVEL_MAX;
    p_slicer->b_black  = false;
}

/*!
 * @brief One analogue sample: over a bar (true) or a space (false).
 *
 * @param[in,out] p_slicer  Slicer context.
 * @param[in]     sample    Blackness (higher = blacker).
 * @param[in]     travel    Encoder edges (L+R) since the previous sample.
 *
 * @return The colour after this sample (with hysteresis).
 *
 * The sample is low-passed (1/4 per sample) so ADC and sensor noise on a
 * slow, blurred bar edge does not chatter across the band. The white
 * reference follows the signal down at once (1/8 of the gap
 * per sample, so one noisy sample does not drag it to the floor). While
 * the sensor reads white it creeps up 1 count per cfg.creep_counts of
 * travel, so a slightly darker surface is learnt. It never moves on a bar
 * or while the car is still, so stopping on a bar keeps the colour.
 *
 * WARNING: called from the sampler ISR: integer only, never waits.
 */
bool
barcode_slice (barcode_slicer_t * p_slicer, uint16_t sample,
               uint16_t travel)
{
    uint16_t const step  = p_slicer->cfg.creep_counts;
    uint16_t       level = 0u;

    if (LEVEL_MAX == p_slicer->white)
    {
        /* First sample: start the filter on it. */
        p_slicer->smooth = (uint32_t)sample << SMOOTH_SHIFT;
    }
    else
    {
        p_slicer->smooth = (p_slicer->smooth -
                            (p_slicer->smooth >> SMOOTH_SHIFT)) + sample;
    }

    /* Cast: the filter output is a weighted mean of 16-bit samples. */
    level = (uint16_t)(p_slicer->smooth >> SMOOTH_SHIFT);

    if (level < p_slicer->white)
    {
        /* Cast: level < white, so the gap is 1..65535. */
        uint32_t const gap = (uint32_t)(p_slicer->white - level);

        /* Cast: the step is at most the gap, so the result >= level. */
        p_slicer->white = (uint16_t)(p_slicer->white -
                                     ((gap + FOLLOW_ROUND) >> FOLLOW_SHIFT));
        p_slicer->creep = 0u;
    }
    else if ((!p_slicer->b_black) && (0u != step))
    {
        uint32_t const total = (uint32_t)p_slicer->creep + travel;
        uint32_t const room  = LEVEL_MAX - (uint32_t)p_slicer->white;
        uint32_t       steps = total / step;

        steps = (steps > room) ? room : steps;
        /* Casts: steps <= room and the remainder < step. */
        p_slicer->white = (uint16_t)(p_slicer->white + steps);
        p_slicer->creep = (uint16_t)(total % step);
    }
    else
    {
        /* On a bar: the reference holds. */
    }

    p_slicer->black_on = add_sat(p_slicer->white, p_slicer->cfg.rise);
    p_slicer->white_on = add_sat(p_slicer->white, p_slicer->cfg.fall);

    if (!p_slicer->b_black)
    {
        p_slicer->extreme = (level < p_slicer->extreme) ? level :
                            p_slicer->extreme;

        if ((level >= p_slicer->black_on) &&
            (level >= add_sat(p_slicer->extreme, p_slicer->cfg.delta)))
        {
            p_slicer->b_black = true;
            p_slicer->extreme = level;       /* Peak of the new bar.     */
        }
    }
    else
    {
        p_slicer->extreme = (level > p_slicer->extreme) ? level :
                            p_slicer->extreme;

        if ((level <= p_slicer->white_on) ||
            ((0u != p_slicer->cfg.delta) &&
             (add_sat(level, p_slicer->cfg.delta) <= p_slicer->extreme)))
        {
            p_slicer->b_black = false;
            p_slicer->extreme = level;       /* Valley of the new space. */
        }
    }

    return p_slicer->b_black;
}

/*!
 * @brief Append one element, or merge comparator chatter.
 *
 * An element shorter than cfg.glitch_ticks is a flicker inside the
 * element before it: it and the element after it are both added to that
 * one. Colours must alternate bar, space, bar; a missed edge breaks that,
 * and the capture restarts at the next bar (nothing before it can be
 * lined up any more). A full buffer drops its oldest bar/space pair.
 */
static void
add_element (barcode_scan_t * p_scan, uint16_t ticks, uint16_t counts,
             bool b_was_bar)
{
    /* Cast: count >= 1 in that branch, so count - 1 fits uint8_t. */
    uint8_t const last = (p_scan->count > 0u) ?
                         (uint8_t)(p_scan->count - 1u) : 0u;
    uint8_t const slot = p_scan->b_merge ? last : p_scan->count;

    p_scan->travel += counts;

    if (b_was_bar != (0u == (slot % 2u)))
    {
        /* Ended on a space: a bar starts now. Ended on a bar: wait for the
         * next white-to-black edge. */
        p_scan->count   = 0u;
        p_scan->b_merge = false;
        /* Cast: both states are 2-bit constants. */
        p_scan->state   = (uint8_t)(b_was_bar ? BARCODE_ST_IDLE :
                                                BARCODE_ST_CAPTURE);
    }
    else if (p_scan->b_merge)
    {
        p_scan->width_t[last] = add_sat(p_scan->width_t[last], ticks);
        p_scan->width_d[last] = add_sat(p_scan->width_d[last], counts);
        p_scan->b_merge       = false;
    }
    else if (ticks < p_scan->cfg.glitch_ticks)
    {
        if (0u == p_scan->count)
        {
            p_scan->state = BARCODE_ST_IDLE;   /* First "bar" was a blip. */
        }
        else
        {
            p_scan->width_t[last] = add_sat(p_scan->width_t[last], ticks);
            p_scan->width_d[last] = add_sat(p_scan->width_d[last], counts);
            p_scan->b_merge       = true;
        }
    }
    else
    {
        if (p_scan->count >= APP_BC_MAX_ELEMENTS)
        {
            drop_oldest(p_scan);
        }

        p_scan->width_t[p_scan->count] = ticks;
        p_scan->width_d[p_scan->count] = counts;
        p_scan->count++;
    }
}

/*!
 * @brief Slide the buffer: forget the oldest bar and space.
 */
static void
drop_oldest (barcode_scan_t * p_scan)
{
    for (uint8_t idx = DROP_ELEMENTS; idx < p_scan->count; idx++)
    {
        /* Cast: idx >= 2 here, so idx - 2 fits uint8_t. */
        uint8_t const dst = (uint8_t)(idx - DROP_ELEMENTS);

        p_scan->width_t[dst] = p_scan->width_t[idx];
        p_scan->width_d[dst] = p_scan->width_d[idx];
    }

    /* Cast: only called with count = APP_BC_MAX_ELEMENTS (49). */
    p_scan->count = (uint8_t)(p_scan->count - DROP_ELEMENTS);
}

/*!
 * @brief Final decode when the quiet zone arrives.
 */
static uint8_t
finish (barcode_scan_t * p_scan, barcode_result_t * p_result)
{
    uint8_t out = BARCODE_OUT_NONE;

    if (p_scan->count >= BARCODE_MIN_ELEMENTS)
    {
        out = decode_capture(p_scan, p_result) ? BARCODE_OUT_DECODED :
                                                 BARCODE_OUT_REJECTED;
    }

    return out;
}

/*!
 * @brief Look for a framed code ending at the newest bar.
 *
 * Tries the last 29, 39 and 49 elements (3, 4 and 5 characters). Each
 * window is decoded on distance if the car drove across it, else on time.
 * On failure p_result holds the 29-element attempt (or, for a shorter
 * capture, the length reject).
 */
static bool
decode_capture (barcode_scan_t const * p_scan, barcode_result_t * p_result)
{
    /* Cast: count >= 1 in that branch. A window must end on a bar (odd
     * count); a trailing space is left out. */
    uint8_t const usable = ((0u != p_scan->count) &&
                            (0u == (p_scan->count % 2u))) ?
                           (uint8_t)(p_scan->count - 1u) : p_scan->count;
    bool          b_ok   = false;

    if (usable < FIRST_DECODE_COUNT)
    {
        (void)barcode_decode(p_scan->width_t, usable, p_result);
    }

    for (uint8_t len = FIRST_DECODE_COUNT; (len <= usable) && (!b_ok);
         len = (uint8_t)(len + CHAR_STRIDE))
    {
        /* Cast: len <= usable, so the start index is 0..usable. */
        uint8_t const    start   = (uint8_t)(usable - len);
        uint32_t         travel  = 0u;
        bool             b_dist  = false;
        barcode_result_t attempt = { '\0', false, false, BARCODE_OK, 0u };

        for (uint8_t idx = start; idx < usable; idx++)
        {
            travel += p_scan->width_d[idx];
        }

        b_dist = (travel >= p_scan->cfg.still_counts);
        b_ok   = barcode_decode(b_dist ? &p_scan->width_d[start] :
                                         &p_scan->width_t[start],
                                len, &attempt);
        attempt.b_by_distance = b_dist;

        if (b_ok || (FIRST_DECODE_COUNT == len))
        {
            *p_result = attempt;
        }
    }

    return b_ok;
}

static bool
decode_pass (uint16_t const * p_widths, uint8_t count, bool b_reverse,
             barcode_result_t * p_result)
{
    /* Cast: count <= 255, so (count + 1) / 10 <= 25. */
    uint8_t const chars      = (uint8_t)((count + 1u) / CHAR_STRIDE);
    uint8_t       reason     = BARCODE_OK;
    char          first_data = '\0';

    if ((chars < MIN_CHARS) || (chars > MAX_CHARS) ||
        ((uint32_t)count != (((uint32_t)chars * CHAR_STRIDE) - 1u)))
    {
        reason = BARCODE_ERR_LENGTH;
    }

    for (uint8_t chr = 0u; (chr < chars) && (BARCODE_OK == reason); chr++)
    {
        /* Cast: chr < MAX_CHARS (5), so chr x 10 <= 40. */
        uint8_t const first      = (uint8_t)(chr * CHAR_STRIDE);
        bool const    b_end_char = ((0u == chr) || ((chr + 1u) == chars));
        uint16_t      pattern    = 0u;
        uint16_t      wide_space = 0u;
        char          symbol     = '\0';

        reason = classify_char(p_widths, count, first, b_reverse, &pattern,
                               &wide_space);

        if (BARCODE_OK == reason)
        {
            symbol = lookup(pattern);
            reason = ('\0' == symbol) ? BARCODE_ERR_PATTERN : BARCODE_OK;
        }

        if ((BARCODE_OK == reason) && (b_end_char != (FRAME_SYMBOL == symbol)))
        {
            reason = BARCODE_ERR_FRAME;
        }

        /* The gap after a character is narrower than its wide space.
         * Cast: first + 9 <= 49 < count here. */
        if ((BARCODE_OK == reason) && ((chr + 1u) < chars) &&
            (width_at(p_widths, count, (uint8_t)(first + CHAR_ELEMENTS),
                      b_reverse) >= wide_space))
        {
            reason = BARCODE_ERR_GAP;
        }

        if ((BARCODE_OK == reason) && (1u == chr))
        {
            first_data = symbol;
        }
    }

    p_result->symbol     = (BARCODE_OK == reason) ? first_data : '\0';
    p_result->b_reversed = b_reverse;
    p_result->reason     = reason;
    p_result->elements   = count;

    return (BARCODE_OK == reason);
}

/*!
 * @brief Classify one character: bars and spaces are split separately.
 *
 * Normal symbols have 2 wide bars and 1 wide space; $ / + % have no wide
 * bar and 3 wide spaces. Each group's wides must be at least
 * APP_BC_WIDE_RATIO_X10 / 10 times its widest narrow.
 */
static uint8_t
classify_char (uint16_t const * p_widths, uint8_t count, uint8_t first,
               bool b_reverse, uint16_t * p_pattern, uint16_t * p_wide_space)
{
    uint16_t bars[CHAR_BARS]     = { 0u };
    uint16_t spaces[CHAR_SPACES] = { 0u };
    split_t  bar_split           = { 0u, 0u, 0u, 0u, 0u };
    split_t  space_split         = { 0u, 0u, 0u, 0u, 0u };
    uint8_t  wide_spaces         = WIDE_SPACES;
    uint8_t  reason              = BARCODE_OK;
    uint16_t pattern             = 0u;

    /* Casts: first <= 40 and the offsets <= 8, so the index fits. */
    for (uint8_t idx = 0u; idx < CHAR_BARS; idx++)
    {
        bars[idx] = width_at(p_widths, count,
                             (uint8_t)(first + (2u * idx)), b_reverse);
    }

    for (uint8_t idx = 0u; idx < CHAR_SPACES; idx++)
    {
        spaces[idx] = width_at(p_widths, count,
                               (uint8_t)(first + (2u * idx) + 1u),
                               b_reverse);
    }

    split_widest(bars, CHAR_BARS, WIDE_BARS, &bar_split);

    if (is_separated(bar_split.min_wide, bar_split.max_narrow))
    {
        wide_spaces = WIDE_SPACES;
    }
    else if (!is_separated(bar_split.max_all, bar_split.min_all))
    {
        bar_split.wide_mask = 0u;            /* All bars narrow.         */
        wide_spaces         = WIDE_SPACES_NO_BAR;
    }
    else
    {
        reason = BARCODE_ERR_RATIO;
    }

    if (BARCODE_OK == reason)
    {
        split_widest(spaces, CHAR_SPACES, wide_spaces, &space_split);

        if (!is_separated(space_split.min_wide, space_split.max_narrow))
        {
            reason = BARCODE_ERR_RATIO;
        }
    }

    for (uint8_t idx = 0u; idx < CHAR_BARS; idx++)
    {
        if (0u != (bar_split.wide_mask & (1u << idx)))
        {
            pattern |= (uint16_t)(1u << (PATTERN_TOP_BIT - (2u * idx)));
        }
    }

    for (uint8_t idx = 0u; idx < CHAR_SPACES; idx++)
    {
        if (0u != (space_split.wide_mask & (1u << idx)))
        {
            pattern |= (uint16_t)(1u << (PATTERN_TOP_BIT - ((2u * idx) + 1u)));
        }
    }

    *p_pattern    = pattern;
    *p_wide_space = space_split.min_wide;

    return reason;
}

/*!
 * @brief Mark the num_wide largest of num values as wide.
 */
static void
split_widest (uint16_t const * p_vals, uint8_t num, uint8_t num_wide,
              split_t * p_split)
{
    uint8_t  mask       = 0u;
    uint16_t min_wide   = WIDTH_MAX;
    uint16_t max_narrow = 0u;
    uint16_t min_all    = WIDTH_MAX;
    uint16_t max_all    = 0u;

    for (uint8_t pick = 0u; pick < num_wide; pick++)
    {
        uint16_t best     = 0u;
        uint8_t  best_idx = 0u;

        for (uint8_t idx = 0u; idx < num; idx++)
        {
            if ((0u == (mask & (1u << idx))) && (p_vals[idx] >= best))
            {
                best     = p_vals[idx];
                best_idx = idx;
            }
        }

        /* Cast: best_idx < 5, so the bit fits uint8_t. */
        mask     = (uint8_t)(mask | (1u << best_idx));
        min_wide = (best < min_wide) ? best : min_wide;
    }

    for (uint8_t idx = 0u; idx < num; idx++)
    {
        uint16_t const val = p_vals[idx];

        if ((0u == (mask & (1u << idx))) && (val > max_narrow))
        {
            max_narrow = val;
        }

        min_all = (val < min_all) ? val : min_all;
        max_all = (val > max_all) ? val : max_all;
    }

    p_split->min_wide   = min_wide;
    p_split->max_narrow = max_narrow;
    p_split->min_all    = min_all;
    p_split->max_all    = max_all;
    p_split->wide_mask  = mask;
}

static bool
is_separated (uint16_t wide, uint16_t narrow)
{
    return (((uint32_t)wide * RATIO_SCALE) >=
            ((uint32_t)narrow * APP_BC_WIDE_RATIO_X10));
}

static uint16_t
width_at (uint16_t const * p_widths, uint8_t count, uint8_t idx,
          bool b_reverse)
{
    /* Cast: callers keep idx < count, so the result is 0..count-1. */
    uint8_t const pos_at = b_reverse ? (uint8_t)(count - 1u - idx) : idx;

    return p_widths[pos_at];
}

/*!
 * @brief Keep a peak or valley; when full, drop the oldest pair (keeps
 *        the valley/peak alternation).
 */
static void
prof_push (prof_ext_t * p_ext, uint16_t pos, uint16_t val)
{
    if (p_ext->count >= PROF_MAX_EXT)
    {
        for (uint16_t idx = 2u; idx < p_ext->count; idx++)
        {
            p_ext->pos[idx - 2u] = p_ext->pos[idx];
            p_ext->val[idx - 2u] = p_ext->val[idx];
        }

        p_ext->count = (uint16_t)(p_ext->count - 2u);
    }

    p_ext->pos[p_ext->count] = pos;
    p_ext->val[p_ext->count] = val;
    p_ext->count++;
}

/*!
 * @brief Peaks and valleys with a swing of at least delta, starting with
 *        a valley; the trailing valley (paper after the code) is kept.
 */
static void
prof_find (uint16_t const * p_bins, uint16_t first, uint16_t count,
           uint16_t delta, prof_ext_t * p_ext)
{
    bool     b_rising = false;          /* Looking for a peak.         */
    uint16_t cur_val  = WIDTH_MAX;
    uint16_t cur_pos  = first;

    p_ext->count = 0u;

    for (uint16_t idx = first; idx < count; idx++)
    {
        uint16_t const level = (uint16_t)(p_bins[idx] &
                                          BARCODE_PROF_LEVEL_MASK);

        if (!b_rising)
        {
            if (level < cur_val)
            {
                cur_val = level;
                cur_pos = idx;
            }
            else if (level >= add_sat(cur_val, delta))
            {
                prof_push(p_ext, cur_pos, cur_val);
                b_rising = true;
                cur_val  = level;
                cur_pos  = idx;
            }
            else
            {
                /* Not yet a confirmed valley. */
            }
        }
        else
        {
            if (level > cur_val)
            {
                cur_val = level;
                cur_pos = idx;
            }
            else if (add_sat(level, delta) <= cur_val)
            {
                prof_push(p_ext, cur_pos, cur_val);
                b_rising = false;
                cur_val  = level;
                cur_pos  = idx;
            }
            else
            {
                /* Not yet a confirmed peak. */
            }
        }
    }

    if ((!b_rising) && (0u != p_ext->count))
    {
        prof_push(p_ext, cur_pos, cur_val);  /* Paper after the last bar. */
    }
}

/*!
 * @brief Drop peaks below bar_min (paper texture, not a bar): the peak
 *        and the higher of its two valleys go.
 */
static void
prof_floor (prof_ext_t * p_ext, uint16_t bar_min)
{
    uint16_t out = 0u;

    for (uint16_t idx = 0u; idx < p_ext->count; idx++)
    {
        bool const b_peak = (1u == (idx % 2u));

        if (b_peak && (p_ext->val[idx] < bar_min) &&
            ((idx + 1u) < p_ext->count) && (0u != out))
        {
            /* Skip this peak; merge its valleys into the lower one. */
            if (p_ext->val[idx + 1u] < p_ext->val[out - 1u])
            {
                p_ext->pos[out - 1u] = p_ext->pos[idx + 1u];
                p_ext->val[out - 1u] = p_ext->val[idx + 1u];
            }

            idx++;
        }
        else
        {
            p_ext->pos[out] = p_ext->pos[idx];
            p_ext->val[out] = p_ext->val[idx];
            out++;
        }
    }

    /* A list ending on a peak has no closing valley: drop the peak. */
    p_ext->count = ((0u != out) && (0u == (out % 2u))) ?
                   (uint16_t)(out - 1u) : out;
}

/*!
 * @brief Edge between extreme from_ext and the next one, in 1/16 bin: the
 *        first crossing of half-way between their levels.
 */
static uint32_t
prof_cross (uint16_t const * p_bins, prof_ext_t const * p_ext,
            uint16_t from_ext)
{
    uint16_t const a_pos  = p_ext->pos[from_ext];
    uint16_t const b_pos  = p_ext->pos[from_ext + 1u];
    uint32_t const a_val  = p_ext->val[from_ext];
    uint32_t const b_val  = p_ext->val[from_ext + 1u];
    uint32_t const mid    = (a_val + b_val) / 2u;
    bool const     b_up   = (b_val > a_val);
    uint32_t       edge   = (uint32_t)b_pos << PROF_FRAC_SHIFT;
    bool           b_done = false;

    for (uint16_t idx = (uint16_t)(a_pos + 1u); (idx <= b_pos) && (!b_done);
         idx++)
    {
        uint32_t const now  = p_bins[idx] & BARCODE_PROF_LEVEL_MASK;
        uint32_t const prev = p_bins[idx - 1u] & BARCODE_PROF_LEVEL_MASK;

        if ((b_up && (now >= mid)) || ((!b_up) && (now <= mid)))
        {
            uint32_t const step = b_up ? (now - prev) : (prev - now);
            uint32_t const part = b_up ? (mid - prev) : (prev - mid);
            /* part <= step here; step > 0 since the level crossed mid. */
            uint32_t const frac = (0u != step) ?
                                  ((part << PROF_FRAC_SHIFT) / step) : 0u;

            edge   = ((uint32_t)(idx - 1u) << PROF_FRAC_SHIFT) + frac;
            b_done = true;
        }
    }

    return edge;
}

static uint16_t
add_sat (uint16_t lhs, uint16_t rhs)
{
    uint32_t const sum = (uint32_t)lhs + (uint32_t)rhs;

    /* Cast: clamped to WIDTH_MAX first. */
    return (uint16_t)((sum > WIDTH_MAX) ? WIDTH_MAX : sum);
}

/*!
 * @brief Peaks and valleys of a profile, with hysteresis p_cfg->delta.
 *
 * Paper = the lowest filled bin. A peak lower than paper + min_rise is
 * not a bar: the valleys either side of it merge (the lower one stays).
 */
static void
prof_extrema (uint16_t const * p_bins, uint16_t count,
              barcode_peaks_cfg_t const * p_cfg, prof_t * p_prof)
{
    uint16_t first = 0u;
    uint16_t paper = LEVEL_MAX;
    uint16_t top   = 0u;
    uint16_t low   = LEVEL_MAX;
    uint8_t  seek  = PROF_SEEK_NONE;

    p_prof->peaks  = 0u;
    p_prof->b_open = false;

    while ((first < count) && (0u == (p_bins[first] & PROF_LEVEL_MASK)))
    {
        first++;                 /* Bins not filled since power-on.      */
    }

    for (uint16_t idx = first; idx < count; idx++)
    {
        uint16_t const level = (uint16_t)(p_bins[idx] & PROF_LEVEL_MASK);

        paper = (level < paper) ? level : paper;
    }

    p_prof->paper = paper;

    for (uint16_t idx = first; idx < count; idx++)
    {
        uint16_t const level = (uint16_t)(p_bins[idx] & PROF_LEVEL_MASK);

        top = (level > top) ? level : top;
        low = (level < low) ? level : low;

        if ((PROF_SEEK_VALLEY != seek) &&
            (add_sat(level, p_cfg->delta) <= top))
        {
            prof_add_peak(p_prof, top, p_cfg->min_rise);
            seek = PROF_SEEK_VALLEY;
            low  = level;
        }
        else if ((PROF_SEEK_PEAK != seek) &&
                 (level >= add_sat(low, p_cfg->delta)))
        {
            prof_add_valley(p_prof, low);
            seek = PROF_SEEK_PEAK;
            top  = level;
        }
        else
        {
            /* Still climbing or falling. */
        }
    }
}

static void
prof_add_peak (prof_t * p_prof, uint16_t level, uint16_t min_rise)
{
    if ((level >= add_sat(p_prof->paper, min_rise)) &&
        (p_prof->peaks < PROF_MAX_PEAKS))
    {
        if (p_prof->peaks > 0u)
        {
            /* Cast: peaks >= 1 here. The valley before this bar. */
            p_prof->valley[(uint8_t)(p_prof->peaks - 1u)] =
                p_prof->b_open ? p_prof->valley_open : p_prof->paper;
        }

        /* Cast: level >= paper + min_rise >= paper. */
        p_prof->peak[p_prof->peaks] = (uint16_t)(level - p_prof->paper);
        p_prof->peaks++;
        p_prof->b_open = false;
    }
}

static void
prof_add_valley (prof_t * p_prof, uint16_t level)
{
    if (p_prof->peaks > 0u)
    {
        if ((!p_prof->b_open) || (level < p_prof->valley_open))
        {
            p_prof->valley_open = level;
        }

        p_prof->b_open = true;
    }
}

/*!
 * @brief One window of chars x 5 bars: framed *X*, every character valid.
 */
static bool
prof_window (prof_t const * p_prof, uint8_t first, uint8_t chars,
             bool b_reverse, barcode_peaks_cfg_t const * p_cfg,
             char * p_symbol)
{
    bool b_ok = true;

    for (uint8_t chr = 0u; (chr < chars) && b_ok; chr++)
    {
        /* Cast: chr x 5 < 25. Reverse: characters from the far end. */
        uint8_t const offset = (uint8_t)(b_reverse ?
                                         ((chars - 1u - chr) * CHAR_BARS) :
                                         (chr * CHAR_BARS));
        char const    symbol = prof_char(p_prof, (uint8_t)(first + offset),
                                         b_reverse, p_cfg);
        bool const    b_end  = ((0u == chr) || ((chr + 1u) == chars));

        b_ok = ('\0' != symbol) && (b_end == (FRAME_SYMBOL == symbol));

        if (b_ok && (1u == chr))
        {
            *p_symbol = symbol;
        }
    }

    return b_ok;
}

/*!
 * @brief One character from 5 peaks and the 4 valleys between them:
 *        the 2 highest peaks are wide bars (each >= APP_BC_WIDE_RATIO_X10
 *        / 10 x the highest narrow one), the deepest valley the wide
 *        space (space_margin below the next). $ / + % are not read.
 *
 * @return The symbol, or '\0'.
 */
static char
prof_char (prof_t const * p_prof, uint8_t first, bool b_reverse,
           barcode_peaks_cfg_t const * p_cfg)
{
    uint16_t bars[CHAR_BARS];
    uint16_t depth[CHAR_SPACES];
    split_t  bar_split   = { 0u, 0u, 0u, 0u, 0u };
    split_t  space_split = { 0u, 0u, 0u, 0u, 0u };
    uint16_t pattern     = 0u;
    char     symbol      = '\0';

    for (uint8_t idx = 0u; idx < CHAR_BARS; idx++)
    {
        /* Cast: first + 4 < peaks <= 64. Reverse reads right to left. */
        uint8_t const pos_at = (uint8_t)(first + (b_reverse ?
                                                  (CHAR_BARS - 1u - idx) :
                                                  idx));

        bars[idx] = p_prof->peak[pos_at];
    }

    for (uint8_t idx = 0u; idx < CHAR_SPACES; idx++)
    {
        /* Cast: as above; valley n lies between peaks n and n + 1. */
        uint8_t const pos_at = (uint8_t)(first + (b_reverse ?
                                                  (CHAR_SPACES - 1u - idx) :
                                                  idx));
        uint16_t const level = p_prof->valley[pos_at];

        /* Deeper (closer to paper) = wider space. */
        depth[idx] = (uint16_t)(LEVEL_MAX - level);
    }

    split_widest(bars, CHAR_BARS, WIDE_BARS, &bar_split);
    split_widest(depth, CHAR_SPACES, WIDE_SPACES, &space_split);

    if (is_separated(bar_split.min_wide, bar_split.max_narrow) &&
        (space_split.min_wide >= add_sat(space_split.max_narrow,
                                         p_cfg->space_margin)))
    {
        for (uint8_t idx = 0u; idx < CHAR_BARS; idx++)
        {
            if (0u != (bar_split.wide_mask & (1u << idx)))
            {
                pattern |= (uint16_t)(1u << (PATTERN_TOP_BIT - (2u * idx)));
            }
        }

        for (uint8_t idx = 0u; idx < CHAR_SPACES; idx++)
        {
            if (0u != (space_split.wide_mask & (1u << idx)))
            {
                pattern |= (uint16_t)(1u << (PATTERN_TOP_BIT -
                                             ((2u * idx) + 1u)));
            }
        }

        symbol = lookup(pattern);
    }

    return symbol;
}

static char
lookup (uint16_t pattern)
{
    char symbol = '\0';

    for (uint8_t idx = 0u; (idx < NUM_SYMBOLS) && ('\0' == symbol); idx++)
    {
        if (g_code39[idx].pattern == pattern)
        {
            symbol = g_code39[idx].symbol;
        }
    }

    return symbol;
}

/*** end of file ***/
