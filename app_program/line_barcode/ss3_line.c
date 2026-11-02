/** @file ss3_line.c
 *
 * @brief Subsystem 3 entry points: ADC and barcode sampler set-up, start
 *        T3, on-target tests.
 *
 * Owner: Buddy 3 (LIM JUN HAO).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "../integration/app_cfg.h"
#include "../app_main.h"
#include "ir_line.h"
#include "bc_sensor.h"
#include "ss3_line.h"

#if (1 == APP_TEST)
#include "../integration/msg_fmt.h"
#include "../integration/test_log.h"
#include "barcode.h"
#endif

/*!
 * @brief ADC for the three analogue IR sensors, then the barcode sampler.
 *
 * @return E_OK or the bc_sensor_init() error.
 */
ER
ss3_line_init (void)
{
    ir_line_init();

    return bc_sensor_init();
}

ID
ss3_line_start (void)
{
    return app_main_start_task(ir_line_task, APP_PRI_LINE, APP_STK_LINE);
}

#if (1 == APP_TEST)

#define SHORT_COUNT         (3u)
#define TEST_WIDTH_US       (400u)
#define CODE_CHARS          (3u)
#define CHAR_ELEMENTS       (9u)
#define PATTERN_TOP_BIT     (8u)
#define WIDE_FACTOR         (3u)
#define NARROW_COUNTS       (36u)    /* About 3 mm of travel, L + R.     */
#define NARROW_TICKS        (1250u)  /* 20 ms: 3 mm at 150 mm/s.        */
#define QUIET_COUNTS        (600u)
#define BIAS_PCT            (40u)    /* Bars wider, spaces narrower.     */
#define PCT_FULL            (100u)
#define TEST_QUIET_TICKS    (25000u) /* 400 ms in 16 us ticks.           */
#define TEST_GLITCH_TICKS   (18u)    /* 300 us in 16 us ticks.           */

static uint16_t const g_test_code[CODE_CHARS] = { 0x094u, 0x049u, 0x094u };
static barcode_scan_t g_test_scan;   /* Static: keeps it off the stack. */

static uint8_t feed_element(uint16_t width, bool b_black_after,
                            barcode_result_t * p_result);
static bool    scan_test_code(uint32_t bias_pct, barcode_result_t * p_result);

/*!
 * @brief S3 unit tests: command mapping, length rejection and the full
 *        capture pipeline (barcode_scan_*) on the target compiler.
 */
void
ss3_line_test (void)
{
    uint16_t const   widths[SHORT_COUNT] = { TEST_WIDTH_US, TEST_WIDTH_US,
                                             TEST_WIDTH_US };
    barcode_result_t result;
    uint8_t          cmd  = 0xFFu;
    bool             b_ok = false;

    test_log_begin();

    b_ok = barcode_to_command('A', &cmd);
    test_log_check("S3-U01", b_ok && (MSG_BC_LEFT == cmd),
                   "symbol A maps to LEFT");

    b_ok = barcode_to_command('D', &cmd);
    test_log_check("S3-U02", b_ok && (MSG_BC_UTURN == cmd),
                   "symbol D maps to U-TURN");

    test_log_check("S3-U03", !barcode_to_command('Z', &cmd),
                   "unknown symbol is rejected");

    b_ok = barcode_decode(widths, SHORT_COUNT, &result);
    test_log_check("S3-U04", (!b_ok) && (BARCODE_ERR_LENGTH == result.reason),
                   "3-element capture rejected with ERR_LENGTH");

    test_log_check("S3-U05", (ir_line_rejects() < 0xFFu),
                   "T3 running, reject counter readable");

    b_ok = scan_test_code(0u, &result) && ('B' == result.symbol) &&
           result.b_by_distance;
    test_log_check("S3-U07", b_ok, "*B* crossed: decoded on distance");

    b_ok = scan_test_code(BIAS_PCT, &result) && ('B' == result.symbol);
    test_log_check("S3-U08", b_ok, "*B* with bars 40 % too wide decodes");

    test_log_summary("subsystem3");
}

/*!
 * @brief Feed one element of the given width (counts; ticks scaled).
 */
static uint8_t
feed_element (uint16_t width, bool b_black_after, barcode_result_t * p_result)
{
    /* Cast: width <= 3 x 1.4 x 36 counts, so the product fits easily. */
    uint16_t const ticks = (uint16_t)(((uint32_t)width * NARROW_TICKS) /
                                      NARROW_COUNTS);

    return barcode_scan_edge(&g_test_scan, ticks, width, b_black_after,
                             p_result);
}

/*!
 * @brief Drive the capture across *B* as the ISR would report it.
 *
 * @return true if the code was reported as soon as its last bar ended.
 */
static bool
scan_test_code (uint32_t bias_pct, barcode_result_t * p_result)
{
    barcode_cfg_t const cfg = { .quiet_counts = QUIET_COUNTS,
                                .still_counts = NARROW_COUNTS,
                                .quiet_ticks  = TEST_QUIET_TICKS,
                                .glitch_ticks = TEST_GLITCH_TICKS };
    uint8_t             out = BARCODE_OUT_NONE;

    barcode_scan_init(&g_test_scan, &cfg);

    /* The quiet zone ends on the first bar. */
    (void)feed_element(QUIET_COUNTS, true, p_result);

    for (uint8_t chr = 0u; chr < CODE_CHARS; chr++)
    {
        for (uint8_t elem = 0u; elem < CHAR_ELEMENTS; elem++)
        {
            bool const     b_bar  = (0u == (elem % 2u));
            bool const     b_wide = (0u != (g_test_code[chr] &
                                            (1u << (PATTERN_TOP_BIT - elem))));
            uint32_t       width  = b_wide ? (WIDE_FACTOR * NARROW_COUNTS) :
                                             NARROW_COUNTS;

            width = b_bar ? ((width * (PCT_FULL + bias_pct)) / PCT_FULL) :
                            ((width * (PCT_FULL - bias_pct)) / PCT_FULL);
            /* Cast: width <= 152, see feed_element(). */
            out   = feed_element((uint16_t)width, !b_bar, p_result);
        }

        if ((chr + 1u) < CODE_CHARS)
        {
            /* Inter-character gap (narrow space). */
            out = feed_element(NARROW_COUNTS, true, p_result);
        }
    }

    return (BARCODE_OUT_DECODED == out);
}

#endif /* APP_TEST */

/*** end of file ***/
