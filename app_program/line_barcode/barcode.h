/** @file barcode.h
 *
 * @brief Code 39 capture and decoding for the barcode lane (FR2, NFR8).
 *
 * Hardware-independent, so the whole pipeline runs in the host tests.
 * bc_sensor.c feeds it one call per sensor edge (barcode_scan_edge()) and
 * one call per T3 cycle (barcode_scan_idle()).
 *
 * Each element (bar or space) is measured twice: in time (16 us ticks)
 * and in distance (encoder edges of both wheels added together). A capture
 * the car drove across is decoded on distance, so it does not matter if
 * the car speeds up, slows down or stops part-way. A capture with almost
 * no travel (sensor swiped by hand on the bench, or both encoders lost)
 * falls back to time.
 *
 * Decoding classifies every 9-element character on its own: the 2 widest
 * of its 5 bars and the widest of its 4 spaces are wide (0 wide bars and
 * 3 wide spaces for $ / + %). Bars and spaces are judged separately, so a
 * sensor spot that makes every bar look wider than it is (and every space
 * narrower) does not break the read. A code must be framed *...* and is
 * accepted read forwards or backwards.
 *
 * Course sheets are A4: the code is about 141 mm long (narrow ~3 mm, wide
 * ~9 mm) and starts a few mm from the paper edge on one side. Whatever
 * the sensor sees just before the first bar (paper edge, shadow, floor)
 * is skipped: a code is found as the newest 29, 39 or 49 elements.
 *
 * The bar/space decision itself is made by the slicer (barcode_slice()),
 * one call per analogue sample. Its thresholds follow the paper, not a
 * fixed ADC level: black is "rise" counts above the white reference and
 * white is "fall" counts above it. A faint print (bars ~350 against paper
 * ~160 on the IR sensor, while the tape line reads ~2500) therefore reads
 * as well as a dark one, and the line no longer sets the thresholds.
 *
 * Peak decoder (29 Oct): barcode_decode_peaks() reads a whole pass
 * from the distance-binned blackness trace instead of from edges. With
 * this sensor a thin bar only lifts the reading ~40 counts while a thick
 * one lifts it 150..800, and a narrow space never gets back to paper
 * while a wide one does, so bar/space widths measured at a threshold are
 * unreliable but peak heights and valley depths are not: per character
 * the 2 highest peaks are the wide bars and the deepest valley is the
 * wide space. bc_sensor.c runs it when a capture ends without an edge
 * decode.
 *
 * Owner: Buddy 3 (LIM JUN HAO).
 */

#ifndef BARCODE_H
#define BARCODE_H

#include <stdint.h>
#include <stdbool.h>
#include "../integration/app_cfg.h"

/* Reject reasons, for logs and the A7 decode-rate test. */
#define BARCODE_OK              (0u)
#define BARCODE_ERR_LENGTH      (1u)  /* Not 10k - 1 elements, k = 3..5. */
#define BARCODE_ERR_RATIO       (2u)  /* Wide/narrow not separable.      */
#define BARCODE_ERR_PATTERN     (3u)  /* Not a Code 39 character.        */
#define BARCODE_ERR_FRAME       (4u)  /* Missing * start or stop.        */
#define BARCODE_ERR_GAP         (5u)  /* Inter-character gap is wide.    */

/* Capture states (2 bits in the T3 telemetry frame). */
#define BARCODE_ST_IDLE         (0u)  /* Waiting for the first bar.      */
#define BARCODE_ST_CAPTURE      (1u)  /* Collecting element widths.      */
#define BARCODE_ST_HOLD         (2u)  /* Done; ignore edges until quiet. */

/* What a barcode_scan_*() call produced. */
#define BARCODE_OUT_NONE        (0u)
#define BARCODE_OUT_DECODED     (1u)  /* p_result holds the symbol.      */
#define BARCODE_OUT_REJECTED    (2u)  /* p_result->reason says why.      */

/* Fewer elements than one character: a cross line or a speck, not a
 * barcode, so it is dropped without counting a reject. */
#define BARCODE_MIN_ELEMENTS    (9u)

typedef struct
{
    char    symbol;        /* Payload (first data character).            */
    bool    b_reversed;    /* Decoded from the reversed capture.         */
    bool    b_by_distance; /* Decoded on encoder distance, not time.     */
    uint8_t reason;        /* BARCODE_OK or a BARCODE_ERR_* code.        */
    uint8_t elements;      /* Elements in the capture.                   */
} barcode_result_t;

/* Thresholds, set once by the caller (units as named). */
typedef struct
{
    uint16_t quiet_counts;  /* White this far ends a code.               */
    uint16_t still_counts;  /* Less travel = car still: use time.        */
    uint32_t quiet_ticks;   /* White this long ends a still capture.     */
    uint16_t glitch_ticks;  /* Shorter elements are comparator chatter.  */
} barcode_cfg_t;

/* Capture context; one per sensor, owned by T3. */
typedef struct
{
    barcode_cfg_t cfg;
    uint16_t      width_t[APP_BC_MAX_ELEMENTS];  /* 16 us ticks.         */
    uint16_t      width_d[APP_BC_MAX_ELEMENTS];  /* Encoder edges, L+R.  */
    uint32_t      travel;                        /* Sum of width_d.      */
    uint8_t       count;
    uint8_t       state;
    bool          b_merge;       /* Next element joins the last one.     */
} barcode_scan_t;

/* Slicer thresholds, set once by the caller (blackness in ADC counts). */
typedef struct
{
    uint16_t rise;          /* Black at white reference + rise.          */
    uint16_t fall;          /* White again at white reference + fall.    */
    uint16_t delta;         /* Swing from the last peak / valley that    */
                            /* also flips the colour (0 = off).          */
    uint16_t creep_counts;  /* On white, reference +1 per this travel.   */
} barcode_slicer_cfg_t;

/* Slicer context; one per sensor, owned by the sampler ISR. */
typedef struct
{
    barcode_slicer_cfg_t cfg;
    uint32_t             smooth;    /* Filtered blackness x 4.           */
    uint16_t             white;     /* Paper reference (lowest recent).  */
    uint16_t             black_on;  /* Thresholds in use (for DIAG).     */
    uint16_t             white_on;
    uint16_t             creep;     /* Travel toward the next +1.        */
    uint16_t             extreme;   /* Valley on white, peak on black.   */
    bool                 b_black;
} barcode_slicer_t;

/* Peak decoder thresholds (blackness in ADC counts). */
typedef struct
{
    uint16_t delta;         /* Swing that confirms a peak or a valley.   */
    uint16_t min_rise;      /* A bar peaks at least this above paper.    */
    uint16_t space_margin;  /* Wide space this much deeper than the next.*/
} barcode_peaks_cfg_t;

bool    barcode_decode_peaks(uint16_t const * p_bins, uint16_t count,
                             barcode_peaks_cfg_t const * p_cfg,
                             barcode_result_t * p_result);

void    barcode_slicer_init(barcode_slicer_t * p_slicer,
                            barcode_slicer_cfg_t const * p_cfg);
bool    barcode_slice(barcode_slicer_t * p_slicer, uint16_t sample,
                      uint16_t travel);

/*
 * Profile decoder (29 Oct): decodes a code from a blackness profile in
 * equal distance bins (the sampler's trace ring), oldest first. Bars and
 * spaces are the peaks and valleys of the profile (found with a swing of
 * delta counts); each edge is put where the profile crosses half-way
 * between its peak and valley. This reads thin bars that the spot blurs
 * down to small bumps (~200 on paper ~160, thick bars ~320+), which no
 * fixed threshold separates from the narrow spaces next to thick bars.
 */
typedef struct
{
    uint16_t delta;     /* Swing that confirms a peak or valley.         */
    uint16_t floor;     /* A bar peaks at least this above the paper.    */
} barcode_prof_cfg_t;

#define BARCODE_PROF_LEVEL_MASK (0xFFFu)   /* Other bits are ignored.    */

bool    barcode_decode_profile(uint16_t const * p_bins, uint16_t count,
                               barcode_prof_cfg_t const * p_cfg,
                               barcode_result_t * p_result);

void    barcode_scan_init(barcode_scan_t * p_scan,
                          barcode_cfg_t const * p_cfg);
uint8_t barcode_scan_edge(barcode_scan_t * p_scan, uint16_t ticks,
                          uint16_t counts, bool b_black,
                          barcode_result_t * p_result);
uint8_t barcode_scan_idle(barcode_scan_t * p_scan, uint32_t ticks_since,
                          uint16_t counts_since, bool b_black,
                          barcode_result_t * p_result);
void    barcode_scan_abort(barcode_scan_t * p_scan);

bool    barcode_decode(uint16_t const * p_widths, uint8_t count,
                       barcode_result_t * p_result);
bool    barcode_to_command(char symbol, uint8_t * p_cmd);

#endif /* BARCODE_H */

/*** end of file ***/
