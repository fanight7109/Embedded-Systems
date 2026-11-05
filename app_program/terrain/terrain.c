/** @file terrain.c
 *
 * @brief Encoder-corrected pitch, hump estimation, motion class, impact.
 *
 * Owner: Buddy 4 (GO JUN JIE).
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "../integration/app_cfg.h"
#include "../integration/app_types.h"
#include "terrain.h"

#define LP_ALPHA            (0.3f)
#define SPEED_ALPHA         (0.25f)     /* Smooths encoder quantisation.  */
#define RAD_TO_DEG          (57.29578f)
#define DEG_TO_RAD          (0.01745329f)
#define MG_PER_MMS2         (0.1019716f)    /* 1000 / 9806.65 */
#define ENTER_SAMPLES       (2u)
#define PRE_CLIMB_DEG       (1.0f)      /* Start counting the climb here. */
#define LEVEL_SAMPLES       (3u)
#define END_FRACTION        (0.2f)
#define DESC_FRACTION       (0.5f)      /* Wheel height falling: descent. */
#define HALF_TRACK_MM       ((float32_t)APP_TRACK_WIDTH_MM / 2.0f)
#define HUMP_FIELD_MAX      (127.0f)
#define HUMP_FIELD_MASK     (0x7Fu)
#define STILL_MMS           (10)
#define UM_PER_MM_F         (1000.0f)
#define MS_PER_S            (1000)
#define PI_F                (3.14159265f)
#define HALF_PI_F           (1.57079633f)
#define TINY_F              (1.0e-6f)

static void    update_hump(terrain_t * p_terr, float32_t ds_mm, bool b_quiet,
                           float32_t roll, float32_t thr,
                           terrain_out_t * p_out);
static void    finish_hump(terrain_t * p_terr, float32_t wheel_mm,
                           terrain_out_t * p_out);
static uint8_t classify(terrain_t const * p_terr, terrain_out_t const * p_out,
                        int32_t speed, int32_t diff, int32_t accel_mg,
                        float32_t thr);
static uint8_t to_hump_field(float32_t height_mm);
static float32_t absf(float32_t value);
static float32_t atan_unit(float32_t ratio);
static float32_t atan2_rad(float32_t num, float32_t den);
static float32_t sin_rad(float32_t angle);

void
terrain_init (terrain_t * p_terr)
{
    (void)memset(p_terr, 0, sizeof(*p_terr));
    p_terr->flags.state = TERRAIN_ST_MONITOR;
    p_terr->az_f        = 1000.0f;
}

/*!
 * @brief Process one accelerometer sample with the matching wheel motion.
 *
 * @param[in]  ax_mg..az_mg    Bias-corrected acceleration, milli-g.
 * @param[in]  dl_um, dr_um    Wheel travel since the last sample, um.
 * @param[in]  period_ms       Sample period.
 * @param[in]  hump_pitch_deg  Pitch that starts a hump candidate.
 * @param[out] p_out           Results for this sample.
 */
void
terrain_update (terrain_t * p_terr, int16_t ax_mg, int16_t ay_mg,
                int16_t az_mg, int32_t dl_um, int32_t dr_um,
                uint16_t period_ms, uint8_t hump_pitch_deg,
                terrain_out_t * p_out)
{
    int32_t const   speed  = ((dl_um + dr_um) / 2) / (int32_t)period_ms;
    int32_t const   diff   = (dl_um - dr_um) / (int32_t)period_ms;
    float32_t const prev_v = p_terr->speed_f;
    float32_t const new_v  = prev_v + (SPEED_ALPHA *
                                       ((float32_t)speed - prev_v));
    float32_t const a_mms2 = ((new_v - prev_v) * (float32_t)MS_PER_S) /
                             (float32_t)period_ms;
    int32_t const   a_mg   = (int32_t)(a_mms2 * MG_PER_MMS2);
    float32_t const ax_c   = (float32_t)ax_mg - (float32_t)a_mg;
    float32_t const hp_x     = ax_c - p_terr->ax_f;
    float32_t const hp_y     = (float32_t)ay_mg - p_terr->ay_f;
    float32_t const thr    = (float32_t)hump_pitch_deg;
    bool const      b_quiet = (a_mg < APP_ACCEL_QUIET_MG) &&
                              (a_mg > -APP_ACCEL_QUIET_MG);
    /* Turning adds centripetal Y acceleration: hold roll while turning. */
    bool const      b_straight = (diff <= APP_TURN_DIFF_MMS) &&
                                 (diff >= -APP_TURN_DIFF_MMS);
    float32_t       pitch  = 0.0f;
    float32_t       roll   = 0.0f;

    (void)memset(p_out, 0, sizeof(*p_out));
    p_terr->speed_f = new_v;

    /* Impact: horizontal high-pass magnitude above the threshold (F08). */
    p_out->b_impact = (0u != p_terr->flags.b_primed) &&
                      (((hp_x * hp_x) + (hp_y * hp_y)) >
                       ((float32_t)APP_COLLISION_MG *
                        (float32_t)APP_COLLISION_MG));

    p_terr->ax_f += LP_ALPHA * (ax_c - p_terr->ax_f);
    p_terr->ay_f += LP_ALPHA * ((float32_t)ay_mg - p_terr->ay_f);
    p_terr->az_f += LP_ALPHA * ((float32_t)az_mg - p_terr->az_f);
    p_terr->flags.b_primed = 1u;

    pitch = atan2_rad(p_terr->ax_f, p_terr->az_f) * RAD_TO_DEG;
    roll  = atan2_rad(p_terr->ay_f, p_terr->az_f) * RAD_TO_DEG;

    if (isfinite(pitch) && isfinite(roll))
    {
        p_terr->pitch_deg = pitch;
        p_terr->roll_deg  = roll;
    }

    if (b_quiet)
    {
        p_terr->held_pitch = p_terr->pitch_deg;
    }

    if (b_straight)
    {
        p_terr->held_roll = p_terr->roll_deg;
    }

    update_hump(p_terr, ((float32_t)(dl_um + dr_um) / 2.0f) / UM_PER_MM_F,
                b_quiet, p_terr->held_roll, thr, p_out);

    p_out->pitch_x10 = (int16_t)(p_terr->pitch_deg * 10.0f);
    p_out->b_tilt    = (absf(p_terr->pitch_deg) > thr) ||
                       (isfinite(roll) && (absf(roll) > thr));
    p_out->cls       = classify(p_terr, p_out, speed, diff, a_mg, thr);
}

/*!
 * @brief MONITOR -> CANDIDATE -> ESTIMATE -> MONITOR hump state machine.
 *
 * While the robot accelerates, the last quiet pitch is held, so encoder
 * distance is still integrated without the corrupted pitch.
 *
 * Roll is taken relative to ref_roll, the roll just before the climb, so
 * the wheel heights share the centre height's starting point. A hump
 * starts on pitch, or on roll while the centre is rising (pitch > 0: one
 * wheel climbing, not dropping into a dip or coming down a hump). Its
 * height is the higher wheel: centre + (track / 2) x |sin roll change|.
 */
static void
update_hump (terrain_t * p_terr, float32_t ds_mm, bool b_quiet,
             float32_t roll, float32_t thr, terrain_out_t * p_out)
{
    float32_t const pitch = b_quiet ? p_terr->pitch_deg : p_terr->held_pitch;
    float32_t const roll_rel = absf(roll - p_terr->ref_roll);
    float32_t const side_mm  = absf(HALF_TRACK_MM *
                                    (sin_rad(roll * DEG_TO_RAD) -
                                     sin_rad(p_terr->ref_roll * DEG_TO_RAD)));
    bool const      b_pre    = (pitch > PRE_CLIMB_DEG) ||
                               ((roll_rel > PRE_CLIMB_DEG) && (pitch > 0.0f));
    bool const      b_rise   = (pitch > thr) ||
                               ((roll_rel > thr) && (pitch > 0.0f));
    float32_t       wheel_mm = 0.0f;

    switch (p_terr->flags.state)
    {
        case TERRAIN_ST_MONITOR:
            /*
             * Integrate the climb from the first small rise, so the samples
             * spent confirming the candidate are not lost from the height.
             */
            p_terr->pre_mm = b_pre ?
                             (p_terr->pre_mm +
                              (ds_mm * sin_rad(pitch * DEG_TO_RAD))) : 0.0f;

            if (!b_pre)
            {
                p_terr->ref_roll = roll;    /* Level: follow slow changes. */
            }

            p_terr->flags.enter_cnt = (b_rise && b_quiet) ?
                ((p_terr->flags.enter_cnt < ENTER_SAMPLES) ?
                 (p_terr->flags.enter_cnt + 1u) : p_terr->flags.enter_cnt) :
                0u;

            if (ENTER_SAMPLES == p_terr->flags.enter_cnt)
            {
                p_terr->flags.state     = TERRAIN_ST_CANDIDATE;
                p_terr->flags.b_desc    = 0u;
                p_terr->flags.level_cnt = 0u;
                p_terr->height_mm       = p_terr->pre_mm;
                p_terr->peak_mm         = p_terr->pre_mm + side_mm;
                p_terr->hump_dist_mm    = 0;
            }
        break;

        case TERRAIN_ST_CANDIDATE:
        case TERRAIN_ST_ESTIMATE:
            p_terr->height_mm    += ds_mm * sin_rad(pitch * DEG_TO_RAD);
            p_terr->hump_dist_mm += (int32_t)absf(ds_mm);
            wheel_mm        = p_terr->height_mm + side_mm;
            p_terr->peak_mm = (wheel_mm > p_terr->peak_mm) ?
                              wheel_mm : p_terr->peak_mm;

            if ((TERRAIN_ST_CANDIDATE == p_terr->flags.state) &&
                (p_terr->peak_mm >= (float32_t)APP_HUMP_MIN_MM))
            {
                p_terr->flags.state = TERRAIN_ST_ESTIMATE;
            }

            /* One-wheel humps descend with little pitch: also use the
             * wheel height falling from its peak. */
            if ((pitch < -thr) ||
                (wheel_mm < (DESC_FRACTION * p_terr->peak_mm)))
            {
                p_terr->flags.b_desc = 1u;
            }

            p_terr->flags.level_cnt = ((absf(pitch) < (thr / 2.0f)) &&
                                       (absf(roll) < (thr / 2.0f))) ?
                ((p_terr->flags.level_cnt < LEVEL_SAMPLES) ?
                 (p_terr->flags.level_cnt + 1u) : p_terr->flags.level_cnt) :
                0u;

            finish_hump(p_terr, wheel_mm, p_out);
        break;

        default:
            /* T4 owns the INIT, CALIBRATE, DEGRADED and REINIT states. */
        break;
    }

    p_out->hump_cur = (TERRAIN_ST_MONITOR == p_terr->flags.state) ? 0u :
                      to_hump_field(wheel_mm);
}

/*!
 * @brief Close, confirm or abort the current hump.
 *
 * @param[in] wheel_mm  Current height of the higher wheel.
 */
static void
finish_hump (terrain_t * p_terr, float32_t wheel_mm, terrain_out_t * p_out)
{
    bool const b_settled = (LEVEL_SAMPLES == p_terr->flags.level_cnt);
    bool const b_down    = (wheel_mm <= (END_FRACTION * p_terr->peak_mm));

    if ((0u != p_terr->flags.b_desc) && (b_settled || b_down))
    {
        if (TERRAIN_ST_ESTIMATE == p_terr->flags.state)
        {
            uint8_t const peak = to_hump_field(p_terr->peak_mm);

            p_out->b_hump_done = true;
            p_out->hump_last   = peak;
            p_terr->flags.hump_n = (p_terr->flags.hump_n < 15u) ?
                                   (p_terr->flags.hump_n + 1u) : 15u;
            if (peak > p_terr->flags.hump_max)
            {
                p_terr->flags.hump_max = (uint32_t)peak & HUMP_FIELD_MASK;
            }
        }

        p_terr->flags.state = TERRAIN_ST_MONITOR;
        p_terr->height_mm   = 0.0f;
    }
    else if ((p_terr->hump_dist_mm > APP_HUMP_ABORT_MM) ||
             ((TERRAIN_ST_CANDIDATE == p_terr->flags.state) && b_settled))
    {
        /* A long ramp or a short bump, not a hump: no event. */
        p_terr->flags.state = TERRAIN_ST_MONITOR;
        p_terr->height_mm   = 0.0f;
    }
    else
    {
        /* Hump still in progress. */
    }
}

static uint8_t
classify (terrain_t const * p_terr, terrain_out_t const * p_out,
          int32_t speed, int32_t diff, int32_t accel_mg, float32_t thr)
{
    uint8_t cls = TERRAIN_CLS_CRUISING;

    if (p_out->b_impact)
    {
        cls = TERRAIN_CLS_IMPACT;
    }
    else if (p_terr->pitch_deg > thr)
    {
        cls = TERRAIN_CLS_CLIMBING;
    }
    else if (p_terr->pitch_deg < -thr)
    {
        cls = TERRAIN_CLS_DESCENDING;
    }
    else if ((diff > APP_TURN_DIFF_MMS) || (diff < -APP_TURN_DIFF_MMS))
    {
        cls = TERRAIN_CLS_TURNING;
    }
    else if ((accel_mg >= APP_ACCEL_QUIET_MG) ||
             (accel_mg <= -APP_ACCEL_QUIET_MG))
    {
        cls = TERRAIN_CLS_ACCEL;
    }
    else if ((speed < STILL_MMS) && (speed > -STILL_MMS))
    {
        cls = TERRAIN_CLS_STATIONARY;
    }
    else
    {
        cls = TERRAIN_CLS_CRUISING;
    }

    return cls;
}

/*!
 * @brief Saturate a height to the 7-bit field (0 .. 127 mm, A8).
 */
static uint8_t
to_hump_field (float32_t height_mm)
{
    uint8_t field = 0u;

    if (isfinite(height_mm) && (height_mm > 0.0f))
    {
        field = (height_mm >= HUMP_FIELD_MAX) ? (uint8_t)HUMP_FIELD_MAX
                                              : (uint8_t)(height_mm + 0.5f);
    }

    return field;
}

static float32_t
absf (float32_t value)
{
    return (value < 0.0f) ? -value : value;
}

/*!
 * @brief atan() on [-1, 1], 9th-order minimax polynomial (error < 1e-5
 *        rad). Replaces libm, which the template does not link.
 */
static float32_t
atan_unit (float32_t ratio)
{
    float32_t const square = ratio * ratio;

    return ratio * (0.9998660f +
                    (square * (-0.3302995f +
                    (square * (0.1801410f +
                    (square * (-0.0851330f + (square * 0.0208351f))))))));
}

/*!
 * @brief Four-quadrant atan2(num, den) in radians.
 */
static float32_t
atan2_rad (float32_t num, float32_t den)
{
    float32_t const abs_num = absf(num);
    float32_t const abs_den = absf(den);
    float32_t       angle   = 0.0f;

    if ((abs_num + abs_den) > TINY_F)
    {
        angle = (abs_den >= abs_num) ? atan_unit(abs_num / abs_den)
                                     : (HALF_PI_F -
                                        atan_unit(abs_den / abs_num));
        angle = (den < 0.0f) ? (PI_F - angle) : angle;
        angle = (num < 0.0f) ? -angle : angle;
    }

    return angle;
}

/*!
 * @brief sin() for |angle| <= pi, 7th-order Taylor series folded onto
 *        [-pi/2, pi/2] (error < 2e-4).
 */
static float32_t
sin_rad (float32_t angle)
{
    float32_t folded = angle;
    float32_t square = 0.0f;

    if (folded > HALF_PI_F)
    {
        folded = PI_F - folded;
    }
    else if (folded < -HALF_PI_F)
    {
        folded = -PI_F - folded;
    }
    else
    {
        /* Already in range. */
    }

    square = folded * folded;

    return folded * (1.0f - ((square / 6.0f) * (1.0f - ((square / 20.0f) *
                     (1.0f - (square / 42.0f))))));
}

/*** end of file ***/
