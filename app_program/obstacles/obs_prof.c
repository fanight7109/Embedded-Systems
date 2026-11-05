/** @file obs_prof.c
 *
 * @brief Obstacle profiling, avoidance rules and bypass plan.
 *
 * Decision rules (Design Review 6.3):
 *   CONTINUE  if no part of the obstacle lies in the +/- 100 mm corridor;
 *   REVERSE   if the closest point is under 150 mm, or no side is clear;
 *   BYPASS    on a side with >= 250 mm clearance, preferring the smaller
 *             lateral deviation;
 *   STOP      (F06) once 2 retries are used up.
 *
 * Bypass: turn theta, drive diagonally out to the side-step, turn theta
 * back, pass the obstacle, turn 40 back toward the line, drive until the
 * line is found (heading within the +/- 45 degree gate), drive
 * APP_IR_AHEAD_MM so the axle is on the line, then turn 40 to realign
 * (8 steps). T6 then pivots onto the edge.
 *
 * Theta (26 Oct): the shallowest angle, in 5 degree steps from
 * APP_BYPASS_MIN_DEG, that reaches the side-step before the obstacle face
 * (less APP_BYPASS_MARGIN_MM); 90 degrees when there is no room, which is
 * the original square bypass.
 *
 * Owner: Buddy 5 (OON JUN KANG SHAWN).
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "../integration/app_cfg.h"
#include "../integration/msg_fmt.h"
#include "obs_prof.h"

#define TRIG_SCALE          (1000)
#define CLUSTER_GAP_MM      (120)
#define SIDE_DEPTH_MM       (400)
#define UNIT_MM             (4u)
#define FIELD8_MAX          (255u)
#define PCT                 (100u)
#define RIGHT_ANGLE_DEG     (90u)
#define DEG_STEP            (5u)
#define TABLE_LO_DEG        (30u)
#define COS_FROM_SIN_DEG    (60u)
#define SEARCH_SCALE_X10    (15)
#define PARAM_MAX           (1023u)

/* cos and sin x_mm 1000 for 30 .. 150 degrees in 5 degree steps. */
static int16_t const g_cos[OBS_PROF_NUM_IDX] =
{
    866, 819, 766, 707, 643, 574, 500, 423, 342, 259, 174, 87, 0,
    -87, -174, -259, -342, -423, -500, -574, -643, -707, -766, -819, -866
};

static int16_t const g_sin[OBS_PROF_NUM_IDX] =
{
    500, 574, 643, 707, 766, 819, 866, 906, 940, 966, 985, 996, 1000,
    996, 985, 966, 940, 906, 866, 819, 766, 707, 643, 574, 500
};

static bool    is_near(obs_prof_scan_t const * p_scan, uint8_t idx);
static int32_t lateral_mm(uint16_t range, uint8_t idx);
static int32_t forward_mm(uint16_t range, uint8_t idx);
static uint8_t extend(obs_prof_scan_t const * p_scan, uint8_t from,
                      bool b_up);
static void    clearances(obs_prof_scan_t const * p_scan, uint8_t lo_idx,
                          uint8_t hi_idx, int32_t edge_left, int32_t edge_right,
                          int32_t depth, obs_prof_t * p_prof);
static uint8_t to_units(uint16_t value_mm);
static uint8_t sin_idx_for_deg(uint16_t deg);
static int32_t sin_x1000(uint16_t deg);
static int32_t cos_x1000(uint16_t deg);
static uint16_t bypass_deg(uint16_t dev_mm, uint16_t d_min_mm);

void
obs_prof_scan_reset (obs_prof_scan_t * p_scan)
{
    for (uint8_t idx = 0u; idx < OBS_PROF_NUM_IDX; idx++)
    {
        p_scan->range_mm[idx] = MSG_RANGE_NONE;
    }

    p_scan->valid_mask   = 0u;
    p_scan->scanned_mask = 0u;
}

/*!
 * @brief Build the profile from the merged coarse and fine scans.
 *
 * @return false if more than 30 % of the scanned angles stayed invalid
 *         (treated as a failed scan: reverse and rescan).
 */
bool
obs_prof_build (obs_prof_scan_t const * p_scan, obs_prof_t * p_prof)
{
    uint32_t scanned = 0u;
    uint32_t invalid = 0u;
    uint8_t  closest = OBS_PROF_NUM_IDX;

    (void)memset(p_prof, 0, sizeof(*p_prof));
    p_prof->d_min_mm     = MSG_RANGE_NONE;
    p_prof->clr_left_mm  = OBS_PROF_CLEAR_MAX_MM;
    p_prof->clr_right_mm = OBS_PROF_CLEAR_MAX_MM;
    p_prof->bearing_idx  = APP_SERVO_FWD_IDX;

    for (uint8_t idx = 0u; idx < OBS_PROF_NUM_IDX; idx++)
    {
        uint32_t const bit = (uint32_t)1u << idx;

        if (0u != (p_scan->scanned_mask & bit))
        {
            scanned++;
            invalid += (0u == (p_scan->valid_mask & bit)) ? 1u : 0u;
        }

        if (is_near(p_scan, idx) &&
            (p_scan->range_mm[idx] < p_prof->d_min_mm))
        {
            p_prof->d_min_mm = p_scan->range_mm[idx];
            closest          = idx;
        }
    }

    if (OBS_PROF_NUM_IDX != closest)
    {
        uint8_t const lo_idx = extend(p_scan, closest, false);
        uint8_t const hi_idx = extend(p_scan, closest, true);
        int32_t       edge_left  = lateral_mm(p_scan->range_mm[closest],
                                              closest);
        int32_t       edge_right = edge_left;

        for (uint8_t idx = lo_idx; idx <= hi_idx; idx++)
        {
            if (is_near(p_scan, idx))
            {
                int32_t const x_mm = lateral_mm(p_scan->range_mm[idx], idx);

                edge_left  = (x_mm > edge_left) ? x_mm : edge_left;
                edge_right = (x_mm < edge_right) ? x_mm : edge_right;
            }
        }

        p_prof->b_found     = true;
        p_prof->bearing_idx = closest;
        p_prof->width_mm    = (uint16_t)(edge_left - edge_right);
        clearances(p_scan, lo_idx, hi_idx, edge_left, edge_right,
                   forward_mm(p_prof->d_min_mm, closest) + SIDE_DEPTH_MM,
                   p_prof);
    }

    return ((invalid * PCT) <= (scanned * APP_SCAN_INVALID_PCT));
}

/*!
 * @brief Servo index around which the fine scan should centre.
 */
uint8_t
obs_prof_coarse_centre (obs_prof_t const * p_prof)
{
    return p_prof->b_found ? p_prof->bearing_idx : (uint8_t)APP_SERVO_FWD_IDX;
}

/*!
 * @brief Quantise to the 40-bit PROFILE payload (4 mm units).
 */
void
obs_prof_to_msg (obs_prof_t const * p_prof, msg_nav_t * p_msg)
{
    (void)memset(p_msg, 0, sizeof(*p_msg));
    p_msg->event      = MSG_EV_PROFILE;
    p_msg->small_a = p_prof->bearing_idx;
    p_msg->value   = p_prof->b_found ? p_prof->d_min_mm : MSG_RANGE_NONE;
    p_msg->small_b = to_units(p_prof->width_mm);
    p_msg->small_c = to_units(p_prof->clr_left_mm);
    p_msg->small_d = to_units(p_prof->clr_right_mm);
}

void
obs_prof_from_msg (msg_nav_t const * p_msg, obs_prof_t * p_prof)
{
    p_prof->bearing_idx  = p_msg->small_a;
    p_prof->d_min_mm     = p_msg->value;
    p_prof->b_found      = (MSG_RANGE_NONE != p_msg->value);
    p_prof->width_mm     = (uint16_t)(p_msg->small_b * UNIT_MM);
    p_prof->clr_left_mm  = (uint16_t)(p_msg->small_c * UNIT_MM);
    p_prof->clr_right_mm = (uint16_t)(p_msg->small_d * UNIT_MM);
}

/*!
 * @brief Apply the FR6 decision rules.
 *
 * @param[in]  retries   Reverse-and-rescan attempts already used.
 * @param[out] p_dev_mm  Lateral side-step for a bypass, mm.
 */
uint8_t
obs_prof_decide (obs_prof_t const * p_prof, uint8_t retries,
                 uint16_t * p_dev_mm)
{
    uint8_t decision = OBS_DEC_CONTINUE;

    *p_dev_mm = 0u;

    if (p_prof->b_found && (p_prof->bearing_idx < OBS_PROF_NUM_IDX))
    {
        int32_t const centre = lateral_mm(p_prof->d_min_mm,
                                          p_prof->bearing_idx);
        int32_t const half   = (int32_t)p_prof->width_mm / 2;
        int32_t const left   = centre + half;
        int32_t const right  = centre - half;
        int32_t const dev_l  = left + APP_ROBOT_HALF_W_MM +
                               APP_BYPASS_MARGIN_MM;
        int32_t const dev_r  = -right + APP_ROBOT_HALF_W_MM +
                               APP_BYPASS_MARGIN_MM;
        bool const    b_l_ok = (p_prof->clr_left_mm >= APP_CLEAR_MIN_MM);
        bool const    b_r_ok = (p_prof->clr_right_mm >= APP_CLEAR_MIN_MM);

        if ((right > APP_CORRIDOR_HALF_MM) || (left < -APP_CORRIDOR_HALF_MM))
        {
            decision = OBS_DEC_CONTINUE;
        }
        else if ((p_prof->d_min_mm >= APP_MIN_SAFE_MM) && b_l_ok &&
                 ((!b_r_ok) || (dev_l <= dev_r)))
        {
            decision  = OBS_DEC_BYPASS_LEFT;
            *p_dev_mm = (uint16_t)dev_l;
        }
        else if ((p_prof->d_min_mm >= APP_MIN_SAFE_MM) && b_r_ok)
        {
            decision  = OBS_DEC_BYPASS_RIGHT;
            *p_dev_mm = (uint16_t)dev_r;
        }
        else
        {
            decision = (retries < APP_OBS_MAX_RETRIES) ? OBS_DEC_REVERSE
                                                       : OBS_DEC_STOP;
        }
    }

    return decision;
}

/*!
 * @brief Turn a decision into motion steps for T6 to execute in order.
 *
 * @return Number of steps written (0 for CONTINUE and STOP).
 */
uint8_t
obs_prof_plan (uint8_t decision, uint16_t dev_mm, uint16_t d_min_mm,
               obs_prof_step_t * p_steps)
{
    uint8_t       count  = 0u;
    uint8_t const toward = (uint8_t)((OBS_DEC_BYPASS_LEFT == decision) ?
                                     MSG_DIR_LEFT : MSG_DIR_RIGHT);
    uint8_t const back   = (uint8_t)((MSG_DIR_LEFT == toward) ?
                                     MSG_DIR_RIGHT : MSG_DIR_LEFT);

    if (OBS_DEC_REVERSE == decision)
    {
        p_steps[0] = (obs_prof_step_t){ MSG_MC_REVERSE, APP_REVERSE_MM,
                                        MSG_DIR_LEFT, false };
        count = 1u;
    }
    else if ((OBS_DEC_BYPASS_LEFT == decision) ||
             (OBS_DEC_BYPASS_RIGHT == decision))
    {
        uint32_t const sin_rejoin = (uint32_t)g_sin[sin_idx_for_deg(
                                        APP_REJOIN_DEG)];
        uint16_t const turn_deg   = bypass_deg(dev_mm, d_min_mm);
        uint32_t const sin_turn   = (uint32_t)sin_x1000(turn_deg);
        uint32_t const fwd        = ((uint32_t)dev_mm *
                                     (uint32_t)cos_x1000(turn_deg)) / sin_turn;
        uint32_t search = (((uint32_t)dev_mm * TRIG_SCALE) / sin_rejoin) *
                          SEARCH_SCALE_X10 / 10u;
        uint32_t side   = ((uint32_t)dev_mm * TRIG_SCALE) / sin_turn;
        uint32_t pass   = (uint32_t)d_min_mm + APP_BYPASS_PASS_MM;

        /* The diagonal leg already covered fwd of the pass distance. */
        pass   = (pass > fwd) ? (pass - fwd) : 0u;
        search = (search > PARAM_MAX) ? PARAM_MAX : search;
        side   = (side > PARAM_MAX) ? PARAM_MAX : side;
        pass   = (pass > PARAM_MAX) ? PARAM_MAX : pass;

        p_steps[0] = (obs_prof_step_t){ MSG_MC_TURN_ANGLE, turn_deg,
                                        toward, false };
        p_steps[1] = (obs_prof_step_t){ MSG_MC_MOVE_DIST, (uint16_t)side,
                                        toward, false };
        p_steps[2] = (obs_prof_step_t){ MSG_MC_TURN_ANGLE, turn_deg,
                                        back, false };
        p_steps[3] = (obs_prof_step_t){ MSG_MC_MOVE_DIST, (uint16_t)pass,
                                        back, false };
        p_steps[4] = (obs_prof_step_t){ MSG_MC_TURN_ANGLE, APP_REJOIN_DEG,
                                        back, false };
        p_steps[5] = (obs_prof_step_t){ MSG_MC_MOVE_DIST, (uint16_t)search,
                                        back, true };
        /* Axle onto the line before pivoting back parallel to it. */
        p_steps[6] = (obs_prof_step_t){ MSG_MC_MOVE_DIST, APP_IR_AHEAD_MM,
                                        back, false };
        p_steps[7] = (obs_prof_step_t){ MSG_MC_TURN_ANGLE, APP_REJOIN_DEG,
                                        toward, false };
        count = OBS_PROF_MAX_STEPS;
    }
    else
    {
        count = 0u;
    }

    return count;
}

static bool
is_near (obs_prof_scan_t const * p_scan, uint8_t idx)
{
    uint32_t const bit = (uint32_t)1u << idx;

    return (0u != (p_scan->valid_mask & bit)) &&
           (p_scan->range_mm[idx] < MSG_RANGE_NONE) &&
           (p_scan->range_mm[idx] <= APP_PROFILE_WINDOW_MM);
}

static int32_t
lateral_mm (uint16_t range, uint8_t idx)
{
    int32_t const x_right = ((int32_t)range * g_cos[idx]) / TRIG_SCALE;

#if (1 == APP_SERVO_LOW_IS_RIGHT)
    return -x_right;
#else
    return x_right;
#endif
}

static int32_t
forward_mm (uint16_t range, uint8_t idx)
{
    return ((int32_t)range * g_sin[idx]) / TRIG_SCALE;
}

/*!
 * @brief Grow the cluster around the closest return, skipping angles
 *        that were not scanned, while ranges stay continuous.
 */
static uint8_t
extend (obs_prof_scan_t const * p_scan, uint8_t from, bool b_up)
{
    uint8_t edge = from;
    bool    b_go = true;

    while (b_go)
    {
        uint8_t next = edge;

        do
        {
            next = b_up ? (uint8_t)(next + 1u) : (uint8_t)(next - 1u);
        } while ((next < OBS_PROF_NUM_IDX) &&
                 (0u == (p_scan->scanned_mask & ((uint32_t)1u << next))));

        if ((next < OBS_PROF_NUM_IDX) && is_near(p_scan, next))
        {
            int32_t const gap = (int32_t)p_scan->range_mm[next] -
                                (int32_t)p_scan->range_mm[edge];

            b_go = ((gap < CLUSTER_GAP_MM) && (gap > -CLUSTER_GAP_MM));
            edge = b_go ? next : edge;
        }
        else
        {
            b_go = false;
        }
    }

    return edge;
}

/*!
 * @brief Free lateral space beside the cluster, from returns at a similar
 *        depth; unlimited (1020 mm) if nothing is seen on that side.
 */
static void
clearances (obs_prof_scan_t const * p_scan, uint8_t lo_idx, uint8_t hi_idx,
            int32_t edge_left, int32_t edge_right, int32_t depth,
            obs_prof_t * p_prof)
{
    int32_t clr_l = OBS_PROF_CLEAR_MAX_MM;
    int32_t clr_r = OBS_PROF_CLEAR_MAX_MM;

    for (uint8_t idx = 0u; idx < OBS_PROF_NUM_IDX; idx++)
    {
        if (((idx < lo_idx) || (idx > hi_idx)) && is_near(p_scan, idx) &&
            (forward_mm(p_scan->range_mm[idx], idx) <= depth))
        {
            int32_t const x_mm = lateral_mm(p_scan->range_mm[idx], idx);

            if ((x_mm > edge_left) && ((x_mm - edge_left) < clr_l))
            {
                clr_l = x_mm - edge_left;
            }
            else if ((x_mm < edge_right) && ((edge_right - x_mm) < clr_r))
            {
                clr_r = edge_right - x_mm;
            }
            else
            {
                /* Not closer than what is already known. */
            }
        }
    }

    p_prof->clr_left_mm  = (uint16_t)clr_l;
    p_prof->clr_right_mm = (uint16_t)clr_r;
}

static uint8_t
to_units (uint16_t value_mm)
{
    uint32_t const units = ((uint32_t)value_mm + (UNIT_MM / 2u)) / UNIT_MM;

    return (uint8_t)((units > FIELD8_MAX) ? FIELD8_MAX : units);
}

/*!
 * @brief Table index whose sine matches @p deg (0 .. 90 by symmetry).
 */
static uint8_t
sin_idx_for_deg (uint16_t deg)
{
    uint16_t const clamped = (deg < 30u) ? 30u : ((deg > 90u) ? 90u : deg);

    return (uint8_t)((clamped - 30u) / 5u);
}

/*!
 * @brief sin(deg) x 1000 for 0 .. 90 degrees in 5 degree steps, from the
 *        tables: sin(deg) = cos(90 - deg) below 30 degrees.
 */
static int32_t
sin_x1000 (uint16_t deg)
{
    uint16_t const clamped = (deg > RIGHT_ANGLE_DEG) ? RIGHT_ANGLE_DEG : deg;
    int32_t        value   = 0;

    if (clamped >= TABLE_LO_DEG)
    {
        value = g_sin[(clamped - TABLE_LO_DEG) / DEG_STEP];
    }
    else
    {
        value = g_cos[(COS_FROM_SIN_DEG - clamped) / DEG_STEP];
    }

    return value;
}

static int32_t
cos_x1000 (uint16_t deg)
{
    uint16_t const clamped = (deg > RIGHT_ANGLE_DEG) ? RIGHT_ANGLE_DEG : deg;

    return sin_x1000((uint16_t)(RIGHT_ANGLE_DEG - clamped));
}

/*!
 * @brief Shallowest bypass turn that reaches the side-step @p dev_mm
 *        before the obstacle face, keeping APP_BYPASS_MARGIN_MM in hand.
 *
 * Forward travel on the diagonal leg is dev / tan(theta), so the test is
 * dev x cos(theta) <= budget x sin(theta). d_min is the slant range, never
 * more than the forward distance, so the budget errs short.
 *
 * @return Degrees, APP_BYPASS_MIN_DEG .. 85, or 90 if none fits.
 */
static uint16_t
bypass_deg (uint16_t dev_mm, uint16_t d_min_mm)
{
    int32_t const budget = (int32_t)d_min_mm - APP_BYPASS_MARGIN_MM;
    uint16_t      deg    = RIGHT_ANGLE_DEG;
    bool          b_fits = false;

    for (uint16_t try_deg = APP_BYPASS_MIN_DEG;
         (try_deg < RIGHT_ANGLE_DEG) && (!b_fits);
         try_deg = (uint16_t)(try_deg + DEG_STEP))
    {
        int32_t const need = (int32_t)dev_mm * cos_x1000(try_deg);
        int32_t const have = budget * sin_x1000(try_deg);

        if (need <= have)
        {
            deg    = try_deg;
            b_fits = true;
        }
    }

    return deg;
}

/*** end of file ***/
