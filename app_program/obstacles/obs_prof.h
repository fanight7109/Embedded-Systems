/** @file obs_prof.h
 *
 * @brief Obstacle profile generator, avoidance decision and bypass plan
 *        (brief Stages 3 to 5; Design Review 6.3, FR5, FR6).
 *
 * Coordinates: servo index 12 is straight ahead; lateral x is positive to
 * the LEFT of the robot and y is forward, both in mm from the sensor.
 * Trigonometry uses a fixed table (x 1000), so no floating point is used.
 *
 * T5 builds the profile; T6 calls the decision and plan functions, so the
 * navigation coordinator chooses the manoeuvre (Tribunal 1 position) using
 * rules owned by Buddy 5. Pure functions: unit-tested on the host.
 *
 * Owner: Buddy 5 (OON JUN KANG SHAWN).
 */

#ifndef OBS_PROF_H
#define OBS_PROF_H

#include <stdint.h>
#include <stdbool.h>
#include "../integration/msg_fmt.h"

#define OBS_PROF_NUM_IDX        (25u)
#define OBS_PROF_CLEAR_MAX_MM   (1020u)   /* 8 bits x 4 mm.               */

/* Decisions (FR6). */
#define OBS_DEC_CONTINUE        (0u)
#define OBS_DEC_BYPASS_LEFT     (1u)
#define OBS_DEC_BYPASS_RIGHT    (2u)
#define OBS_DEC_REVERSE         (3u)      /* Reverse 100 mm then rescan.  */
#define OBS_DEC_STOP            (4u)      /* F06.                         */

typedef struct
{
    uint16_t range_mm[OBS_PROF_NUM_IDX];  /* MSG_RANGE_NONE if clear.     */
    uint32_t valid_mask;                  /* Valid or clear reading.      */
    uint32_t scanned_mask;                /* Angle attempted.             */
} obs_prof_scan_t;

typedef struct
{
    bool     b_found;
    uint8_t  bearing_idx;
    uint16_t d_min_mm;
    uint16_t width_mm;
    uint16_t clr_left_mm;
    uint16_t clr_right_mm;
} obs_prof_t;

typedef struct
{
    uint8_t  type;        /* MSG_MC_TURN_ANGLE, MOVE_DIST or REVERSE.     */
    uint16_t param;       /* Degrees or mm.                               */
    uint8_t  dir;         /* MSG_DIR_LEFT or MSG_DIR_RIGHT for turns.     */
    bool     b_search;    /* Stop early when the line is found.           */
} obs_prof_step_t;

#define OBS_PROF_MAX_STEPS      (8u)

void    obs_prof_scan_reset(obs_prof_scan_t * p_scan);
bool    obs_prof_build(obs_prof_scan_t const * p_scan, obs_prof_t * p_prof);
uint8_t obs_prof_coarse_centre(obs_prof_t const * p_prof);
void    obs_prof_to_msg(obs_prof_t const * p_prof, msg_nav_t * p_msg);
void    obs_prof_from_msg(msg_nav_t const * p_msg, obs_prof_t * p_prof);
uint8_t obs_prof_decide(obs_prof_t const * p_prof, uint8_t retries,
                        uint16_t * p_dev_mm);
uint8_t obs_prof_plan(uint8_t decision, uint16_t dev_mm, uint16_t d_min_mm,
                      obs_prof_step_t * p_steps);

#endif /* OBS_PROF_H */

/*** end of file ***/
