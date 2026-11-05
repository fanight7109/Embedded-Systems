/** @file terrain.h
 *
 * @brief Pitch, hump height, motion class and collision from accelerometer
 *        plus encoder data (FR4, FR12, Design Review 6.4).
 *
 * The LSM303DLHC has no gyroscope, so the longitudinal acceleration derived
 * from encoder speed is removed from the X axis before pitch is computed,
 * and hump samples are only integrated while that acceleration is small.
 * Centre height = sum of encoder distance x sin(pitch) over the hump. Roll
 * gives the side-to-side difference, so the reported hump is the higher
 * wheel: centre + (track / 2) x sin|roll|. A hump under one wheel only
 * (small pitch, large roll) is therefore detected and measured too.
 * Turning comes from the wheel-speed difference, never the magnetometer.
 * Collision uses only the horizontal high-pass components, so the vertical
 * jolt of driving onto a hump is not mistaken for an impact.
 *
 * Pure computation, no hardware access: unit-tested on the host.
 *
 * Owner: Buddy 4 (GO JUN JIE).
 */

#ifndef TERRAIN_H
#define TERRAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "../integration/app_types.h"

/* IMU subsystem states (7 values, 3 bits). */
#define TERRAIN_ST_INIT         (0u)
#define TERRAIN_ST_CALIBRATE    (1u)
#define TERRAIN_ST_MONITOR      (2u)
#define TERRAIN_ST_CANDIDATE    (3u)
#define TERRAIN_ST_ESTIMATE     (4u)
#define TERRAIN_ST_DEGRADED     (5u)
#define TERRAIN_ST_REINIT       (6u)

/* Motion classes (7 values, 3 bits). CRUISING added for constant speed. */
#define TERRAIN_CLS_STATIONARY  (0u)
#define TERRAIN_CLS_ACCEL       (1u)
#define TERRAIN_CLS_CLIMBING    (2u)
#define TERRAIN_CLS_DESCENDING  (3u)
#define TERRAIN_CLS_IMPACT      (4u)
#define TERRAIN_CLS_TURNING     (5u)
#define TERRAIN_CLS_CRUISING    (6u)

typedef struct
{
    uint32_t state      : 3;
    uint32_t hump_n     : 4;
    uint32_t hump_max   : 7;
    uint32_t enter_cnt  : 2;
    uint32_t level_cnt  : 2;
    uint32_t b_desc     : 1;
    uint32_t b_primed   : 1;
    uint32_t reserved   : 12;
} terrain_flags_t;

APP_STATIC_ASSERT(sizeof(terrain_flags_t) == 4u, terrain_flags_size);

typedef struct
{
    terrain_flags_t flags;
    float32_t       ax_f;
    float32_t       ay_f;
    float32_t       az_f;
    float32_t       pitch_deg;
    float32_t       held_pitch;
    float32_t       roll_deg;      /* Left side up positive.            */
    float32_t       held_roll;     /* Last roll while driving straight. */
    float32_t       ref_roll;      /* Roll before the climb began.      */
    float32_t       height_mm;
    float32_t       peak_mm;
    float32_t       speed_f;       /* Smoothed encoder speed, mm/s.     */
    float32_t       pre_mm;        /* Climb before the hump is confirmed.*/
    int32_t         hump_dist_mm;
} terrain_t;

typedef struct
{
    int16_t pitch_x10;      /* 0.1 degree, nose up positive.             */
    uint8_t cls;            /* TERRAIN_CLS_*                             */
    bool    b_tilt;
    bool    b_impact;       /* Collision candidate this sample (F08).    */
    bool    b_hump_done;    /* A hump just finished: see hump_last.      */
    uint8_t hump_last;      /* Peak height of the finished hump, mm.     */
    uint8_t hump_cur;       /* Running height of the current hump, mm.   */
} terrain_out_t;

void terrain_init(terrain_t * p_terr);
void terrain_update(terrain_t * p_terr, int16_t ax_mg, int16_t ay_mg,
                    int16_t az_mg, int32_t dl_um, int32_t dr_um,
                    uint16_t period_ms, uint8_t hump_pitch_deg,
                    terrain_out_t * p_out);

#endif /* TERRAIN_H */

/*** end of file ***/
