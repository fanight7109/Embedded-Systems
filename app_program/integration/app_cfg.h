/** @file app_cfg.h
 *
 * @brief Frozen pin map, RTOS timing, priorities and tunable constants.
 *
 * Every number that a buddy may need to calibrate lives here, so changes are
 * reviewed in one place. Values marked (CAL) are initial guesses that the
 * Week 7 validation tests (A1 to A10) replace with measured values.
 *
 * Owner: Integration (all buddies). Pin map frozen at the Week 6 review.
 */

#ifndef APP_CFG_H
#define APP_CFG_H

/* ------------------------------------------------------------------------
 * Pin map (Design Review Section 2, frozen).
 * --------------------------------------------------------------------- */
#define APP_PIN_M1A             (8u)    /* Left motor, Robo Pico M1.      */
#define APP_PIN_M1B             (9u)
#define APP_PIN_M2A             (10u)   /* Right motor, Robo Pico M2.     */
#define APP_PIN_M2B             (11u)
/* Which H-bridge input drives each motor forward (bench 26 Oct, CAL):
 * 1 = PWM on pin B for forward, 0 = PWM on pin A. */
#define APP_MOTOR_L_INVERT      (1)     /* Left: M1B is forward.          */
#define APP_MOTOR_R_INVERT      (0)     /* Right: M2A is forward.         */
#define APP_PIN_SERVO           (12u)   /* Servo port 1, 50 Hz PWM.       */
#define APP_PIN_PROBE           (13u)   /* TIMING_PROBE, plain GPIO out.  */
#define APP_PIN_US_TRIG         (4u)    /* HC-SR04 TRIG (Grove 3).        */
#define APP_PIN_US_ECHO         (5u)    /* HC-SR04 ECHO via 1k/2k divider.*/
#define APP_PIN_IR_L            (26u)   /* IR left AO  -> ADC0.           */
#define APP_PIN_IR_C            (27u)   /* IR centre AO -> ADC1.          */
#define APP_PIN_IR_BC           (28u)   /* IR right AO -> ADC2, barcode.  */
#define APP_ADC_CH_IR_L         (0u)
#define APP_ADC_CH_IR_C         (1u)
#define APP_ADC_CH_IR_BC        (2u)
#define APP_PIN_IMU_SDA         (2u)    /* I2C1 SDA (Grove 2).            */
#define APP_PIN_IMU_SCL         (3u)    /* I2C1 SCL (Grove 2).            */
#define APP_PIN_ENC_L_A         (16u)   /* Left encoder (Grove 4).        */
#define APP_PIN_ENC_L_B         (17u)
#define APP_PIN_ENC_R_A         (0u)    /* Right encoder (Grove 1).       */
#define APP_PIN_ENC_R_B         (1u)
#define APP_PIN_BTN_START       (20u)   /* User button GP20: START.       */
#define APP_PIN_BTN_STOP        (21u)   /* User button GP21: STOP.        */
#define APP_BTN_DEBOUNCE_SAMPLES (3u)   /* 3 x 10 ms equal samples.       */

/* ------------------------------------------------------------------------
 * RTOS timing. Tribunal 1 changes: T3 period 10 -> 20 ms (line and
 * barcode read time), T4 period 10 -> 20 ms (IMU priority/period relaxed).
 * T2 stays at 10 ms so NFR9 (PWM zero within 20 ms) still holds.
 * --------------------------------------------------------------------- */
#define APP_CTRL_PERIOD_MS      (10u)   /* T2 cyclic release.             */
#define APP_LINE_PERIOD_MS      (20u)   /* T3 cyclic release.             */
#define APP_LINE_PHASE_MS       (0u)    /* Tick is 10 ms: no 5 ms offset. */
#define APP_IMU_PERIOD_MS       (20u)   /* T4 cyclic release.             */
#define APP_TLM_POLL_MS         (20u)   /* T1 cyw43 poll period.          */
#define APP_PING_PERIOD_MS      (60u)   /* T5 HC-SR04 minimum cycle.      */
#define APP_NAV_RCV_TMO_MS      (100)   /* T6 mbf_nav receive timeout.    */
#define APP_JITTER_LIMIT_US     (2000u) /* NFR1 release jitter bound.     */
#define APP_OVERRUN_LIMIT       (3u)    /* F12 after 3 consecutive.       */

/*
 * Boot settle. The kernel's WiFi task (priority 4, above every app task)
 * loads the CYW43 firmware and joins the AP in the first seconds after
 * power-on and can hold the CPU for long stretches, so T2/T3 start late.
 * CALIBRATE therefore waits until MQTT is connected, or this long after
 * power-on if it never connects (no broker, WIFI=none), and only then
 * arms the F12 deadline check and spins the motors.
 */
#define APP_BOOT_SETTLE_MS      (10000u)

/*
 * Task priorities (1 is highest). The template reserves 1..4 for the
 * kernel's USB console (3) and Wi-Fi service (4) tasks, so the frozen
 * Section 3 order T2 > T3 > T6 > T5 > T4 > T1 is kept inside the
 * application range 12..17 (TEAM.md section 4).
 */
#define APP_PRI_MOTION          (12)
#define APP_PRI_LINE            (13)
#define APP_PRI_NAV             (14)
#define APP_PRI_OBSTACLE        (15)
#define APP_PRI_IMU             (16)
#define APP_PRI_TELEMETRY       (17)

/* Stack sizes in bytes; tm_printf needs about 1 KB (template app.h). */
#define APP_STK_MOTION          (2048)
#define APP_STK_LINE            (2048)
#define APP_STK_NAV             (2048)
#define APP_STK_OBSTACLE        (2048)
#define APP_STK_IMU             (2048)
#define APP_STK_TELEMETRY       (2048)

/* ------------------------------------------------------------------------
 * Motion control (Buddy 2).
 * --------------------------------------------------------------------- */
#define APP_MOTOR_PWM_HZ        (20000u) /* Robo Pico driver max 20 kHz.  */
#define APP_DUTY_MAX            (1000u)  /* 0.1 % units.                  */
#define APP_ENC_QUADRATURE      (1)      /* 0 if single-channel (W3).     */
#define APP_ENC_L_INVERT        (0)      /* Flip if forward counts down.  */
#define APP_ENC_R_INVERT        (1)      /* Mirror-mounted motor. (CAL)   */
/*
 * Edges on channel A per wheel revolution, per wheel (CAL, A2). Hand test
 * 26 Oct, 5 turns each: L 5288, R 6327 edges. Left set equal to the right
 * on 26 Oct (same motor model; the left hand count is taken as short).
 * Speeds, distances and turns use each wheel's own value.
 */
#define APP_ENC_EDGES_PER_REV_L (1265u)    /* Hand test gave 1058.       */
#define APP_ENC_EDGES_PER_REV_R (1265u)
#define APP_ENC_EDGES_PER_REV   ((APP_ENC_EDGES_PER_REV_L + \
                                  APP_ENC_EDGES_PER_REV_R) / 2u)
#define APP_WHEEL_CIRC_UM       (204200u) /* 65 mm wheel. (CAL)           */
#define APP_TRACK_WIDTH_MM      (120u)   /* Wheel centre spacing. (CAL)   */
#define APP_SPEED_WIN           (4u)     /* Speed window, cycles.         */
/*
 * Feed-forward, duty (0.1 %) per mm/s, per wheel (CAL). From the 26 Oct
 * logs: ~1.4 per mm/s as measured with the old 810 edges/rev, rescaled to
 * true speed with each wheel's edge count (1.4 x 1058/810, 1.4 x 1265/810).
 */
#define APP_FF_L_X10            (22u)    /* 2.2 duty/(mm/s), left = right.*/
#define APP_FF_R_X10            (22u)    /* 2.2 duty/(mm/s), right.       */
#define APP_STRAIGHT_K_X10      (20u)    /* Straight-line correction.     */
/*
 * Line PD (CAL, A2): steer mm/s per 0.5 mm of line error. The IR error
 * reads up to +/-63 half-mm, about 3x the real offset the straddling pair
 * can see, so 6.0/3.0 (first guess) gave +/-378 mm/s steer and lost the
 * line every cycle on 26 Oct. Start low and raise in INT-08.
 */
/* Bench 26 Oct 16:23: 1.5/1.0 swung left/right every ~200 ms and lost
 * the line (sensor ~125 mm ahead of the axle; the edge error is nearly
 * on/off, so each flip gave a +/-126 mm/s D kick). +/-31 mm/s max now. */
#define APP_LINE_KP_X10         (5u)     /* 0.5 (was 1.5).                */
#define APP_LINE_KD_X10         (0u)     /* Off (was 1.0).                */
/*
 * Slow down in bends (2 Nov): the cruise falls linearly with |error|
 * from the set speed to APP_LINE_TURN_SPEED at full error, so the same
 * +/-31 mm/s steer gives about half the turn radius (~95 mm, was ~190).
 * Straights keep small errors and so run at full speed.
 */
#define APP_LINE_TURN_SPEED     (50)     /* Cruise at full error. (CAL)   */
#define APP_LINE_ERR_FULL       (63)     /* Full-scale error, half-mm.    */
#define APP_STOP_RAMP_MS        (300u)   /* Commanded stop ramp.          */
#define APP_MIN_MOVE_SPEED      (80)     /* Approach speed near target.   */
#define APP_TURN_SPEED          (150u)   /* Default turn wheel speed.     */
#define APP_ENC_FAULT_DUTY      (300u)   /* F01: |PWM| above 30 %.        */
#define APP_ENC_FAULT_MS        (200u)   /* F01: no counts for 200 ms.    */
#define APP_LINE_STALE_MS       (100u)   /* F14 command/data timeout.     */
#define APP_PID_INTEG_LIMIT     (1000)   /* Anti-windup clamp.            */

/* Default configuration (g_config), MQTT-settable in READY. */
#define APP_DEF_BASE_SPEED      (100u)   /* mm/s (FR1). 29 Oct: barcode   */
                                         /* trace runs (was 150, then    */
                                         /* 200; raise again in INT-08). */
/* Speed PID (CAL, A2). Bench 26 Oct: 1.50 / 0.30 overshot by 2x and
 * reversed the wheels; 1.00 / 0.05 tracked. */
#define APP_DEF_KP_X100         (100u)   /* 1.00 (was 1.50).              */
#define APP_DEF_KI_X100         (5u)     /* 0.05 (was 0.30).              */
#define APP_DEF_KD_X100         (0u)
#define APP_DEF_OBSTACLE_MM     (200u)   /* FR5 threshold. 28 Oct: was    */
                                         /* 300, stopped too early.     */
#define APP_DEF_HUMP_PITCH_DEG  (4u)

/* ------------------------------------------------------------------------
 * Fitted sensors. 0 = not fitted: its task is not started, its pins stay
 * at their reset state (input, pull-down), CALIBRATE neither waits for nor
 * judges its self-test, and T6 sends it no requests. With both 0 the car
 * calibrates on the IR sensors and the motors/encoders only, which is
 * enough for line following (INT-07). Both 1 since 26 Oct (sensors
 * fitted): full FR10 self-test, adaptive speed and obstacle avoidance.
 * --------------------------------------------------------------------- */
#ifndef APP_IMU_FITTED
#define APP_IMU_FITTED          (1)     /* LSM303 on I2C1 (Grove 2).      */
#endif
#ifndef APP_ULTRASONIC_FITTED
#define APP_ULTRASONIC_FITTED   (1)     /* HC-SR04 and servo.             */
#endif

/* ------------------------------------------------------------------------
 * Line and barcode (Buddy 3).
 * --------------------------------------------------------------------- */
#define APP_IR_BLACK_IS_HIGH    (1)      /* AO rises over black (all 3).  */
#define APP_IR_MIN_CONTRAST     (300u)   /* F04 if max-min below. (CAL)   */
#define APP_IR_OVERSAMPLE       (4u)
/*
 * Line sensing mode. 1 = single-edge: the left sensor tracks the LEFT
 * edge of the line at APP_EDGE_TARGET (half black). The centre sensor may
 * sit over the line or right of it, depending on the spacing; only both
 * sensors above APP_JCT_BLACK together count as a junction (cross bar,
 * corner), and the steering holds its heading while they do.
 * 0 = the pair straddles the line (Design Review; needs ~18 mm spacing).
 */
#define APP_LINE_EDGE_FOLLOW    (1)
/*
 * Which edge the left sensor (GP26) rides in single-edge mode.
 * 1 = RIGHT edge (bench 26 Oct, team layout): the left sensor sits on the
 *     line (black) and the centre sensor (GP27) right of it on white; the
 *     car drives straight with left black / centre white, steers left as
 *     the left sensor whitens and right as it (or the centre) blackens.
 * 0 = LEFT edge (left sensor half off the line's left side).
 */
#define APP_EDGE_RIGHT          (1)
#define APP_EDGE_TARGET         (500)    /* 0 white .. 1000 black.        */
#define APP_EDGE_SEEN           (150)    /* Left sensor "sees" line above.*/
/*
 * Edge alignment after calibration: creep toward the edge and stop on
 * reaching it (within the tolerance, or crossing it between two samples:
 * the sensor goes white to black in ~3 mm, faster than one 20 ms sample).
 */
#define APP_EDGE_ALIGN_TOL      (200)    /* "On the edge" within +/- 20 %.*/
#define APP_EDGE_ALIGN_MS       (120u)   /* Reached flag held for T6.     */
#define APP_EDGE_ALIGN_SPEED    (40)     /* Pivot wheel speed cap, mm/s.  */
#define APP_JCT_BLACK           (800)    /* Both above = junction. (CAL)  */
#define APP_LINE_LOST_MS        (200u)   /* No line for 200 ms. (CAL)     */
#define APP_LINE_FOUND_SAMPLES  (3u)     /* Left (GP26) black, 3 samples. */
#define APP_JUNCTION_SAMPLES    (3u)     /* Both black, 3 samples (FR11). */
/*
 * Barcode (Code 39 *X*, wide:narrow 3:1) printed on an A4 sheet laid beside
 * the line: ~141 mm long, narrow ~3 mm, wide ~9 mm, bars start 20 mm from
 * the line. On one side the first bar is only a few mm from the paper
 * edge, so barcode.c finds the code at the end of the capture rather than
 * needing white before it. Elements are measured on encoder travel (speed
 * independent); a capture with less than APP_BC_STILL_MM of travel (hand
 * swipe) is measured on time.
 */
#define APP_BC_RING_SIZE        (64u)    /* Power of two, <= 128.         */
#define APP_BC_MAX_ELEMENTS     (49u)    /* Up to 5 Code 39 characters.   */
#define APP_BC_WIDE_RATIO_X10   (15u)    /* Wide >= 1.5 x widest narrow.  */
#define APP_BC_QUIET_MM         (25u)    /* White run ending a code: more */
                                         /* than a wide space (18 mm on   */
                                         /* the big sample page). (CAL)   */
#define APP_BC_STILL_MM         (20u)    /* Less travel = car still.      */
#define APP_BC_QUIET_MS         (400u)   /* Quiet zone when car is still. */
#define APP_BC_GLITCH_US        (1200u)  /* Shorter: noise, merged (a 3  */
                                         /* mm bar at 300 mm/s is 10 ms). */
/*
 * Barcode sensor AO on ADC2 (28 Oct: the module's DO comparator was not
 * reliable). A TIMER alarm 0 ISR samples it every APP_BC_SAMPLE_US
 * (average of APP_BC_OVERSAMPLE conversions) and a software slicer makes
 * the bar/space edges. Levels are blackness in ADC counts (0..4095,
 * higher = blacker).
 *
 * 29 Oct: the printed bars read only ~300..350 against paper ~150..190
 * (the tape line reads ~2000..2500), so fixed thresholds, or thresholds
 * from the CALIBRATE spin over the line, never saw a bar. The slicer now
 * tracks the paper (white reference W, the lowest recent reading) and
 * calls a bar at W + APP_BC_RISE and a space again at W + APP_BC_FALL.
 * Set RISE to ~50 % and FALL to ~30 % of (bar - paper) for the print in
 * use (the dashboard's barcode panel works them out from two samples).
 * 29 Oct, car: thin bars peak at ~200+, paper stays below ~200 (W ~150..
 * 160), so the bar threshold sits at W + 40 and the space one at W + 25.
 * Host model (test_barcode_analog): reads thin bars of 215 with thick
 * bars of 320 at 50..100 mm/s (course-size thin bars fail at 200), and
 * the earlier 340/160 print with up to +/- 25 counts of noise.
 * APP_BC_DELTA > 0 also flips the colour on a swing of that size from
 * the last peak/valley (for narrow spaces that do not get back to paper
 * level); the model does better without it.
 */
#define APP_BC_SAMPLE_US        (500u)   /* 2 kHz: a 3 mm bar at 250 mm/s */
                                         /* is 24 samples.               */
#define APP_BC_OVERSAMPLE       (4u)     /* Conversions per sample (2 us).*/
#define APP_BC_RISE             (40u)    /* Bar: W + 40 (~195). (CAL)     */
#define APP_BC_FALL             (25u)    /* Space: W + 25 (~180). (CAL)   */
#define APP_BC_DELTA            (0u)     /* Swing test, 0 = off. (CAL)    */
#define APP_BC_CREEP_COUNTS     (60u)    /* On white, W + 1 per ~5 mm     */
                                         /* (60 edges L+R) of travel; 6  */
                                         /* let W climb ~40 counts inside */
                                         /* a code and miss thin bars    */
                                         /* (trace, 29 Oct 23:04).       */
#define APP_BC_LOG              (1)      /* 1 = print each read on USB.   */
/*
 * Barcode trace (29 Oct): the sampler ISR keeps the last APP_BC_TRACE_BINS
 * distance bins of blackness (mean over APP_BC_TRACE_BIN_COUNTS encoder
 * edges L+R, ~0.48 mm) and the slicer's colour. A copy is published on
 * is05/car/bctrace when a capture ends (decoded, rejected or too short;
 * once the car is APP_BC_TRACE_TAIL_MM past it) and on "diag bc_trace".
 * 1024 bins =
 * ~495 mm, enough for the A4 sample code (~290 mm) and its quiet zones.
 */
#define APP_BC_TRACE_BINS       (1024u)  /* Power of two.                 */
/*
 * Peak decoder (29 Oct, barcode_decode_peaks()): once the profile has
 * been flat for APP_BC_PROF_QUIET_MM after a pass, the bins since the
 * last read are decoded by peak height (bars) and valley depth
 * (spaces). Every trace of 29 Oct that covered a whole code decodes this
 * way (*A* and *Z*, read in reverse); the edge decoder read none.
 */
#define APP_BC_PROF_DELTA       (8u)     /* Peak/valley swing. (CAL)      */
#define APP_BC_PROF_MIN_RISE    (20u)    /* Bar above paper. (CAL)        */
#define APP_BC_PROF_SPACE_GAP   (3u)     /* Wide space deeper by. (CAL)   */
#define APP_BC_PROF_QUIET_MM    (25u)    /* Flat this far: pass is over.  */
#define APP_BC_PROF_FLAT        (15u)    /* Flat = range below this.      */
#define APP_BC_TRACE_BIN_COUNTS (6u)     /* Edges L+R per bin.            */
#define APP_BC_TRACE_TAIL_MM    (40u)    /* Paper after the code to send. */
#define APP_BC_TRACE_TAIL_MS    (1500u)  /* Or this long (car stopped).   */
#define APP_BC_TRACE_PER_CYCLE  (1u)     /* Frames T1 queues per 20 ms    */
                                         /* (3 overran lwIP, 29 Oct).    */

/* ------------------------------------------------------------------------
 * IMU and terrain (Buddy 4).
 * --------------------------------------------------------------------- */
#define APP_IMU_I2C_HZ          (400000u)
#define APP_IMU_I2C_TMO_US      (2000u)
#define APP_IMU_AXIS_X_SIGN     (1)      /* +X forward. (CAL)             */
#define APP_IMU_AXIS_Z_SIGN     (1)      /* +Z up. (CAL)                  */
#define APP_IMU_BIAS_SAMPLES    (25u)    /* 0.5 s at rest.                */
#define APP_IMU_FAULT_READS     (3u)     /* F09 after 3 bad reads.        */
#define APP_IMU_REINIT_MS       (1000u)
#define APP_COLLISION_MG        (2000)   /* F08 threshold, tuned in A4.   */
#define APP_ACCEL_QUIET_MG      (60)     /* Hump samples only below this. */
#define APP_TURN_DIFF_MMS       (80)     /* TURNING class threshold.      */
#define APP_HUMP_ABORT_MM       (1000)   /* Ramp, not a hump.             */
#define APP_HUMP_MIN_MM         (3)      /* Ignore smaller bumps.         */

/* ------------------------------------------------------------------------
 * Obstacle scanning and avoidance (Buddy 5).
 * --------------------------------------------------------------------- */
#define APP_ECHO_TMO_MS         (30)     /* Echo wait (about 5 m).        */
#define APP_OBS_CONFIRM         (2u)     /* Valid echoes to confirm.      */
#define APP_OBS_INVALID_LIMIT   (3u)     /* F05 in MONITOR.               */
#define APP_SCAN_PINGS          (3u)     /* Pings per angle.              */
#define APP_SCAN_INVALID_PCT    (30u)    /* Failed scan above this.       */
#define APP_FINE_HALF_SPAN      (6u)     /* Fine scan +/- 30 degrees.     */
#define APP_SCAN_LO_IDX         (6u)     /* Scan limit 60 deg (26 Oct).   */
#define APP_SCAN_HI_IDX         (18u)    /* Scan limit 120 deg (26 Oct).  */
#define APP_SERVO_FWD_IDX       (12u)    /* 90 degrees.                   */
#define APP_SERVO_LOW_IS_RIGHT  (1)      /* 30 degrees points right.      */
#define APP_SERVO_MIN_US        (500u)   /* 0 degrees. (CAL)              */
#define APP_SERVO_MAX_US        (2500u)  /* 180 degrees. (CAL)            */
#define APP_PROFILE_WINDOW_MM   (700u)   /* Returns that matter.          */
#define APP_CORRIDOR_HALF_MM    (100)    /* CONTINUE corridor.            */
#define APP_CLEAR_MIN_MM        (250u)   /* Side clear for bypass.        */
#define APP_MIN_SAFE_MM         (150u)   /* Closer: reverse and rescan.   */
#define APP_OBS_MAX_RETRIES     (2u)
#define APP_ROBOT_HALF_W_MM     (80)     /* (CAL)                         */
#define APP_BYPASS_MARGIN_MM    (60)
#define APP_BYPASS_MIN_DEG      (20u)    /* Shallowest bypass turn. (CAL) */
#define APP_BYPASS_PASS_MM      (250)    /* Sensor offset + depth + body. */
#define APP_REVERSE_MM          (100u)
#define APP_REJOIN_DEG          (40u)    /* Within the +/- 45 heading gate.*/
#define APP_REACQUIRE_MS        (5000u)  /* F07.                          */
#define APP_LINE_SEARCH_MS      (7000u)  /* F03 (slower, wider sweeps).   */
/* 60 mm/s: the sensors cross an 18 mm line in >= 3 samples (was 120). */
#define APP_SEARCH_SPEED        (60u)
#define APP_LOST_SEARCH_DEG     (90u)    /* First sweep, towards the line.*/
#define APP_CONTINUE_HOLD_MM    (300u)   /* Ignore the same obstacle.     */
/*
 * Rejoin geometry (26 Oct, sensors on). The IR pair sits about this far
 * ahead of the wheel axle (CAL: measure axle centre to sensor, A7). When
 * the rejoin leg finds the line at APP_REJOIN_DEG, the car first drives
 * this far so the axle reaches the line, then pivots back parallel; a
 * pivot straight away left the car parallel but ~80 mm beside the line.
 * It then pivots onto the edge (EDGE_ALIGN) before line following.
 */
#define APP_IR_AHEAD_MM         (125u)   /* Axle to IR sensors. (CAL)     */
#define APP_OBS_ALIGN_TMO_MS    (2000u)  /* Edge align after a bypass.    */

/* ------------------------------------------------------------------------
 * Adaptive cruise speed (T6, LINE_FOLLOW only, 26 Oct). Each 100 ms the
 * line-follow speed moves towards a target by at most APP_SPEED_STEP_MMS
 * (100 mm/s^2, ~10 mg, below APP_ACCEL_QUIET_MG so the hump pitch stays
 * valid). Target = base_speed (MQTT-settable while driving), reduced:
 *  - linearly from base at APP_SLOW_ZONE_MM down to APP_APPROACH_SPEED at
 *    the obstacle threshold, from the HC-SR04 forward range;
 *  - to APP_HUMP_SPEED while the IMU reports a hump or a slope.
 * --------------------------------------------------------------------- */
#define APP_SPEED_ADJ_MS        (100u)   /* Speed update period.          */
#define APP_SPEED_STEP_MMS      (10u)    /* Max change per update.        */
#define APP_SLOW_ZONE_MM        (450u)   /* Start slowing (was 600).      */
#define APP_APPROACH_SPEED      (80u)    /* Speed at the threshold. (CAL) */
#define APP_HUMP_SPEED          (100u)   /* Speed over a hump. (CAL)      */

/* ------------------------------------------------------------------------
 * Navigation (T6).
 * --------------------------------------------------------------------- */
#define APP_BC_JUNCTION_MM      (400u)   /* Turn by here if no junction.  */
#define APP_BC_ALIGN_MM         (80u)    /* Axle onto junction centre.    */
#define APP_CAL_SPIN_DEG        (25u)    /* IR/encoder self-test spin.    */
#define APP_CAL_SPIN_SPEED      (60u)    /* Spin wheel speed, mm/s (45+). */
#define APP_CAL_ALIGN_TMO_MS    (4000u)  /* Edge alignment give-up time.  */
#define APP_CORNER_MM           (150u)   /* Lost this soon after a        */
                                         /* junction = corner: wide search*/
#define APP_CORNER_SEARCH_DEG   (90u)    /* Corner search sweep.          */
/*
 * Minimum edges per wheel over the calibration spin: half of the expected
 * travel. Each wheel turns 4 x APP_CAL_SPIN_DEG in total on a circle of
 * diameter APP_TRACK_WIDTH_MM; 8727 = pi / 360 x 1e6 (um per mm-degree).
 * About 207 edges with 810 edges/rev (32-bit safe up to ~40000 edges/rev).
 */
#define APP_CAL_MIN_EDGES       (((((4u * APP_CAL_SPIN_DEG) * \
                                    APP_TRACK_WIDTH_MM * 8727u) / 1000u) * \
                                  APP_ENC_EDGES_PER_REV_L) / \
                                 (2u * APP_WHEEL_CIRC_UM))

/* ------------------------------------------------------------------------
 * Network (Buddy 1). SSID, password and broker address live in the
 * template's config/wifi_secrets.h (gitignored), read by the kernel's
 * Wi-Fi and MQTT service (build with NET=lwip MQTT=1).
 * --------------------------------------------------------------------- */
#define APP_TLM_PERIOD_MS       (200u)   /* 5 Hz (FR7).                   */
#define APP_HB_PERIOD_MS        (1000u)  /* 1 Hz (FR13).                  */

/* ------------------------------------------------------------------------
 * Evidence build (EVIDENCE=1 -> APP_EVIDENCE 1): on-target recorders for
 * the PostTrib1 subsystem test plan, INT-09 .. INT-15 (app_program/
 * evidence/). 0 = not built: every hook is an empty function.
 * --------------------------------------------------------------------- */
#ifndef APP_EVIDENCE
#define APP_EVIDENCE            (0)
#endif
#define APP_PRI_EVIDENCE        (18)     /* Below T1: prints when idle.   */
#define APP_PRI_EV_STRESS_A     (19)     /* INT-15 task A (ISR-woken).    */
#define APP_PRI_EV_STRESS_B     (20)     /* INT-15 task B (back to back). */
#define APP_STK_EVIDENCE        (2048)
#define APP_STK_EV_STRESS       (1024)
#define APP_EV_RAW_T2           (4096u)  /* T2 start stamps kept: 41 s.   */
#define APP_EV_RAW_T3           (2048u)  /* T3 start stamps kept: 41 s.   */
#define APP_EV_SEC_SLOTS        (180u)   /* Per-second maxima kept: 3 min.*/
#define APP_EV_STOP_LIMIT_US    (20000u) /* NFR9: flag -> PWM zero.       */
#define APP_EV_LATCH_CHECK_MS   (5000u)  /* Still latched this much later.*/
#define APP_EV_RESUME_LIMIT_MS  (10000u) /* NFR5: telemetry back within.  */
#define APP_EV_BENCH_HOLD_MS    (200u)   /* INT-14 time per servo angle.  */
#define APP_EV_SERVO_TOL_US     (20u)    /* Pulse width vs programmed:    */
                                         /* ISR timestamp jitter margin.  */
#define APP_EV_SERVO_PER_TOL_US (200u)   /* 20 ms +/- 0.2 ms.             */
#define APP_EV_STRESS_ISR_US    (500u)   /* INT-15 ISR rate, 2 kHz.       */

#endif /* APP_CFG_H */

/*** end of file ***/
