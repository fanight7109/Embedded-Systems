/** @file probe.h
 *
 * @brief TIMING_PROBE on GP13 for NFR1 jitter and WCET measurement.
 *
 * Each periodic task raises its probe bit on entry and lowers it on exit;
 * a logic analyser on GP13 measures release jitter and execution time
 * (A1). Compiled out unless APP_PROBE_ENABLE is defined.
 *
 * Owner: Integration (Buddy 1 runs the A1 measurement).
 */

#ifndef PROBE_H
#define PROBE_H

void probe_init(void);
void probe_high(void);
void probe_low(void);

#endif /* PROBE_H */

/*** end of file ***/
