/** @file test_log.h
 *
 * @brief On-target test reporting in the template's fixed line format.
 *
 * Replaces the template's test_report.h, whose function-like macros break
 * Barr C 6.3.a. Every check prints one line that can be searched for in a
 * saved console log (grep '\[TEST\]' log.txt):
 *
 *     [TEST] S1-U01 PASS  header round-trips topic, QoS and retain
 *     [TEST] subsystem1 SUMMARY 4 passed, 0 failed
 *
 * IDs match testing/<subsystem>/TEST_LOG.md. Used only from usermain() in
 * TEST=1 builds, one subsystem at a time, so the counters need no lock.
 *
 * Owner: Integration.
 */

#ifndef TEST_LOG_H
#define TEST_LOG_H

#include <stdbool.h>

void test_log_begin(void);
void test_log_check(char const * p_id, bool b_pass, char const * p_desc);
void test_log_summary(char const * p_name);

#endif /* TEST_LOG_H */

/*** end of file ***/
