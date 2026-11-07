/** @file test_log.c
 *
 * @brief On-target test reporting (see test_log.h).
 *
 * Owner: Integration.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "test_log.h"

static uint16_t g_passed = 0u;
static uint16_t g_failed = 0u;

/*!
 * @brief Reset the counters before one subsystem's tests.
 */
void
test_log_begin (void)
{
    g_passed = 0u;
    g_failed = 0u;
}

/*!
 * @brief Print one PASS or FAIL line and count it.
 */
void
test_log_check (char const * p_id, bool b_pass, char const * p_desc)
{
    if (b_pass)
    {
        g_passed++;
        (void)tm_printf((UB *)"[TEST] %s PASS  %s\n", p_id, p_desc);
    }
    else
    {
        g_failed++;
        (void)tm_printf((UB *)"[TEST] %s FAIL  %s\n", p_id, p_desc);
    }
}

/*!
 * @brief Print the summary line for one subsystem.
 */
void
test_log_summary (char const * p_name)
{
    (void)tm_printf((UB *)"[TEST] %s SUMMARY %d passed, %d failed\n",
                    p_name, (INT)g_passed, (INT)g_failed);
}

/*** end of file ***/
