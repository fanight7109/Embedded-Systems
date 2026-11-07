/** @file probe.c
 *
 * @brief GP13 timing probe (debug only, NFR1 / A1).
 *
 * Owner: Integration.
 */

#include "board.h"
#include "app_cfg.h"
#include "probe.h"

void
probe_init (void)
{
#ifdef APP_PROBE_ENABLE
    board_pin_output(APP_PIN_PROBE, false);
#endif
}

void
probe_high (void)
{
#ifdef APP_PROBE_ENABLE
    board_pin_put(APP_PIN_PROBE, true);
#endif
}

void
probe_low (void)
{
#ifdef APP_PROBE_ENABLE
    board_pin_put(APP_PIN_PROBE, false);
#endif
}

/*** end of file ***/
