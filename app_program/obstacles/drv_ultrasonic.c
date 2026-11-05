/** @file drv_ultrasonic.c
 *
 * @brief HC-SR04 trigger, echo timing ISR and range conversion.
 *
 * Owner: Buddy 5 (OON JUN KANG SHAWN).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "../integration/board.h"
#include "../integration/app_cfg.h"
#include "../integration/msg_fmt.h"
#include "../integration/rtos_obj.h"
#include "drv_ultrasonic.h"

#define TRIG_PULSE_US       (10u)
#define WIDTH_MAX_US        (0x7FFFu)
#define SOUND_MM_PER_MS     (343u)      /* At about 20 degrees C.         */
#define ROUND_TRIP_DIV      (2000u)     /* us -> ms and there-and-back.   */

/* ISR-owned values: single writer each (NFR12 pattern 2). */
static volatile uint16_t g_echo_width_us = 0u;
static volatile uint8_t  g_echo_rises    = 0u;
static volatile uint32_t g_echo_rise_us  = 0u;

/*!
 * @brief Configure TRIG low and enable both echo edges.
 */
void
drv_ultrasonic_init (void)
{
    board_pin_output(APP_PIN_US_TRIG, false);
    board_pin_input(APP_PIN_US_ECHO, false);
    board_pin_irq(APP_PIN_US_ECHO, BOARD_EDGE_BOTH);
}

/*!
 * @brief Fire one ping and wait up to 30 ms for the echo (T5 only).
 *
 * @param[out] p_range_mm  Range in mm, or MSG_RANGE_NONE.
 *
 * @return DRV_ULTRASONIC_VALID, DRV_ULTRASONIC_CLEAR or DRV_ULTRASONIC_MISSING.
 */
uint8_t
drv_ultrasonic_ping (uint16_t * p_range_mm)
{
    uint8_t const rises_before = g_echo_rises;
    uint8_t       result       = DRV_ULTRASONIC_MISSING;

    rtos_obj_echo_clear();
    board_pin_put(APP_PIN_US_TRIG, true);
    board_busy_wait_us(TRIG_PULSE_US);
    board_pin_put(APP_PIN_US_TRIG, false);

    *p_range_mm = MSG_RANGE_NONE;

    if (rtos_obj_echo_wait((TMO)APP_ECHO_TMO_MS))
    {
        *p_range_mm = drv_ultrasonic_width_to_mm(g_echo_width_us);
        result      = DRV_ULTRASONIC_VALID;
    }
    else if (rises_before != g_echo_rises)
    {
        result = DRV_ULTRASONIC_CLEAR;
    }
    else
    {
        result = DRV_ULTRASONIC_MISSING;
    }

    return result;
}

/*!
 * @brief Pulse width to range, saturating to the 11-bit sentinel.
 */
uint16_t
drv_ultrasonic_width_to_mm (uint16_t width_us)
{
    uint32_t const range = ((uint32_t)width_us * SOUND_MM_PER_MS) /
                           ROUND_TRIP_DIV;

    return (uint16_t)((range >= MSG_RANGE_NONE) ? MSG_RANGE_NONE : range);
}

/*!
 * @brief Echo branch of gpio_dispatch_isr().
 *
 * WARNING: interrupt context; timestamps only and sets ECHO_DONE.
 */
void
drv_ultrasonic_echo_isr (uint32_t events)
{
    uint32_t const now = board_time_us();

    if (0u != (events & BOARD_EDGE_RISE))
    {
        g_echo_rise_us = now;
        g_echo_rises   = (uint8_t)(g_echo_rises + 1u);
    }

    if (0u != (events & BOARD_EDGE_FALL))
    {
        uint32_t const width = now - g_echo_rise_us;

        g_echo_width_us = (uint16_t)((width > WIDTH_MAX_US) ? WIDTH_MAX_US
                                                            : width);
        rtos_obj_echo_set();
    }
}

/*** end of file ***/
