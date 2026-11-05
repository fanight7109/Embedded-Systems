/** @file drv_ultrasonic.h
 *
 * @brief HC-SR04 ranging: TRIG on GP4, ECHO on GP5 via a 1k/2k divider.
 *
 * The echo ISR timestamps both edges. It publishes the pulse width
 * (g_echo_width_us, 15 bits used) and counts rising edges, each a single
 * writer word, then sets flg_echo ECHO_DONE on the falling edge.
 *
 * A ping is classified as:
 *   valid    - falling edge within 30 ms: range in mm;
 *   clear    - rising edge seen but no fall in 30 ms: nothing within
 *              about 5 m, range = 2047 (no-echo sentinel);
 *   missing  - no rising edge at all: sensor or wiring fault (F05 x3).
 * This refines F05 so that an open track never trips SAFE STOP.
 *
 * Owner: Buddy 5 (OON JUN KANG SHAWN). Only T5 may trigger pings.
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>

#define DRV_ULTRASONIC_VALID          (0u)
#define DRV_ULTRASONIC_CLEAR          (1u)
#define DRV_ULTRASONIC_MISSING        (2u)

void     drv_ultrasonic_init(void);
uint8_t  drv_ultrasonic_ping(uint16_t * p_range_mm);
uint16_t drv_ultrasonic_width_to_mm(uint16_t width_us);

/* Called only from gpio_dispatch_isr(). */
void     drv_ultrasonic_echo_isr(uint32_t events);

#endif /* DRV_ULTRASONIC_H */

/*** end of file ***/
