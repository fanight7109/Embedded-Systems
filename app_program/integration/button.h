/** @file button.h
 *
 * @brief Robo Pico user buttons: GP20 START and GP21 STOP.
 *
 * Both buttons are sampled every 10 ms from cyc_ctrl (interrupt context)
 * and debounced there. Each debounced press bumps a counter that only the
 * sampler writes; the one consuming task keeps its own copy, so no lock is
 * needed (NFR12 pattern 2, single writer per variable).
 *
 *   BUTTON_START  consumed by T6 (start in READY or STOPPED, ignored in
 *                 every other state).
 *   BUTTON_STOP   consumed by T6 (commanded stop while ACTIVE, ignored
 *                 in every other state).
 *
 * Owner: Integration.
 */

#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>
#include <stdbool.h>

#define BUTTON_START            (0u)
#define BUTTON_STOP             (1u)
#define BUTTON_COUNT            (2u)

void button_init(void);
void button_sample_isr(void);
bool button_take(uint8_t button_id);
uint8_t button_presses(uint8_t button_id);

#endif /* BUTTON_H */

/*** end of file ***/
