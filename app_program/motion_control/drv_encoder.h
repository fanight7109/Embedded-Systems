/** @file drv_encoder.h
 *
 * @brief Wheel encoder counting (ISR) and snapshot access.
 *
 * g_enc_counts packs both 16-bit wrapping counters in one 32-bit word
 * (left in bits 31..16, right in bits 15..0). The encoder ISR is the only
 * writer; T2 and T4 take one aligned load, so they always see a consistent
 * L/R pair, and compute deltas with modular uint16_t subtraction.
 *
 * Owner: Buddy 2 (PEH JUN WEI JOEL).
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

void     drv_encoder_init(void);
uint32_t drv_encoder_snapshot(void);
uint16_t drv_encoder_left(uint32_t snapshot);
uint16_t drv_encoder_right(uint32_t snapshot);
int16_t  drv_encoder_delta(uint16_t now, uint16_t before);
int32_t  drv_encoder_edges_to_mm(int32_t edges);
int32_t  drv_encoder_mm_to_edges(int32_t distance_mm);
void     drv_encoder_set_direction_hint(bool b_left_fwd, bool b_right_fwd);

/* Called only from gpio_dispatch_isr(). */
void     drv_encoder_edge_isr(uint8_t gpio, uint32_t events);

#endif /* DRV_ENCODER_H */

/*** end of file ***/
