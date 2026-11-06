/** @file bitpack.h
 *
 * @brief MSB-first bit packing for shared, queued and transmitted data.
 *
 * Implements rule R5 of Design Review Section 1.5: fields are written into
 * plain uint8_t arrays with shifts and masks, so the layout never depends
 * on struct padding, bit-field order or endianness (Barr C 5.5). Signed
 * values are converted to offset binary first (rule R2, Barr C 5.3).
 *
 * Owner: Buddy 1 (JONATHAN TAN JIANG MING); used by every task.
 */

#ifndef BITPACK_H
#define BITPACK_H

#include <stdint.h>

void     bitpack_put(uint8_t * p_buf, uint16_t * p_pos, uint32_t value,
                     uint8_t width);
uint32_t bitpack_get(uint8_t const * p_buf, uint16_t pos, uint8_t width);
uint32_t bitpack_offset(int32_t value, int32_t offset, uint8_t width,
                        uint8_t * p_sat_count);
uint32_t bitpack_limit(uint32_t value, uint8_t width, uint8_t * p_sat_count);
int32_t  bitpack_signed(uint32_t field, int32_t offset);
uint16_t bitpack_bytes(uint16_t bits);

#endif /* BITPACK_H */

/*** end of file ***/
