/** @file bitpack.c
 *
 * @brief MSB-first bit packing with saturation (Design Review 1.4, 1.5).
 *
 * The caller owns the saturation counter (one per task), so packing never
 * needs a lock and stays race-free under NFR12.
 *
 * Owner: Buddy 1 (JONATHAN TAN JIANG MING).
 */

#include <stddef.h>
#include <stdint.h>
#include "bitpack.h"

#define BITPACK_BYTE_BITS   (8u)
#define BITPACK_MAX_WIDTH   (32u)
#define BITPACK_SAT_MAX     (255u)

static uint32_t field_mask(uint8_t width);
static void     count_saturation(uint8_t * p_sat_count);

/*!
 * @brief Append a field to a byte buffer, most significant bit first.
 *
 * @param[in,out] p_buf   Destination buffer, zeroed by the caller.
 * @param[in,out] p_pos   Current bit position; advanced by @p width.
 * @param[in]     value   Field value; bits above @p width are dropped.
 * @param[in]     width   Field width in bits (1 to 32).
 */
void
bitpack_put (uint8_t * p_buf, uint16_t * p_pos, uint32_t value,
             uint8_t width)
{
    uint32_t const masked = value & field_mask(width);

    for (uint8_t bit = width; bit > 0u; bit--)
    {
        uint16_t const pos      = *p_pos;
        uint16_t const byte_idx = (uint16_t)(pos / BITPACK_BYTE_BITS);
        uint8_t  const shift    = (uint8_t)(7u - (pos % BITPACK_BYTE_BITS));

        if (0u != ((masked >> (bit - 1u)) & 1u))
        {
            p_buf[byte_idx] = (uint8_t)(p_buf[byte_idx] | (1u << shift));
        }

        *p_pos = (uint16_t)(pos + 1u);
    }
}

/*!
 * @brief Read a field written by bitpack_put().
 *
 * @param[in] p_buf  Source buffer.
 * @param[in] pos    Bit position of the field's most significant bit.
 * @param[in] width  Field width in bits (1 to 32).
 *
 * @return The unsigned field value.
 */
uint32_t
bitpack_get (uint8_t const * p_buf, uint16_t pos, uint8_t width)
{
    uint32_t value = 0u;

    for (uint8_t bit = 0u; bit < width; bit++)
    {
        uint16_t const pos_at       = (uint16_t)(pos + bit);
        uint16_t const byte_idx = (uint16_t)(pos_at / BITPACK_BYTE_BITS);
        uint8_t  const shift    = (uint8_t)(7u - (pos_at % BITPACK_BYTE_BITS));

        value = (value << 1u) | ((uint32_t)(p_buf[byte_idx] >> shift) & 1u);
    }

    return value;
}

/*!
 * @brief Convert a signed value to a saturated offset-binary field.
 *
 * Only additions and comparisons touch the signed value (Barr C 5.3.b).
 *
 * @param[in]     value        Signed physical value.
 * @param[in]     offset       Offset added before storage.
 * @param[in]     width        Field width in bits.
 * @param[in,out] p_sat_count  Caller's saturation counter, or NULL.
 *
 * @return The unsigned field, clamped to 0 .. 2^width - 1.
 */
uint32_t
bitpack_offset (int32_t value, int32_t offset, uint8_t width,
                uint8_t * p_sat_count)
{
    int32_t const  shifted = value + offset;
    uint32_t const max_u   = field_mask(width);
    uint32_t       field   = 0u;

    if (shifted < 0)
    {
        count_saturation(p_sat_count);
    }
    else if ((uint32_t)shifted > max_u)   /* Cast: shifted is >= 0 here. */
    {
        field = max_u;
        count_saturation(p_sat_count);
    }
    else
    {
        field = (uint32_t)shifted;          /* Cast: range checked above. */
    }

    return field;
}

/*!
 * @brief Clamp an unsigned value to a field width.
 *
 * @return The value, or the field maximum if it did not fit.
 */
uint32_t
bitpack_limit (uint32_t value, uint8_t width, uint8_t * p_sat_count)
{
    uint32_t const max_u = field_mask(width);
    uint32_t       field = value;

    if (value > max_u)
    {
        field = max_u;
        count_saturation(p_sat_count);
    }

    return field;
}

/*!
 * @brief Undo bitpack_offset(): field minus offset.
 */
int32_t
bitpack_signed (uint32_t field, int32_t offset)
{
    /* Cast: every packed field is pos_at most 31 bits wide in this project. */
    return (int32_t)field - offset;
}

/*!
 * @brief Number of bytes needed to hold @p bits bits.
 */
uint16_t
bitpack_bytes (uint16_t bits)
{
    return (uint16_t)((bits + (BITPACK_BYTE_BITS - 1u)) / BITPACK_BYTE_BITS);
}

static uint32_t
field_mask (uint8_t width)
{
    uint32_t mask = 0xFFFFFFFFu;

    if (width < BITPACK_MAX_WIDTH)
    {
        mask = ((uint32_t)1u << width) - 1u;
    }

    return mask;
}

static void
count_saturation (uint8_t * p_sat_count)
{
    if ((NULL != p_sat_count) && (*p_sat_count < BITPACK_SAT_MAX))
    {
        *p_sat_count = (uint8_t)(*p_sat_count + 1u);
    }
}

/*** end of file ***/
