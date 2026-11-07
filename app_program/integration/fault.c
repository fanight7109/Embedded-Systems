/** @file fault.c
 *
 * @brief Fault bitmask with mutex-protected writes (NFR12, Tribunal 1).
 *
 * Owner: Integration (all buddies).
 */

#include <stdint.h>
#include <stdbool.h>
#include "rtos_obj.h"
#include "../evidence/evidence.h"
#include "fault.h"

/* Several writers, one 16-bit aligned load for readers. */
static volatile uint16_t g_fault_mask = 0u;

static void update_mask(uint16_t set_bits, uint16_t clear_bits);

/*!
 * @brief Raise a fault. A critical fault also sets flg_sys FAULT, so T2
 *        zeroes the PWM on its next cycle even if the lock timed out.
 *
 * @param[in] fault_bit  FAULT_Fxx_* bit number.
 */
void
fault_set (uint8_t fault_bit)
{
    if (fault_bit < FAULT_NUM_BITS)
    {
        uint16_t const bit = (uint16_t)(1u << fault_bit);

        evidence_fault(fault_bit);              /* INT-10 stamp. */
        update_mask(bit, 0u);

        if (0u != (bit & FAULT_CRITICAL_MASK))
        {
            rtos_obj_sys_set(RTOS_SYS_FAULT);
        }
    }
}

/*!
 * @brief Clear a non-critical fault on recovery. Critical bits stay set.
 */
void
fault_clear (uint8_t fault_bit)
{
    if (fault_bit < FAULT_NUM_BITS)
    {
        uint16_t const bit = (uint16_t)(1u << fault_bit);

        if (0u == (bit & FAULT_CRITICAL_MASK))
        {
            update_mask(0u, bit);
        }
    }
}

uint16_t
fault_get_mask (void)
{
    return g_fault_mask;
}

bool
fault_is_critical (void)
{
    return (0u != (g_fault_mask & FAULT_CRITICAL_MASK));
}

/*!
 * @brief Lowest set fault as its 1-based ID (F01 = 1), or 0 if none.
 */
uint8_t
fault_first_id (uint16_t mask)
{
    uint8_t fault_id = 0u;

    for (uint8_t bit = 0u; (bit < FAULT_NUM_BITS) && (0u == fault_id); bit++)
    {
        if (0u != (mask & (uint16_t)(1u << bit)))
        {
            fault_id = (uint8_t)(bit + 1u);
        }
    }

    return fault_id;
}

static void
update_mask (uint16_t set_bits, uint16_t clear_bits)
{
    if (rtos_obj_fault_lock())
    {
        uint16_t const mask = g_fault_mask;

        g_fault_mask = (uint16_t)((mask | set_bits) & (uint16_t)~clear_bits);
        rtos_obj_fault_unlock();
    }

    /*
     * WARNING: the lock times out only if the ceiling protocol is broken.
     * The critical path still works because fault_set() sets flg_sys
     * FAULT afterwards regardless.
     */
}

/*** end of file ***/
