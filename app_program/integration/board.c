/** @file board.c
 *
 * @brief RP2040 pins, pads, TIMER and resets through kernel register
 *        helpers (see board.h).
 *
 * Register addresses not defined by the kernel's sysdef.h come from the
 * RP2040 datasheet (sections 2.15 watchdog, 2.19 IO_BANK0, 4.6 TIMER).
 *
 * Owner: Integration (all buddies).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include "board.h"

#define WATCHDOG_TICK       (0x4005802Cu)
#define WATCHDOG_TICK_EN    (0x200u)          /* bit 9: ENABLE            */
#define IO_INTR0            (IO_BANK0_BASE + 0x0F0u)
#define IO_PROC0_INTE0      (IO_BANK0_BASE + 0x100u)
#define IO_PROC0_INTS0      (IO_BANK0_BASE + 0x120u)
#define PINS_PER_REG        (8u)
#define BITS_PER_PIN        (4u)
#define EVENT_MASK          (0xFu)
#define US_PER_MS           (1000u)
#define HIGH_WORD_SHIFT     (32u)

/*!
 * @brief One-time board set-up, called first in usermain().
 *
 * Programs the 1 us TIMER tick from the 12 MHz crystal (PF-001 fix). The
 * write is idempotent, so a corrected kernel startup is not disturbed.
 */
void
board_init (void)
{
    out_w(WATCHDOG_TICK, WATCHDOG_TICK_EN | (uint32_t)XOSC_MHz);
}

/*!
 * @brief Digital input, optionally with the pad pull-up.
 */
void
board_pin_input (uint8_t pin, bool b_pull_up)
{
    (void)gpio_set_pin(pin, GPIO_MODE_IN);
    clr_w(GPIO(pin), GPIO_PDE);

    if (b_pull_up)
    {
        set_w(GPIO(pin), GPIO_PUE);
    }
    else
    {
        clr_w(GPIO(pin), GPIO_PUE);
    }
}

/*!
 * @brief Digital output, driven to @p b_level.
 */
void
board_pin_output (uint8_t pin, bool b_level)
{
    (void)gpio_set_val(pin, b_level ? 1u : 0u);
    (void)gpio_set_pin(pin, GPIO_MODE_OUT);
}

void
board_pin_put (uint8_t pin, bool b_level)
{
    (void)gpio_set_val(pin, b_level ? 1u : 0u);
}

bool
board_pin_get (uint8_t pin)
{
    return (0u != gpio_get_val(pin));
}

/*!
 * @brief Analogue input (ADC): digital input and pulls off, no function.
 */
void
board_pin_analog (uint8_t pin)
{
    clr_w(GPIO(pin), GPIO_IE | GPIO_PUE | GPIO_PDE);
    set_w(GPIO(pin), GPIO_OD);
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_NULL);
}

/*!
 * @brief Hand a pin to a peripheral function (I2C, PWM, ...).
 */
void
board_pin_function (uint8_t pin, uint32_t funcsel, bool b_pull_up)
{
    clr_w(GPIO(pin), GPIO_OD | GPIO_PDE);
    set_w(GPIO(pin), GPIO_IE);

    if (b_pull_up)
    {
        set_w(GPIO(pin), GPIO_PUE);
    }
    else
    {
        clr_w(GPIO(pin), GPIO_PUE);
    }

    out_w(GPIO_CTRL(pin), funcsel);
}

/*!
 * @brief Enable edge interrupts for one pin on core 0 (PROC0).
 *
 * Called only during start-up, before the interrupt is enabled. Stale
 * edges are cleared first so the first interrupt is a real one.
 *
 * @param[in] edges  BOARD_EDGE_RISE and/or BOARD_EDGE_FALL.
 */
void
board_pin_irq (uint8_t pin, uint32_t edges)
{
    uint32_t const reg   = (uint32_t)pin / PINS_PER_REG;
    uint32_t const shift = ((uint32_t)pin % PINS_PER_REG) * BITS_PER_PIN;
    uint32_t const bits  = (edges & BOARD_EDGE_BOTH) << shift;

    out_w(IO_INTR0 + (reg * 4u), bits);
    set_w(IO_PROC0_INTE0 + (reg * 4u), bits);
}

/*!
 * @brief Disable edge interrupts for one pin on core 0 (evidence bench).
 */
void
board_pin_irq_off (uint8_t pin, uint32_t edges)
{
    uint32_t const reg   = (uint32_t)pin / PINS_PER_REG;
    uint32_t const shift = ((uint32_t)pin % PINS_PER_REG) * BITS_PER_PIN;
    uint32_t const bits  = (edges & BOARD_EDGE_BOTH) << shift;

    clr_w(IO_PROC0_INTE0 + (reg * 4u), bits);
    out_w(IO_INTR0 + (reg * 4u), bits);
}

/*!
 * @brief Pending, enabled interrupt bits of one IO_BANK0 register (0..3).
 */
uint32_t
board_irq_status (uint8_t bank_reg)
{
    return in_w(IO_PROC0_INTS0 + ((uint32_t)bank_reg * 4u));
}

/*!
 * @brief Clear latched edge events (write-1-to-clear).
 */
void
board_irq_ack (uint8_t bank_reg, uint32_t bits)
{
    out_w(IO_INTR0 + ((uint32_t)bank_reg * 4u), bits);
}

/*!
 * @brief Low 32 bits of the 1 MHz TIMER (wraps every 71.6 minutes).
 *
 * Safe from tasks and interrupts: one read of the raw low word.
 */
uint32_t
board_time_us (void)
{
    return in_w(TIMER_TIMERAWL);
}

/*!
 * @brief Milliseconds since boot from the 64-bit TIMER (wraps after
 *        49.7 days).
 */
uint32_t
board_time_ms (void)
{
    uint32_t high = 0u;
    uint32_t low  = 0u;
    uint32_t check = 0u;

    do
    {
        high  = in_w(TIMER_TIMERAWH);
        low   = in_w(TIMER_TIMERAWL);
        check = in_w(TIMER_TIMERAWH);
    } while (high != check);

    return (uint32_t)(((((uint64_t)high) << HIGH_WORD_SHIFT) | low) /
                      US_PER_MS);
}

/*!
 * @brief Spin for a few microseconds (HC-SR04 trigger pulse only).
 *
 * WARNING: burns CPU; never use for delays longer than about 20 us.
 */
void
board_busy_wait_us (uint32_t delay_us)
{
    uint32_t const start = board_time_us();

    while ((board_time_us() - start) < delay_us)
    {
        /* Busy-wait: the delay is shorter than one kernel tick. */
    }
}

/*!
 * @brief Take a peripheral out of reset, only if it is still held.
 *
 * Never re-resets a running block, so it is safe if the kernel already
 * released it.
 */
void
board_unreset (uint32_t reset_mask)
{
    if (0u != (in_w(RESETS_RESET) & reset_mask))
    {
        clr_w(RESETS_RESET, reset_mask);

        while (reset_mask != (in_w(RESETS_RESET_DONE) & reset_mask))
        {
            /* Wait for the block to leave reset (a few cycles). */
        }
    }
}

/*** end of file ***/
