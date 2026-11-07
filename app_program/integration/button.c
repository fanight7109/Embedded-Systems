/** @file button.c
 *
 * @brief Debounced press detection for the two Robo Pico user buttons.
 *
 * The buttons are active low with the internal pull-up. A level change is
 * accepted only after APP_BTN_DEBOUNCE_SAMPLES equal samples in a row
 * (30 ms at the 10 ms cyc_ctrl rate), so contact bounce on press and on
 * release never counts as a second press. Polling instead of a pin
 * interrupt keeps release bounce out of the IO_BANK0 handler.
 *
 * Owner: Integration.
 */

#include <stdint.h>
#include <stdbool.h>
#include "app_cfg.h"
#include "board.h"
#include "button.h"

static uint8_t const g_button_pin[BUTTON_COUNT] =
{
    APP_PIN_BTN_START,
    APP_PIN_BTN_STOP
};

/* Written only by button_sample_isr() (cyc_ctrl). */
static bool             gb_stable_down[BUTTON_COUNT] = { false, false };
static uint8_t          g_change_count[BUTTON_COUNT] = { 0u, 0u };
static volatile uint8_t g_press_count[BUTTON_COUNT]  = { 0u, 0u };

/* Written only by the one task that consumes each button. */
static uint8_t          g_taken_count[BUTTON_COUNT]  = { 0u, 0u };

/*!
 * @brief Configure both pins as inputs with pull-ups. Called once from
 *        usermain() before the cyclic handlers start.
 */
void
button_init (void)
{
    for (uint8_t idx = 0u; idx < BUTTON_COUNT; idx++)
    {
        board_pin_input(g_button_pin[idx], true);
    }
}

/*!
 * @brief Sample and debounce both buttons. Runs every 10 ms in cyc_ctrl,
 *        so it only reads two GPIO bits and updates counters.
 */
void
button_sample_isr (void)
{
    for (uint8_t idx = 0u; idx < BUTTON_COUNT; idx++)
    {
        bool const b_down = !board_pin_get(g_button_pin[idx]);

        if (b_down == gb_stable_down[idx])
        {
            g_change_count[idx] = 0u;
        }
        else
        {
            g_change_count[idx]++;

            if (g_change_count[idx] >= APP_BTN_DEBOUNCE_SAMPLES)
            {
                gb_stable_down[idx] = b_down;
                g_change_count[idx] = 0u;

                if (b_down)
                {
                    /* Wraps at 256; button_take() compares for equality. */
                    g_press_count[idx]++;
                }
            }
        }
    }
}

/*!
 * @brief Debounced presses so far (wraps at 256). Read-only; used by the
 *        evidence build to timestamp a press in cyc_ctrl (INT-10).
 */
uint8_t
button_presses (uint8_t button_id)
{
    return (button_id < BUTTON_COUNT) ? g_press_count[button_id] : 0u;
}

/*!
 * @brief Consume one pending press of a button.
 *
 * @param[in] button_id  BUTTON_START or BUTTON_STOP.
 *
 * @return true once for every debounced press not yet taken.
 */
bool
button_take (uint8_t button_id)
{
    bool b_pressed = false;

    if (button_id < BUTTON_COUNT)
    {
        if (g_press_count[button_id] != g_taken_count[button_id])
        {
            g_taken_count[button_id]++;
            b_pressed = true;
        }
    }

    return b_pressed;
}

/*** end of file ***/
