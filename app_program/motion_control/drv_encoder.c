/** @file drv_encoder.c
 *
 * @brief Count-only encoder ISR and unit conversion (Section 4, NFR10).
 *
 * Both edges of channel A are counted. With quadrature encoders the
 * direction comes from channel B; with single-channel encoders (W3) it
 * comes from the commanded motor direction set by T2.
 *
 * Owner: Buddy 2 (PEH JUN WEI JOEL).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "../integration/board.h"
#include "../integration/app_cfg.h"
#include "drv_encoder.h"

#define LEFT_SHIFT      (16u)
#define HALF_MASK       (0xFFFFu)
#define UM_PER_MM       (1000)
#define UM_PER_EDGE     ((int32_t)(APP_WHEEL_CIRC_UM / APP_ENC_EDGES_PER_REV))

/* Written only by drv_encoder_edge_isr(); see encoder.h. */
static volatile uint32_t g_enc_counts = 0u;

/* Direction hints, written only by T2 (single writer), read by the ISR. */
static volatile bool gb_left_fwd  = true;
static volatile bool gb_right_fwd = true;

static void setup_pin(uint8_t gpio);
static bool edge_is_forward(uint8_t pin_a, uint8_t pin_b, uint32_t events,
                            bool b_invert, bool b_hint);

/*!
 * @brief Configure the encoder pins and enable both edges on channel A.
 */
void
drv_encoder_init (void)
{
    setup_pin(APP_PIN_ENC_L_A);
    board_pin_irq(APP_PIN_ENC_L_A, BOARD_EDGE_BOTH);

#if (1 == APP_ENC_QUADRATURE)
    setup_pin(APP_PIN_ENC_L_B);
#endif

#if (1 == TM_CONSOLE_UART)
    /*
     * WARNING: CONSOLE=uart and CRASH_REPORT=1 give GP0/GP1 to UART0, so the
     * right encoder is left unconfigured. Expect F01 if the motors run.
     */
    tm_printf((UB *)"drv_encoder: GP0/GP1 used by UART0, right encoder off\n");
#else
    setup_pin(APP_PIN_ENC_R_A);
    board_pin_irq(APP_PIN_ENC_R_A, BOARD_EDGE_BOTH);

#if (1 == APP_ENC_QUADRATURE)
    setup_pin(APP_PIN_ENC_R_B);
#endif
#endif
}

/*!
 * @brief One aligned 32-bit load of both counters.
 */
uint32_t
drv_encoder_snapshot (void)
{
    return g_enc_counts;
}

uint16_t
drv_encoder_left (uint32_t snapshot)
{
    return (uint16_t)((snapshot >> LEFT_SHIFT) & HALF_MASK);
}

uint16_t
drv_encoder_right (uint32_t snapshot)
{
    return (uint16_t)(snapshot & HALF_MASK);
}

/*!
 * @brief Signed edge delta between two wrapping counts.
 *
 * Valid while fewer than 32768 edges pass between reads; the slowest
 * reader sees at most about 300 edges (Section 1.5).
 */
int16_t
drv_encoder_delta (uint16_t now, uint16_t before)
{
    uint16_t const diff = (uint16_t)(now - before);

    /* Cast: two's-complement reinterpretation of the modular difference. */
    return (int16_t)diff;
}

int32_t
drv_encoder_edges_to_mm (int32_t edges)
{
    return (edges * UM_PER_EDGE) / UM_PER_MM;
}

int32_t
drv_encoder_mm_to_edges (int32_t distance_mm)
{
    return ((distance_mm * UM_PER_MM) + (UM_PER_EDGE / 2)) / UM_PER_EDGE;
}

/*!
 * @brief Commanded wheel directions, used with single-channel encoders.
 */
void
drv_encoder_set_direction_hint (bool b_left_fwd, bool b_right_fwd)
{
    gb_left_fwd  = b_left_fwd;
    gb_right_fwd = b_right_fwd;
}

static void
setup_pin (uint8_t gpio)
{
    board_pin_input(gpio, true);
}

static bool
edge_is_forward (uint8_t pin_a, uint8_t pin_b, uint32_t events,
                 bool b_invert, bool b_hint)
{
    bool b_fwd = b_hint;

#if (1 == APP_ENC_QUADRATURE)
    bool const b_a = (0u != (events & BOARD_EDGE_RISE));
    bool const b_b = board_pin_get(pin_b);

    (void)pin_a;
    b_fwd = (b_a != b_b) ? (!b_invert) : b_invert;
#else
    (void)pin_a;
    (void)pin_b;
    (void)events;
    (void)b_invert;
#endif

    return b_fwd;
}

/*!
 * @brief Encoder branch of gpio_dispatch_isr(): count one edge.
 *
 * WARNING: interrupt context. Single writer of g_enc_counts; the whole
 * word is rebuilt and stored once, so readers never see a torn pair.
 */
void
drv_encoder_edge_isr (uint8_t gpio, uint32_t events)
{
    uint32_t const snap  = g_enc_counts;
    uint16_t       left  = drv_encoder_left(snap);
    uint16_t       right = drv_encoder_right(snap);

    if (APP_PIN_ENC_L_A == gpio)
    {
        bool const b_fwd = edge_is_forward(APP_PIN_ENC_L_A, APP_PIN_ENC_L_B,
                                           events, (1 == APP_ENC_L_INVERT),
                                           gb_left_fwd);

        left = b_fwd ? (uint16_t)(left + 1u) : (uint16_t)(left - 1u);
    }
    else
    {
        bool const b_fwd = edge_is_forward(APP_PIN_ENC_R_A, APP_PIN_ENC_R_B,
                                           events, (1 == APP_ENC_R_INVERT),
                                           gb_right_fwd);

        right = b_fwd ? (uint16_t)(right + 1u) : (uint16_t)(right - 1u);
    }

    g_enc_counts = ((uint32_t)left << LEFT_SHIFT) | (uint32_t)right;
}

/*** end of file ***/
