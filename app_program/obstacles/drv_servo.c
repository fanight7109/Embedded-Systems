/** @file drv_servo.c
 *
 * @brief 50 Hz servo PWM on GP12 (slice 6): 1 us per count, 20000-count
 *        period, using the template BSP PWM helpers.
 *
 * Owner: Buddy 5 (OON JUN KANG SHAWN).
 */

#include <stdint.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include "../integration/app_cfg.h"
#include "../integration/board.h"
#include "drv_servo.h"

#define PERIOD_COUNTS       (20000u)    /* 20 ms at 1 MHz.                */
#define TICK_HZ             (1000000u)
#define DIV_INT_SHIFT       (4u)        /* CHx_DIV: INT in bits 11..4.    */
#define SLICE_PAIR_SHIFT    (1u)        /* Two pins per PWM slice.        */
#define SLICE_MASK          (0x7u)
#define SLICE_STRIDE        (0x14u)
#define MIN_ANGLE_DEG       (30u)
#define STEP_DEG            (5u)
#define FULL_SCALE_DEG      (180u)
#define SETTLE_BASE_MS      (40u)       /* (CAL) SG90 at 5 V.             */
#define SETTLE_PER_STEP_MS  (8u)

static uint32_t slice_of(uint8_t pin);

/*!
 * @brief Configure GP12 for 50 Hz and point straight ahead.
 */
void
drv_servo_init (void)
{
    uint32_t const div = BOARD_SYS_CLK_HZ / TICK_HZ;   /* 125 */

    board_unreset(RESETS_RESET_PWM);
    out_w(PWM_BASE + (slice_of(APP_PIN_SERVO) * SLICE_STRIDE) + PWM_CHx_DIV,
          div << DIV_INT_SHIFT);
    (void)pwm_set_wrap(APP_PIN_SERVO, PERIOD_COUNTS - 1u);
    (void)pwm_set_pin(APP_PIN_SERVO);
    drv_servo_set_index(APP_SERVO_FWD_IDX);
    (void)pwm_set_enabled(APP_PIN_SERVO, TRUE);
}

/*!
 * @brief Move to 30 + 5 x idx degrees (idx clamped to 0 .. 24).
 */
void
drv_servo_set_index (uint8_t idx)
{
    uint32_t const clamped = (idx >= DRV_SERVO_NUM_IDX) ?
                             (DRV_SERVO_NUM_IDX - 1u) : idx;
    uint32_t const angle   = MIN_ANGLE_DEG + (STEP_DEG * clamped);
    uint32_t const pulse   = APP_SERVO_MIN_US +
                             (((APP_SERVO_MAX_US - APP_SERVO_MIN_US) *
                               angle) / FULL_SCALE_DEG);

    (void)pwm_set_cc(APP_PIN_SERVO, pulse);
}

/*!
 * @brief Time to wait after a move before trusting a ping.
 */
uint16_t
drv_servo_settle_ms (uint8_t from_idx, uint8_t to_idx)
{
    uint8_t const steps = (from_idx > to_idx) ? (uint8_t)(from_idx - to_idx)
                                              : (uint8_t)(to_idx - from_idx);

    return (uint16_t)(SETTLE_BASE_MS + (SETTLE_PER_STEP_MS * steps));
}

static uint32_t
slice_of (uint8_t pin)
{
    return ((uint32_t)pin >> SLICE_PAIR_SHIFT) & SLICE_MASK;
}

/*** end of file ***/
