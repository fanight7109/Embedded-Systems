/** @file drv_motor.c
 *
 * @brief PWM motor driver for the Robo Pico H-bridge (template BSP PWM).
 *
 * Forward drives PWM on pin A with pin B low; reverse drives PWM on pin B
 * with pin A low; both low coasts. Duty is in 0.1 % units (0 .. 1000) and
 * is scaled to the PWM compare value only at the register write (rule R6).
 * Slices 4 and 5 run at 20 kHz from the 125 MHz system clock, divider 1.
 *
 * Owner: Buddy 2 (PEH JUN WEI JOEL).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include "../integration/app_cfg.h"
#include "../integration/board.h"
#include "drv_motor.h"

#define PWM_WRAP            ((BOARD_SYS_CLK_HZ / APP_MOTOR_PWM_HZ) - 1u)
#define PWM_DIV_ONE         (0x10u)        /* DIV.INT = 1, DIV.FRAC = 0.   */
#define SLICE_PAIR_SHIFT    (1u)        /* Two pins per PWM slice.        */
#define SLICE_MASK          (0x7u)
#define SLICE_STRIDE        (0x14u)

static uint32_t slice_of(uint8_t pin);
static void     setup_pwm_pin(uint8_t pin);
static uint32_t duty_to_level(uint16_t duty);
static void     drive_one(uint8_t pin_a, uint8_t pin_b, uint16_t duty,
                          bool b_fwd);

/*!
 * @brief Configure the four motor pins for 20 kHz PWM, outputs low.
 */
void
drv_motor_init (void)
{
    board_unreset(RESETS_RESET_PWM);
    setup_pwm_pin(APP_PIN_M1A);
    setup_pwm_pin(APP_PIN_M1B);
    setup_pwm_pin(APP_PIN_M2A);
    setup_pwm_pin(APP_PIN_M2B);
    drv_motor_stop_now();
}

/*!
 * @brief Apply duty and direction to both motors.
 *
 * @param[in] duty_left   0 .. 1000 (0.1 %), clamped.
 * @param[in] b_left_fwd  true for forward.
 */
void
drv_motor_drive (uint16_t duty_left, bool b_left_fwd, uint16_t duty_right,
                 bool b_right_fwd)
{
    drive_one(APP_PIN_M1A, APP_PIN_M1B, duty_left, b_left_fwd);
    drive_one(APP_PIN_M2A, APP_PIN_M2B, duty_right, b_right_fwd);
}

/*!
 * @brief Zero all four PWM channels immediately (SAFE STOP path, NFR9).
 */
void
drv_motor_stop_now (void)
{
    (void)pwm_set_cc(APP_PIN_M1A, 0u);
    (void)pwm_set_cc(APP_PIN_M1B, 0u);
    (void)pwm_set_cc(APP_PIN_M2A, 0u);
    (void)pwm_set_cc(APP_PIN_M2B, 0u);
}

static uint32_t
slice_of (uint8_t pin)
{
    return ((uint32_t)pin >> SLICE_PAIR_SHIFT) & SLICE_MASK;
}

static void
setup_pwm_pin (uint8_t pin)
{
    /* The BSP has no divider call: write CHx_DIV directly. */
    out_w(PWM_BASE + (slice_of(pin) * SLICE_STRIDE) + PWM_CHx_DIV,
          PWM_DIV_ONE);
    (void)pwm_set_wrap(pin, PWM_WRAP);
    (void)pwm_set_cc(pin, 0u);
    (void)pwm_set_pin(pin);
    (void)pwm_set_enabled(pin, TRUE);
}

static uint32_t
duty_to_level (uint16_t duty)
{
    uint32_t const clamped = (duty > APP_DUTY_MAX) ? APP_DUTY_MAX : duty;

    return (clamped * (PWM_WRAP + 1u)) / APP_DUTY_MAX;
}

static void
drive_one (uint8_t pin_a, uint8_t pin_b, uint16_t duty, bool b_fwd)
{
    uint32_t const level = duty_to_level(duty);

    if (b_fwd)
    {
        (void)pwm_set_cc(pin_b, 0u);
        (void)pwm_set_cc(pin_a, level);
    }
    else
    {
        (void)pwm_set_cc(pin_a, 0u);
        (void)pwm_set_cc(pin_b, level);
    }
}

/*** end of file ***/
