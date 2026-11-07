/** @file board.h
 *
 * @brief RP2040 board access for the application, without the Pico SDK.
 *
 * The micro T-Kernel port owns the clocks, the vector table and the
 * interrupt controller, and does not link the Pico SDK runtime (TEAM.md,
 * template README). This module gives the application the few hardware
 * services it needs, built only on kernel register helpers (in_w, out_w,
 * set_w, clr_w) and the template BSP (gpio_set_pin, pwm_set_*):
 *
 *   - pin direction, level and pad pull-ups;
 *   - the 1 MHz TIMER (microseconds) and a short busy-wait;
 *   - releasing a peripheral from reset;
 *   - IO_BANK0 edge-interrupt enables (the handler is in gpio_isr.c).
 *
 * board_init() also programs the watchdog tick generator that clocks the
 * TIMER. The port's startup code leaves it unset, so after a power-on the
 * TIMER otherwise runs about 43 times slow (port issue PF-001).
 *
 * Owner: Integration (all buddies).
 */

#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include <stdbool.h>

#define BOARD_SYS_CLK_HZ        (125000000u)  /* PLL_SYS set by the kernel. */
#define BOARD_EDGE_FALL         (0x4u)        /* IO_BANK0 EDGE_LOW event.   */
#define BOARD_EDGE_RISE         (0x8u)        /* IO_BANK0 EDGE_HIGH event.  */
#define BOARD_EDGE_BOTH         (BOARD_EDGE_FALL | BOARD_EDGE_RISE)
#define BOARD_IRQ_IO_BANK0      (13u)         /* NVIC line of IO_IRQ_BANK0. */

void     board_init(void);
void     board_pin_input(uint8_t pin, bool b_pull_up);
void     board_pin_output(uint8_t pin, bool b_level);
void     board_pin_put(uint8_t pin, bool b_level);
bool     board_pin_get(uint8_t pin);
void     board_pin_analog(uint8_t pin);
void     board_pin_function(uint8_t pin, uint32_t funcsel, bool b_pull_up);
void     board_pin_irq(uint8_t pin, uint32_t edges);
void     board_pin_irq_off(uint8_t pin, uint32_t edges);
uint32_t board_irq_status(uint8_t bank_reg);
void     board_irq_ack(uint8_t bank_reg, uint32_t bits);
uint32_t board_time_us(void);
uint32_t board_time_ms(void);
void     board_busy_wait_us(uint32_t delay_us);
void     board_unreset(uint32_t reset_mask);

#endif /* BOARD_H */

/*** end of file ***/
