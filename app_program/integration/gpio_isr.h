/** @file gpio_isr.h
 *
 * @brief The single IO_IRQ_BANK0 handler, gpio_dispatch_isr() (wiring
 *        note W5).
 *
 * The kernel owns the vector table, so the handler is registered with
 * tk_def_int() and enabled with EnableInt(), never through the Pico SDK.
 * Every GPIO edge in the system (encoders, echo) arrives
 * on this one NVIC line and is routed by pin. The GP20/GP21 user buttons
 * are polled instead (button.c). The Pico W
 * radio is polled by the kernel and uses no GPIO interrupt.
 *
 * Owner: Integration.
 */

#ifndef GPIO_ISR_H
#define GPIO_ISR_H

#include <tk/tkernel.h>

ER   gpio_isr_init(void);

#endif /* GPIO_ISR_H */

/*** end of file ***/
