/** @file gpio_isr.c
 *
 * @brief Shared GPIO interrupt dispatcher, registered with the kernel.
 *
 * Each branch only counts, timestamps or sets an event flag (NFR10): no
 * kernel waits, no floating point, a few microseconds at most. Edge events
 * are acknowledged before dispatch, so an edge arriving during the handler
 * raises the interrupt again instead of being lost.
 *
 * GP20 is no longer a hardware E-STOP: GP20 is now the START button and
 * GP21 the STOP button, both polled and debounced in button.c. E-STOP
 * remains available as the MQTT ESTOP command (NFR9 path unchanged).
 *
 * Owner: Integration.
 */

#include <stdint.h>
#include <tk/tkernel.h>
#include "app_cfg.h"
#include "board.h"
#include "../motion_control/drv_encoder.h"
#include "../obstacles/drv_ultrasonic.h"
#include "../evidence/ev_bench.h"
#include "gpio_isr.h"

#define BANK_REGS           (4u)    /* IO_BANK0 interrupt registers.      */
#define PINS_PER_REG        (8u)
#define BITS_PER_PIN        (4u)
#define EVENT_MASK          (0xFu)
#define GPIO_INT_LEVEL      (2)     /* Same level as the kernel drivers.  */

static void gpio_dispatch_isr(UINT intno);
static void route(uint8_t pin, uint32_t events);

/*!
 * @brief Register and enable the IO_BANK0 handler. Called once from
 *        usermain() before any subsystem enables a pin interrupt.
 *
 * @return E_OK or the tk_def_int() error.
 */
ER
gpio_isr_init (void)
{
    T_DINT dint = { .intatr = TA_HLNG, .inthdr = NULL };
    ER     err  = E_OK;

    /* Cast: the kernel stores handlers as the generic FP type. */
    dint.inthdr = (FP)gpio_dispatch_isr;
    err = tk_def_int(BOARD_IRQ_IO_BANK0, &dint);

    if (E_OK == err)
    {
        EnableInt(BOARD_IRQ_IO_BANK0, GPIO_INT_LEVEL);
    }

    return err;
}

/*!
 * @brief The one IO_BANK0 handler: acknowledge, then route each pin.
 */
static void
gpio_dispatch_isr (UINT intno)
{
    (void)intno;

    for (uint8_t reg = 0u; reg < BANK_REGS; reg++)
    {
        uint32_t const status = board_irq_status(reg);

        if (0u != status)
        {
            board_irq_ack(reg, status);

            for (uint8_t slot = 0u; slot < PINS_PER_REG; slot++)
            {
                uint32_t const events = (status >> (slot * BITS_PER_PIN)) &
                                        EVENT_MASK;

                if (0u != events)
                {
                    route((uint8_t)((reg * PINS_PER_REG) + slot), events);
                }
            }
        }
    }
}

static void
route (uint8_t pin, uint32_t events)
{
    switch (pin)
    {
        case APP_PIN_ENC_L_A:
        case APP_PIN_ENC_R_A:
            drv_encoder_edge_isr(pin, events);
        break;

        case APP_PIN_US_ECHO:
            drv_ultrasonic_echo_isr(events);
        break;

#if (0 != APP_EVIDENCE)
        case APP_PIN_SERVO:
            /* INT-14 bench only: GP12 edges are enabled nowhere else. */
            ev_bench_edge_isr(events);
        break;
#endif

        default:
            /* Unexpected pin: ignore (Barr C 6.5.d stub behaviour). */
        break;
    }
}

/*** end of file ***/
