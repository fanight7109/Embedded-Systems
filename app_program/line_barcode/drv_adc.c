/** @file drv_adc.c
 *
 * @brief Single-shot ADC conversions (RP2040 datasheet section 4.9).
 *
 * The kernel's own ADC device driver (device/adc) is left untouched; this
 * driver uses the ADC registers from T3 (line sensors) and from the
 * barcode sampler ISR (bc_sensor.c). A conversion is started, awaited and
 * read with interrupts masked (about 2 us), so the two never interleave.
 *
 * Owner: Buddy 3 (LIM JUN HAO).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "../integration/app_cfg.h"
#include "../integration/board.h"
#include "drv_adc.h"

#define ADC_BASE            (0x4004C000u)
#define ADC_CS              (ADC_BASE + 0x00u)
#define ADC_RESULT          (ADC_BASE + 0x04u)
#define ADC_CS_EN           (0x1u)
#define ADC_CS_START_ONCE   (0x4u)
#define ADC_CS_READY        (0x100u)
#define ADC_CS_ERR          (0x200u)
#define ADC_AINSEL_SHIFT    (12u)
#define ADC_AINSEL_MASK     (0x7u)
#define ADC_RESULT_MASK     (0xFFFu)
#define READY_TMO_US        (20u)          /* 10 x the 2 us conversion.    */
#define NUM_CHANNELS        (4u)

/*!
 * @brief Release the ADC from reset, enable it, set the three IR pins to
 *        analogue.
 */
void
drv_adc_init (void)
{
    board_unreset(RESETS_RESET_ADC);
    board_pin_analog(APP_PIN_IR_L);
    board_pin_analog(APP_PIN_IR_C);
    board_pin_analog(APP_PIN_IR_BC);
    out_w(ADC_CS, ADC_CS_EN);

    while (0u == (in_w(ADC_CS) & ADC_CS_READY))
    {
        /* The ADC reports READY a few cycles after enable. */
    }
}

/*!
 * @brief Convert one channel. Safe from T3 and from the sampler ISR.
 *
 * @param[in]  channel  ADC input 0..3 (GP26..GP29).
 * @param[out] p_value  12-bit result.
 *
 * @return false on a bad channel, a conversion error or a timeout.
 */
bool
drv_adc_read (uint8_t channel, uint16_t * p_value)
{
    bool b_ok = false;

    if (channel < NUM_CHANNELS)
    {
        uint32_t const start  = board_time_us();
        uint32_t       status = 0u;
        UINT           intsts = 0u;

        /* Masked so the sampler ISR cannot change AINSEL mid-conversion. */
        DI(intsts);
        out_w(ADC_CS, ADC_CS_EN | ADC_CS_START_ONCE |
              (((uint32_t)channel & ADC_AINSEL_MASK) << ADC_AINSEL_SHIFT));

        do
        {
            status = in_w(ADC_CS);
        } while ((0u == (status & ADC_CS_READY)) &&
                 ((board_time_us() - start) < READY_TMO_US));

        b_ok = ((0u != (status & ADC_CS_READY)) &&
                (0u == (status & ADC_CS_ERR)));
        *p_value = (uint16_t)(in_w(ADC_RESULT) & ADC_RESULT_MASK);
        EI(intsts);
    }

    return b_ok;
}

/*** end of file ***/
