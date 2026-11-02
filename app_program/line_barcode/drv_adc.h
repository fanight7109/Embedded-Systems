/** @file drv_adc.h
 *
 * @brief Polled RP2040 ADC for the three IR sensors (ADC0, ADC1, ADC2).
 *
 * One conversion takes 2 us (96 cycles of the 48 MHz ADC clock), so the
 * callers poll for READY instead of using an interrupt. Two users only
 * (TEAM.md section 3): T3 reads the line sensors (ADC0, ADC1) and the
 * barcode sampler ISR reads ADC2. Each conversion runs with interrupts
 * masked, so the ISR can never switch the channel under T3.
 *
 * Owner: Buddy 3 (LIM JUN HAO).
 */

#ifndef DRV_ADC_H
#define DRV_ADC_H

#include <stdint.h>
#include <stdbool.h>

void drv_adc_init(void);
bool drv_adc_read(uint8_t channel, uint16_t * p_value);

#endif /* DRV_ADC_H */

/*** end of file ***/
