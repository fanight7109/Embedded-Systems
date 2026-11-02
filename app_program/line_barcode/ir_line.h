/** @file ir_line.h
 *
 * @brief T3 Line + Barcode: IR line tracking, junctions, barcode capture.
 *
 * Sensors: IR_L and IR_C analogue outputs straddle the 18 mm line (ADC0,
 * ADC1). IR_R is mounted over the barcode lane (20 mm beside the line) and
 * its analogue output is on GP28 (ADC2), sampled every APP_BC_SAMPLE_US by
 * the barcode sampler ISR (bc_sensor.c, NFR8). DO is no longer used.
 *
 * Region A (line, 5 states) follows Design Review 6.1; Region B (barcode,
 * IDLE / CAPTURE / HOLD) lives in bc_sensor.c and barcode.c. T3 sends
 * navigation events only; it never writes PWM.
 *
 * Owner: Buddy 3 (LIM JUN HAO).
 */

#ifndef IR_LINE_H
#define IR_LINE_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

/* Region A line states (5 values, 3 bits). */
#define IR_LINE_ST_TRACKING     (0u)
#define IR_LINE_ST_JUNCTION     (1u)
#define IR_LINE_ST_LOSING       (2u)
#define IR_LINE_ST_LOST         (3u)
#define IR_LINE_ST_CALIBRATE    (4u)

/* Calibration result (single writer T3). */
#define IR_CAL_PENDING          (0u)
#define IR_CAL_PASS             (1u)
#define IR_CAL_FAIL             (2u)

/* Sensor index for ir_line_cal_contrast(). */
#define IR_SENSOR_L             (0u)
#define IR_SENSOR_C             (1u)

void    ir_line_init(void);
void    ir_line_task(INT stacd, void * p_exinf);
void    ir_line_set_calibration(bool b_active);
uint8_t ir_line_cal_result(void);
uint16_t ir_line_cal_contrast(uint8_t sensor);
bool    ir_line_edge_aligned(void);
uint8_t ir_line_rejects(void);

#endif /* IR_LINE_H */

/*** end of file ***/
