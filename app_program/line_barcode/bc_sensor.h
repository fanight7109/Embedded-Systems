/** @file bc_sensor.h
 *
 * @brief Barcode sensor input for T3: AO sampler ISR, edge ring, capture
 *        driver.
 *
 * The barcode IR sensor sits over the barcode lane (20 mm beside the
 * 18 mm line) and its analogue output (AO) drives GP28 (ADC2). The
 * module's DO comparator proved unreliable (28 Oct), so a TIMER alarm 0
 * ISR samples AO every APP_BC_SAMPLE_US and slices it in software, with
 * thresholds that follow the paper (barcode_slice()). Every change of
 * colour is timestamped with both the time and the encoder distance
 * (left + right edge counts) since the previous one. T3 drains the ring
 * once per cycle into barcode.c and reports decoded codes to T6 as
 * MSG_EV_BARCODE (unchanged interface).
 *
 * Trace: the ISR also keeps the last APP_BC_TRACE_BINS distance bins of
 * blackness and colour. T3 copies them out when a capture ends (or on
 * request); T1 publishes the copy with bc_sensor_trace_get() and hands it
 * back with bc_sensor_trace_done().
 *
 * Owner: Buddy 3 (LIM JUN HAO).
 */

#ifndef BC_SENSOR_H
#define BC_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

/*
 * Diagnostic values for the MQTT DIAG command (same numbers as
 * MQTT_DIAG_BC_* in mqtt_fmt.h). All are blackness in ADC counts, 0..4095,
 * higher = blacker (the raw AO reading while APP_IR_BLACK_IS_HIGH is 1).
 */
#define BC_DIAG_RAW             (0u)   /* Live AO sample (0.5 ms old).     */
#define BC_DIAG_CAL_WHITE       (1u)   /* Lowest in the last CALIBRATE.    */
#define BC_DIAG_CAL_BLACK       (2u)   /* Highest in the last CALIBRATE.   */
#define BC_DIAG_BLACK_ON        (3u)   /* White -> black threshold now.    */
#define BC_DIAG_WHITE_ON        (4u)   /* Black -> white threshold now.    */
#define BC_DIAG_TRACE           (5u)   /* Request a trace; returns bins.   */
#define BC_DIAG_COUNT           (6u)

/* Snapshot for the T3 telemetry frame. */
typedef struct
{
    uint8_t state;      /* BARCODE_ST_* (2 bits).                         */
    uint8_t last_cmd;   /* MSG_BC_* of the last decoded code (2 bits).    */
    uint8_t last_dir;   /* 1 = read in reverse.                           */
    uint8_t b_new;      /* 1 = decoded since the previous report.         */
    uint8_t rejects;    /* Saturates at 15.                               */
} bc_sensor_report_t;

ER      bc_sensor_init(void);
void    bc_sensor_poll(void);
void    bc_sensor_set_calibration(bool b_active);
bool    bc_sensor_is_black(void);
uint8_t bc_sensor_rejects(void);
void    bc_sensor_report(bc_sensor_report_t * p_report);
uint16_t bc_sensor_diag(uint8_t item);
uint16_t const * bc_sensor_trace_get(uint8_t * p_trace_id);
void    bc_sensor_trace_done(void);
uint16_t bc_sensor_trace_bin_um(void);

#endif /* BC_SENSOR_H */

/*** end of file ***/
