/** @file drv_servo.h
 *
 * @brief Scan servo on GP12 (PWM slice 6, 50 Hz).
 *
 * Angle = 30 + 5 x index degrees, index 0 .. 24 (5 bits). Index 12 points
 * straight ahead. Only T5 may move the servo.
 *
 * Owner: Buddy 5 (OON JUN KANG SHAWN).
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <stdint.h>

#define DRV_SERVO_NUM_IDX           (25u)

void     drv_servo_init(void);
void     drv_servo_set_index(uint8_t idx);
uint16_t drv_servo_settle_ms(uint8_t from_idx, uint8_t to_idx);

#endif /* DRV_SERVO_H */

/*** end of file ***/
