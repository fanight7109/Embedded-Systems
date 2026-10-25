/** @file drv_motor.h
 *
 * @brief Robo Pico dual motor driver (M1 = left, M2 = right).
 *
 * Only T2 may call these functions: T2 is the sole motor owner.
 * Motors use PWM slices 4 and 5 at 20 kHz; the scan servo uses slice 6 at
 * 50 Hz, so the frequencies never interfere (Tribunal 1, PWM slice check).
 *
 * Owner: Buddy 2 (PEH JUN WEI JOEL).
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

void drv_motor_init(void);
void drv_motor_drive(uint16_t duty_left, bool b_left_fwd, uint16_t duty_right,
                 bool b_right_fwd);
void drv_motor_stop_now(void);

#endif /* DRV_MOTOR_H */

/*** end of file ***/
