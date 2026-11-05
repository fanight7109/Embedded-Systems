/** @file drv_lsm303.h
 *
 * @brief LSM303DLHC accelerometer driver on I2C1 (GP2/GP3, 400 kHz).
 *
 * The accelerometer runs at 100 Hz, +/- 4 g, high-resolution mode, so the
 * 2 g collision threshold (F08) stays inside the range. The LSM303DLHC
 * accelerometer has no WHO_AM_I register; the self-test instead reads the
 * magnetometer identity registers (IRA/IRB/IRC = "H43") and reads back the
 * accelerometer control register.
 *
 * Owner: Buddy 4 (GO JUN JIE). Only T4 may call these functions.
 */

#ifndef DRV_LSM303_H
#define DRV_LSM303_H

#include <stdint.h>
#include <stdbool.h>

bool drv_lsm303_init(void);
bool drv_lsm303_read_accel(int16_t * p_ax_mg, int16_t * p_ay_mg,
                       int16_t * p_az_mg);

#endif /* DRV_LSM303_H */

/*** end of file ***/
