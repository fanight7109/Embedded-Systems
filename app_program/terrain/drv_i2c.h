/** @file drv_i2c.h
 *
 * @brief Polled I2C1 master (GP2 SDA, GP3 SCL, 400 kHz) for the IMU.
 *
 * The kernel's own I2C driver is disabled in config/config_device.h and
 * is configured for other pins, so subsystem 4 drives I2C1 directly. Every
 * wait is bounded by a timeout, so a stuck bus returns an error instead
 * of hanging T4 (F09 handling). Only T4 uses I2C1 (TEAM.md section 3).
 *
 * Owner: Buddy 4 (GO JUN JIE).
 */

#ifndef DRV_I2C_H
#define DRV_I2C_H

#include <stdint.h>
#include <stdbool.h>

void drv_i2c_init(void);
bool drv_i2c_write(uint8_t addr, uint8_t const * p_src, uint8_t len,
                   bool b_nostop);
bool drv_i2c_read(uint8_t addr, uint8_t * p_dst, uint8_t len);

#endif /* DRV_I2C_H */

/*** end of file ***/
