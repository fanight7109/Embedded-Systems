/** @file drv_lsm303.c
 *
 * @brief LSM303DLHC register access with bounded I2C timeouts.
 *
 * Owner: Buddy 4 (GO JUN JIE).
 */

#include <stdint.h>
#include <stdbool.h>
#include "../integration/app_cfg.h"
#include "drv_i2c.h"
#include "drv_lsm303.h"

#define ACC_ADDR            (0x19u)
#define MAG_ADDR            (0x1Eu)
#define REG_CTRL1_A         (0x20u)
#define REG_CTRL4_A         (0x23u)
#define REG_OUT_X_L_A       (0x28u)
#define AUTO_INCREMENT      (0x80u)
#define REG_CRA_M           (0x00u)
#define REG_MR_M            (0x02u)
#define REG_IRA_M           (0x0Au)
#define CTRL1_100HZ_XYZ     (0x57u)   /* ODR 100 Hz, normal, X/Y/Z on.   */
#define CTRL4_BDU_4G_HR     (0x98u)   /* BDU, +/- 4 g, high resolution.  */
#define CRA_30HZ            (0x14u)
#define MR_CONTINUOUS       (0x00u)
#define ID_A                (0x48u)   /* 'H' */
#define ID_B                (0x34u)   /* '4' */
#define ID_C                (0x33u)   /* '3' */
#define RAW_TO_12BIT        (16)      /* Left-justified 12-bit data.     */
#define MG_PER_LSB_4G       (2)
#define AXIS_BYTES          (6u)

static bool write_reg(uint8_t addr, uint8_t reg, uint8_t value);
static bool read_regs(uint8_t addr, uint8_t reg, uint8_t * p_dst,
                      uint8_t len);
static int16_t to_mg(uint8_t low, uint8_t high);

/*!
 * @brief Initialise I2C1 and both sensor blocks, then self-test.
 *
 * @return false on any I2C error or identity mismatch (F04 / F09).
 */
bool
drv_lsm303_init (void)
{
    uint8_t ident[3] = { 0u, 0u, 0u };
    uint8_t ctrl1    = 0u;
    bool    b_ok     = false;

    drv_i2c_init();

    b_ok = write_reg(ACC_ADDR, REG_CTRL1_A, CTRL1_100HZ_XYZ) &&
           write_reg(ACC_ADDR, REG_CTRL4_A, CTRL4_BDU_4G_HR) &&
           write_reg(MAG_ADDR, REG_CRA_M, CRA_30HZ) &&
           write_reg(MAG_ADDR, REG_MR_M, MR_CONTINUOUS) &&
           read_regs(MAG_ADDR, REG_IRA_M, ident, 3u) &&
           read_regs(ACC_ADDR, REG_CTRL1_A, &ctrl1, 1u);

    return b_ok && (ID_A == ident[0]) && (ID_B == ident[1]) &&
           (ID_C == ident[2]) && (CTRL1_100HZ_XYZ == ctrl1);
}

/*!
 * @brief Burst-read X/Y/Z acceleration in milli-g, axis signs applied.
 */
bool
drv_lsm303_read_accel (int16_t * p_ax_mg, int16_t * p_ay_mg, int16_t * p_az_mg)
{
    uint8_t raw[AXIS_BYTES] = { 0u };
    bool    b_ok = read_regs(ACC_ADDR, (uint8_t)(REG_OUT_X_L_A |
                                                 AUTO_INCREMENT),
                             raw, AXIS_BYTES);

    if (b_ok)
    {
        *p_ax_mg = (int16_t)(APP_IMU_AXIS_X_SIGN * to_mg(raw[0], raw[1]));
        *p_ay_mg = to_mg(raw[2], raw[3]);
        *p_az_mg = (int16_t)(APP_IMU_AXIS_Z_SIGN * to_mg(raw[4], raw[5]));
    }

    return b_ok;
}

static bool
write_reg (uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t const frame[2] = { reg, value };

    return drv_i2c_write(addr, frame, 2u, false);
}

static bool
read_regs (uint8_t addr, uint8_t reg, uint8_t * p_dst, uint8_t len)
{
    return drv_i2c_write(addr, &reg, 1u, true) &&
           drv_i2c_read(addr, p_dst, len);
}

/*!
 * @brief Combine bytes (unsigned operations), then reinterpret as signed
 *        and scale arithmetically (Barr C 5.3: no shifts on signed data).
 */
static int16_t
to_mg (uint8_t low, uint8_t high)
{
    uint16_t const word = (uint16_t)(((uint16_t)high << 8u) | low);

    /* Cast: two's-complement sensor word. */
    int16_t const raw = (int16_t)word;

    return (int16_t)((raw / RAW_TO_12BIT) * MG_PER_LSB_4G);
}

/*** end of file ***/
