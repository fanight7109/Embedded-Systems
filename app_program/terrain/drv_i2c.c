/** @file drv_i2c.c
 *
 * @brief DW_apb_i2c master on I2C1, polled (RP2040 datasheet 4.3).
 *
 * Timing for 400 kHz from the 125 MHz system clock follows the Pico SDK's
 * i2c_set_baudrate(): period 313 clocks split 3:2 low:high, spike filter
 * lcnt/16, SDA hold 300 ns.
 *
 * Owner: Buddy 4 (GO JUN JIE).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "../integration/app_cfg.h"
#include "../integration/board.h"
#include "drv_i2c.h"

#define I2C1_BASE           (0x40048000u)
#define IC_CON              (I2C1_BASE + 0x00u)
#define IC_TAR              (I2C1_BASE + 0x04u)
#define IC_DATA_CMD         (I2C1_BASE + 0x10u)
#define IC_FS_SCL_HCNT      (I2C1_BASE + 0x1Cu)
#define IC_FS_SCL_LCNT      (I2C1_BASE + 0x20u)
#define IC_RAW_INTR_STAT    (I2C1_BASE + 0x34u)
#define IC_TX_TL            (I2C1_BASE + 0x3Cu)
#define IC_CLR_TX_ABRT      (I2C1_BASE + 0x54u)
#define IC_CLR_STOP_DET     (I2C1_BASE + 0x60u)
#define IC_ENABLE           (I2C1_BASE + 0x6Cu)
#define IC_TXFLR            (I2C1_BASE + 0x74u)
#define IC_RXFLR            (I2C1_BASE + 0x78u)
#define IC_SDA_HOLD         (I2C1_BASE + 0x7Cu)
#define IC_TX_ABRT_SOURCE   (I2C1_BASE + 0x80u)
#define IC_FS_SPKLEN        (I2C1_BASE + 0xA0u)

#define CON_MASTER_FAST     (0x165u)   /* Master, fast mode, restart on,
                                          slave off, TX_EMPTY_CTRL on.     */
#define CMD_READ            (0x100u)
#define CMD_STOP            (0x200u)
#define CMD_RESTART         (0x400u)
#define RAW_TX_EMPTY        (0x10u)
#define RAW_TX_ABRT         (0x40u)
#define RAW_STOP_DET        (0x200u)
#define DATA_MASK           (0xFFu)
#define ADDR_MASK           (0x7Fu)
#define I2C_FUNCSEL         (3u)

static bool wait_for(uint32_t reg, uint32_t mask, uint32_t start_us);
static bool aborted(void);

/*!
 * @brief Reset I2C1, set 400 kHz timing, hand GP2/GP3 to I2C with pulls.
 */
void
drv_i2c_init (void)
{
    uint32_t const period = (BOARD_SYS_CLK_HZ + (APP_IMU_I2C_HZ / 2u)) /
                            APP_IMU_I2C_HZ;
    uint32_t const lcnt   = (period * 3u) / 5u;
    uint32_t const hcnt   = period - lcnt;
    uint32_t const spklen = (lcnt < 16u) ? 1u : (lcnt / 16u);
    uint32_t const hold   = ((BOARD_SYS_CLK_HZ * 3u) / 10000000u) + 1u;

    board_unreset(RESETS_RESET_I2C1);
    out_w(IC_ENABLE, 0u);
    out_w(IC_CON, CON_MASTER_FAST);
    out_w(IC_FS_SCL_HCNT, hcnt);
    out_w(IC_FS_SCL_LCNT, lcnt);
    out_w(IC_FS_SPKLEN, spklen);
    out_w(IC_SDA_HOLD, hold);
    out_w(IC_TX_TL, 0u);
    out_w(IC_ENABLE, 1u);

    board_pin_function(APP_PIN_IMU_SDA, I2C_FUNCSEL, true);
    board_pin_function(APP_PIN_IMU_SCL, I2C_FUNCSEL, true);
}

/*!
 * @brief Write @p len bytes to a 7-bit address.
 *
 * @param[in] b_nostop  true to end without STOP (a read follows with a
 *                      repeated START).
 *
 * @return false on NACK/abort or timeout.
 */
bool
drv_i2c_write (uint8_t addr, uint8_t const * p_src, uint8_t len,
               bool b_nostop)
{
    uint32_t const start = board_time_us();
    bool           b_ok  = true;

    out_w(IC_ENABLE, 0u);
    out_w(IC_TAR, (uint32_t)addr & ADDR_MASK);
    out_w(IC_ENABLE, 1u);

    for (uint8_t idx = 0u; (idx < len) && b_ok; idx++)
    {
        bool const     b_last = ((idx + 1u) == len);
        uint32_t const cmd    = (uint32_t)p_src[idx] |
                                ((b_last && (!b_nostop)) ? CMD_STOP : 0u);

        out_w(IC_DATA_CMD, cmd);
        b_ok = wait_for(IC_RAW_INTR_STAT, RAW_TX_EMPTY, start) && !aborted();
    }

    if (b_ok && (!b_nostop))
    {
        b_ok = wait_for(IC_RAW_INTR_STAT, RAW_STOP_DET, start);
        (void)in_w(IC_CLR_STOP_DET);
    }

    return b_ok;
}

/*!
 * @brief Read @p len bytes after a repeated START, ending with STOP.
 */
bool
drv_i2c_read (uint8_t addr, uint8_t * p_dst, uint8_t len)
{
    uint32_t const start = board_time_us();
    bool           b_ok  = true;

    (void)addr;     /* Target already set by the preceding write. */

    for (uint8_t idx = 0u; (idx < len) && b_ok; idx++)
    {
        uint32_t const cmd = CMD_READ |
                             ((0u == idx) ? CMD_RESTART : 0u) |
                             (((idx + 1u) == len) ? CMD_STOP : 0u);

        out_w(IC_DATA_CMD, cmd);

        while ((0u == in_w(IC_RXFLR)) && !aborted() &&
               ((board_time_us() - start) < APP_IMU_I2C_TMO_US))
        {
            /* Wait for the byte (22.5 us at 400 kHz). */
        }

        b_ok = (0u != in_w(IC_RXFLR));

        if (b_ok)
        {
            p_dst[idx] = (uint8_t)(in_w(IC_DATA_CMD) & DATA_MASK);
        }
    }

    if (b_ok)
    {
        (void)wait_for(IC_RAW_INTR_STAT, RAW_STOP_DET, start);
        (void)in_w(IC_CLR_STOP_DET);
    }

    return b_ok;
}

/*!
 * @brief Poll until @p mask is set in @p reg, or the transfer times out.
 */
static bool
wait_for (uint32_t reg, uint32_t mask, uint32_t start_us)
{
    while ((0u == (in_w(reg) & mask)) &&
           ((board_time_us() - start_us) < APP_IMU_I2C_TMO_US))
    {
        /* Polled: a byte takes about 22 us at 400 kHz. */
    }

    return (0u != (in_w(reg) & mask));
}

/*!
 * @brief true (and abort cleared) if the controller aborted the transfer.
 */
static bool
aborted (void)
{
    bool const b_abort = (0u != (in_w(IC_RAW_INTR_STAT) & RAW_TX_ABRT));

    if (b_abort)
    {
        (void)in_w(IC_TX_ABRT_SOURCE);
        (void)in_w(IC_CLR_TX_ABRT);
    }

    return b_abort;
}

/*** end of file ***/
