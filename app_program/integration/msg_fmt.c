/** @file msg_fmt.c
 *
 * @brief Pack and unpack the inter-task messages defined in msg_fmt.h.
 *
 * Owner: Integration (all buddies).
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "../communications/bitpack.h"
#include "msg_fmt.h"

#define NAV_NUM_EVENTS      (16u)

/* Payload width in bits for each mbf_nav event code. */
static uint8_t const g_nav_payload_bits[NAV_NUM_EVENTS] =
{
    11u,    /* BARCODE    cmd:2 dir:1 sym:7 nocmd:1      */
    16u,    /* JUNCTION   dist:16                        */
    1u,     /* LINE_LOST  dir:1                          */
    0u,     /* LINE_FOUND                                */
    4u,     /* MOVE_DONE  seq:4                          */
    4u,     /* STOPPED    seq:4                          */
    4u,     /* REJECT     seq:4                          */
    11u,    /* OBSTACLE   d:11                           */
    40u,    /* PROFILE    brg:5 d_min:11 w:8 cl:8 cr:8   */
    0u,     /* SCAN_FAULT                                */
    11u,    /* HUMP       h:7 n:4                        */
    0u,     /* COLLISION                                 */
    1u,     /* IMU_FAULT  active:1                       */
    10u,    /* CMD_START  drive_mm:10 (0 = line mission) */
    0u,     /* CMD_STOP                                  */
    13u     /* CMD_SET    key:3 val:10                   */
};

/* Data width in bits for each MQTT event code (Design Review 1.4). */
static uint8_t const g_mqev_data_bits[7] =
{
    11u, 16u, 11u, 11u, 40u, 18u, 31u
};

/*!
 * @brief Pack an mbf_nav message.
 *
 * @param[in]  p_msg  Unpacked event.
 * @param[out] p_buf  At least MSG_NAV_MAX_BYTES bytes.
 *
 * @return Packed length in bytes; only this length is sent to the kernel.
 */
uint8_t
msg_nav_pack (msg_nav_t const * p_msg, uint8_t * p_buf)
{
    uint16_t pos = 0u;
    uint8_t  event  = (uint8_t)(p_msg->event & 0xFu);

    (void)memset(p_buf, 0, MSG_NAV_MAX_BYTES);
    bitpack_put(p_buf, &pos, event, MSG_NAV_EV_BITS);

    switch (event)
    {
        case MSG_EV_BARCODE:
            bitpack_put(p_buf, &pos, p_msg->small_a, 2u);
            bitpack_put(p_buf, &pos, p_msg->small_b, 1u);
            bitpack_put(p_buf, &pos, p_msg->small_c, MSG_BC_SYM_BITS);
            bitpack_put(p_buf, &pos, p_msg->small_d, 1u);
        break;

        case MSG_EV_JUNCTION:
            bitpack_put(p_buf, &pos, p_msg->value, 16u);
        break;

        case MSG_EV_MOVE_DONE:
        case MSG_EV_STOPPED:
        case MSG_EV_REJECT:
            bitpack_put(p_buf, &pos, p_msg->small_a, 4u);
        break;

        case MSG_EV_OBSTACLE:
            bitpack_put(p_buf, &pos, p_msg->value, 11u);
        break;

        case MSG_EV_PROFILE:
            bitpack_put(p_buf, &pos, p_msg->small_a, 5u);
            bitpack_put(p_buf, &pos, p_msg->value, 11u);
            bitpack_put(p_buf, &pos, p_msg->small_b, 8u);
            bitpack_put(p_buf, &pos, p_msg->small_c, 8u);
            bitpack_put(p_buf, &pos, p_msg->small_d, 8u);
        break;

        case MSG_EV_HUMP:
            bitpack_put(p_buf, &pos, p_msg->value, 7u);
            bitpack_put(p_buf, &pos, p_msg->small_a, 4u);
        break;

        case MSG_EV_LINE_LOST:
        case MSG_EV_IMU_FAULT:
            bitpack_put(p_buf, &pos, p_msg->small_a, 1u);
        break;

        case MSG_EV_CMD_SET:
            bitpack_put(p_buf, &pos, p_msg->small_a, 3u);
            bitpack_put(p_buf, &pos, p_msg->value, 10u);
        break;

        case MSG_EV_CMD_START:
            bitpack_put(p_buf, &pos, p_msg->value, 10u);
        break;

        default:
            /* Events without payload. */
        break;
    }

    return (uint8_t)bitpack_bytes(pos);
}

/*!
 * @brief Unpack an mbf_nav message and check its length.
 *
 * @return true if the length matches the event code.
 */
bool
msg_nav_unpack (uint8_t const * p_buf, uint8_t len, msg_nav_t * p_msg)
{
    bool     b_ok = false;
    uint16_t pos  = MSG_NAV_EV_BITS;

    (void)memset(p_msg, 0, sizeof(*p_msg));

    if (len > 0u)
    {
        uint8_t const event = (uint8_t)bitpack_get(p_buf, 0u, MSG_NAV_EV_BITS);
        uint16_t const bits = (uint16_t)(MSG_NAV_EV_BITS +
                                         g_nav_payload_bits[event]);

        p_msg->event = event;
        b_ok = (bitpack_bytes(bits) == len);

        switch (event)
        {
            case MSG_EV_BARCODE:
                p_msg->small_a = (uint8_t)bitpack_get(p_buf, pos, 2u);
                p_msg->small_b = (uint8_t)bitpack_get(p_buf, pos + 2u, 1u);
                p_msg->small_c = (uint8_t)bitpack_get(p_buf, pos + 3u,
                                                      MSG_BC_SYM_BITS);
                p_msg->small_d = (uint8_t)bitpack_get(p_buf, pos + 10u, 1u);
            break;

            case MSG_EV_JUNCTION:
                p_msg->value = (uint16_t)bitpack_get(p_buf, pos, 16u);
            break;

            case MSG_EV_MOVE_DONE:
            case MSG_EV_STOPPED:
            case MSG_EV_REJECT:
                p_msg->small_a = (uint8_t)bitpack_get(p_buf, pos, 4u);
            break;

            case MSG_EV_OBSTACLE:
                p_msg->value = (uint16_t)bitpack_get(p_buf, pos, 11u);
            break;

            case MSG_EV_PROFILE:
                p_msg->small_a = (uint8_t)bitpack_get(p_buf, pos, 5u);
                p_msg->value   = (uint16_t)bitpack_get(p_buf, pos + 5u, 11u);
                p_msg->small_b = (uint8_t)bitpack_get(p_buf, pos + 16u, 8u);
                p_msg->small_c = (uint8_t)bitpack_get(p_buf, pos + 24u, 8u);
                p_msg->small_d = (uint8_t)bitpack_get(p_buf, pos + 32u, 8u);
            break;

            case MSG_EV_HUMP:
                p_msg->value   = (uint16_t)bitpack_get(p_buf, pos, 7u);
                p_msg->small_a = (uint8_t)bitpack_get(p_buf, pos + 7u, 4u);
            break;

            case MSG_EV_LINE_LOST:
            case MSG_EV_IMU_FAULT:
                p_msg->small_a = (uint8_t)bitpack_get(p_buf, pos, 1u);
            break;

            case MSG_EV_CMD_SET:
                p_msg->small_a = (uint8_t)bitpack_get(p_buf, pos, 3u);
                p_msg->value   = (uint16_t)bitpack_get(p_buf, pos + 3u, 10u);
            break;

            case MSG_EV_CMD_START:
                p_msg->value = (uint16_t)bitpack_get(p_buf, pos, 10u);
            break;

            default:
                /* Events without payload. */
            break;
        }
    }

    return b_ok;
}

/*!
 * @brief Pack a motion command. Speed is clamped to 100 .. 300 mm/s.
 */
uint32_t
msg_mc_pack (uint8_t type, uint16_t speed_mm_s, uint16_t param, uint8_t dir,
             uint8_t seq)
{
    uint16_t speed = speed_mm_s;

    if (speed < MSG_SPEED_MIN)
    {
        speed = MSG_SPEED_MIN;
    }
    else if (speed > MSG_SPEED_MAX)
    {
        speed = MSG_SPEED_MAX;
    }
    else
    {
        /* In range. */
    }

    return (((uint32_t)type & MSG_MC_TYPE_MASK) << MSG_MC_TYPE_SHIFT) |
           (((uint32_t)(speed - MSG_SPEED_MIN) & MSG_MC_SPEED_MASK)
                << MSG_MC_SPEED_SHIFT) |
           (((uint32_t)param & MSG_MC_PARAM_MASK) << MSG_MC_PARAM_SHIFT) |
           (((uint32_t)dir & MSG_MC_DIR_MASK) << MSG_MC_DIR_SHIFT) |
           (((uint32_t)seq & MSG_MC_SEQ_MASK) << MSG_MC_SEQ_SHIFT);
}

uint8_t
msg_mc_type (uint32_t cmd)
{
    return (uint8_t)((cmd >> MSG_MC_TYPE_SHIFT) & MSG_MC_TYPE_MASK);
}

uint16_t
msg_mc_speed (uint32_t cmd)
{
    return (uint16_t)(((cmd >> MSG_MC_SPEED_SHIFT) & MSG_MC_SPEED_MASK) +
                      MSG_SPEED_MIN);
}

uint16_t
msg_mc_param (uint32_t cmd)
{
    return (uint16_t)((cmd >> MSG_MC_PARAM_SHIFT) & MSG_MC_PARAM_MASK);
}

uint8_t
msg_mc_dir (uint32_t cmd)
{
    return (uint8_t)((cmd >> MSG_MC_DIR_SHIFT) & MSG_MC_DIR_MASK);
}

uint8_t
msg_mc_seq (uint32_t cmd)
{
    return (uint8_t)((cmd >> MSG_MC_SEQ_SHIFT) & MSG_MC_SEQ_MASK);
}

/*!
 * @brief Pack a line-error message. The error saturates at +/- 31.5 mm.
 */
uint16_t
msg_le_pack (int16_t err_half_mm, uint8_t ir_bits, bool b_valid,
             bool b_junction, uint8_t seq)
{
    uint32_t const err_field = bitpack_offset(err_half_mm,
                                              MSG_LE_ERR_OFFSET, 7u, NULL);
    uint32_t       msg       = 0u;

    msg = ((err_field & MSG_LE_ERR_MASK) << MSG_LE_ERR_SHIFT) |
          (((uint32_t)ir_bits & MSG_LE_IR_MASK) << MSG_LE_IR_SHIFT) |
          ((b_valid ? 1u : 0u) << MSG_LE_VALID_SHIFT) |
          ((b_junction ? 1u : 0u) << MSG_LE_JCT_SHIFT) |
          (((uint32_t)seq & MSG_LE_SEQ_MASK) << MSG_LE_SEQ_SHIFT);

    return (uint16_t)msg;
}

int16_t
msg_le_err (uint16_t msg)
{
    uint32_t const field = ((uint32_t)msg >> MSG_LE_ERR_SHIFT) &
                           MSG_LE_ERR_MASK;

    return (int16_t)bitpack_signed(field, MSG_LE_ERR_OFFSET);
}

bool
msg_le_valid (uint16_t msg)
{
    return (0u != (((uint32_t)msg >> MSG_LE_VALID_SHIFT) & 1u));
}

uint8_t
msg_le_ir (uint16_t msg)
{
    return (uint8_t)(((uint32_t)msg >> MSG_LE_IR_SHIFT) & MSG_LE_IR_MASK);
}

uint8_t
msg_sr_pack (uint8_t type, uint8_t centre_idx)
{
    return (uint8_t)((((uint32_t)type & MSG_SR_TYPE_MASK) << MSG_SR_TYPE_SHIFT)
                     | ((uint32_t)centre_idx & MSG_SR_CENTRE_MASK));
}

uint8_t
msg_sr_type (uint8_t req)
{
    return (uint8_t)(((uint32_t)req >> MSG_SR_TYPE_SHIFT) & MSG_SR_TYPE_MASK);
}

uint8_t
msg_sr_centre (uint8_t req)
{
    return (uint8_t)((uint32_t)req & MSG_SR_CENTRE_MASK);
}

/*!
 * @brief Data width of an MQTT event (0 for an unknown code).
 */
uint8_t
msg_mqev_data_bits (uint8_t mq_ev)
{
    uint8_t bits = 0u;

    if (mq_ev < sizeof(g_mqev_data_bits))
    {
        bits = g_mqev_data_bits[mq_ev];
    }

    return bits;
}

/*** end of file ***/
