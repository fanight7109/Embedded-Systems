/** @file mqtt_fmt.c
 *
 * @brief Binary MQTT frame packing and command validation (FR13, FR9).
 *
 * Validation order (Design Review 1.4): length, version, sequence (8-bit
 * serial-number arithmetic, (uint8_t)(seq - last) in 1..127, drops QoS 1
 * redeliveries), command and key known, state, value range. A failure is
 * acknowledged with ok = 0 and never forwarded. No command can steer.
 * "set" is accepted in READY for every key, and while the mission runs
 * (ACTIVE) for base_speed, kp, ki and kd only (live speed tuning).
 * "get" and "diag" (read-only sensor values) are accepted in any state.
 * "ev" (code 6, test-plan evidence) exists only in EVIDENCE=1 builds and
 * is then accepted in any state; elsewhere it is UNKNOWN_CMD.
 * "start" with val > 0 is the barcode test drive: straight for val mm,
 * line ignored (READY or STOPPED, like a normal start).
 *
 * Owner: Buddy 1 (JONATHAN TAN JIANG MING).
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "bitpack.h"
#include "../integration/msg_fmt.h"
#include "../integration/config.h"
#include "../integration/sys_state.h"
#include "../integration/app_cfg.h"
#include "mqtt_fmt.h"

#define VER_BITS            (4u)
#define SEQ_NEWEST          (127u)
#if (0 != APP_EVIDENCE)
#define CMD_COUNT           (7u)        /* + MQTT_CMD_EVID.               */
#else
#define CMD_COUNT           (6u)
#endif
#define ACK_VALHI_BITS      (2u)
#define ACK_PAD_BITS        (1u)
#define TRACE_ID_BITS       (8u)
#define TRACE_BIN_BITS      (12u)
#define TRACE_SAMPLE_MASK   (MQTT_TRACE_LEVEL_MASK | MQTT_TRACE_BLACK_BIT)

static bool is_live_key(uint8_t key);
static bool is_active(uint8_t sys_state);

/*!
 * @brief Build the 24-bit state word (bits 23..0):
 *        system:4 | motion:3 | mode:2 | line:3 | barcode:2 | obstacle:4 |
 *        imu:3 | comms:3.
 */
uint32_t
mqtt_fmt_state_word (uint8_t sys, uint8_t motion, uint8_t mode, uint8_t line,
                     uint8_t barcode, uint8_t obstacle, uint8_t imu,
                     uint8_t comms)
{
    uint8_t  buf[3] = { 0u, 0u, 0u };
    uint16_t pos    = 0u;

    bitpack_put(buf, &pos, sys, 4u);
    bitpack_put(buf, &pos, motion, 3u);
    bitpack_put(buf, &pos, mode, 2u);
    bitpack_put(buf, &pos, line, 3u);
    bitpack_put(buf, &pos, barcode, 2u);
    bitpack_put(buf, &pos, obstacle, 4u);
    bitpack_put(buf, &pos, imu, 3u);
    bitpack_put(buf, &pos, comms, 3u);

    return bitpack_get(buf, 0u, 24u);
}

/*!
 * @brief Pack the 22-byte (176-bit) telemetry frame.
 *
 * @return Frame length in bytes.
 */
uint8_t
mqtt_fmt_pack_telemetry (mqtt_fmt_tlm_t const * p_tlm, uint8_t * p_buf)
{
    uint16_t pos = 0u;

    (void)memset(p_buf, 0, MQTT_TLM_BYTES);
    bitpack_put(p_buf, &pos, MQTT_FMT_VER, VER_BITS);
    bitpack_put(p_buf, &pos, p_tlm->t_ticks, MQTT_TIME_BITS);
    bitpack_put(p_buf, &pos, p_tlm->state_word, 24u);
    bitpack_put(p_buf, &pos, p_tlm->spd_l, 10u);
    bitpack_put(p_buf, &pos, p_tlm->spd_r, 10u);
    bitpack_put(p_buf, &pos, p_tlm->enc_l, 16u);
    bitpack_put(p_buf, &pos, p_tlm->enc_r, 16u);
    bitpack_put(p_buf, &pos, p_tlm->dist_mm, 16u);
    bitpack_put(p_buf, &pos, p_tlm->ir_bits, 3u);
    bitpack_put(p_buf, &pos, p_tlm->err, 7u);
    bitpack_put(p_buf, &pos, p_tlm->bc_cmd, 2u);
    bitpack_put(p_buf, &pos, p_tlm->bc_dir, 1u);
    bitpack_put(p_buf, &pos, p_tlm->hump_cur, 7u);
    bitpack_put(p_buf, &pos, p_tlm->hump_max, 7u);
    bitpack_put(p_buf, &pos, p_tlm->obs_d, 11u);
    bitpack_put(p_buf, &pos, p_tlm->obs_brg, 5u);
    bitpack_put(p_buf, &pos, p_tlm->flt, 14u);
    bitpack_put(p_buf, &pos, p_tlm->drop, 4u);

    return (uint8_t)bitpack_bytes(pos);
}

uint8_t
mqtt_fmt_pack_heartbeat (uint8_t sys, uint32_t t_ticks, bool b_critical,
                         uint8_t reconnects, uint8_t * p_buf)
{
    uint16_t pos = 0u;

    (void)memset(p_buf, 0, MQTT_HB_BYTES);
    bitpack_put(p_buf, &pos, MQTT_FMT_VER, VER_BITS);
    bitpack_put(p_buf, &pos, sys, 4u);
    bitpack_put(p_buf, &pos, t_ticks, MQTT_TIME_BITS);
    bitpack_put(p_buf, &pos, b_critical ? 1u : 0u, 1u);
    bitpack_put(p_buf, &pos, bitpack_limit(reconnects, 4u, NULL), 4u);

    return (uint8_t)bitpack_bytes(pos);
}

/*!
 * @brief Pack an event frame, copying the event data bits from a queued
 *        mbf_tlm EVENT message.
 *
 * @param[in] p_src    Source message.
 * @param[in] src_pos  Bit position of the data in @p p_src.
 *
 * @return Frame length (4 to 9 bytes), or 0 for an unknown event.
 */
uint8_t
mqtt_fmt_pack_event (uint32_t t_ticks, uint8_t mq_ev, uint8_t const * p_src,
                     uint16_t src_pos, uint8_t * p_buf)
{
    uint8_t const data_bits = msg_mqev_data_bits(mq_ev);
    uint16_t      pos       = 0u;
    uint8_t       len       = 0u;

    (void)memset(p_buf, 0, MQTT_EVENT_MAX_BYTES);

    if (0u != data_bits)
    {
        bitpack_put(p_buf, &pos, MQTT_FMT_VER, VER_BITS);
        bitpack_put(p_buf, &pos, t_ticks, MQTT_TIME_BITS);
        bitpack_put(p_buf, &pos, mq_ev, 3u);

        for (uint8_t bit = 0u; bit < data_bits; bit++)
        {
            bitpack_put(p_buf, &pos,
                        bitpack_get(p_src, (uint16_t)(src_pos + bit), 1u),
                        1u);
        }

        len = (uint8_t)bitpack_bytes(pos);
    }

    return len;
}

/*!
 * @brief Validate a 4-byte command frame.
 *
 * @param[in]     sys_state  Current SYS_ST_* from T6's latest snapshot.
 * @param[in,out] p_seq      Last accepted sequence number.
 * @param[out]    p_cmd      Decoded fields (seq is valid if len >= 2).
 *
 * @return MQTT_ERR_NONE if the command may be forwarded.
 */
uint8_t
mqtt_fmt_check_cmd (uint8_t const * p_buf, uint16_t len, uint8_t sys_state,
                    mqtt_fmt_seq_t * p_seq, mqtt_fmt_cmd_t * p_cmd)
{
    uint8_t err = MQTT_ERR_NONE;

    (void)memset(p_cmd, 0, sizeof(*p_cmd));

    if (len >= 2u)
    {
        p_cmd->seq = (uint8_t)bitpack_get(p_buf, VER_BITS, 8u);
    }

    if (MQTT_CMD_BYTES != len)
    {
        err = MQTT_ERR_BAD_LEN;
    }
    else if (MQTT_FMT_VER != bitpack_get(p_buf, 0u, VER_BITS))
    {
        err = MQTT_ERR_BAD_VER;
    }
    else
    {
        uint8_t const delta = (uint8_t)(p_cmd->seq - p_seq->last_seq);

        p_cmd->cmd = (uint8_t)bitpack_get(p_buf, 12u, 3u);
        p_cmd->key = (uint8_t)bitpack_get(p_buf, 15u, 3u);
        p_cmd->val = (uint16_t)bitpack_get(p_buf, 18u, 10u);

        if (p_seq->b_have_seq && ((0u == delta) || (delta > SEQ_NEWEST)))
        {
            err = MQTT_ERR_DUP_SEQ;
        }
        else if ((p_cmd->cmd >= CMD_COUNT) ||
                 ((MQTT_CMD_SET == p_cmd->cmd) &&
                  (p_cmd->key >= CONFIG_NUM_KEYS)) ||
                 ((MQTT_CMD_GET == p_cmd->cmd) &&
                  (p_cmd->key >= MQTT_NUM_GET_KEYS)) ||
                 ((MQTT_CMD_DIAG == p_cmd->cmd) &&
                  (p_cmd->key >= MQTT_NUM_DIAG_KEYS)))
        {
            err = MQTT_ERR_UNKNOWN_CMD;
        }
        else if (((MQTT_CMD_SET == p_cmd->cmd) &&
                  (SYS_ST_READY != sys_state) &&
                  !(is_live_key(p_cmd->key) &&
                    (is_active(sys_state) ||
                     (SYS_ST_STOPPED == sys_state)))) ||
                 ((MQTT_CMD_START == p_cmd->cmd) &&
                  (SYS_ST_READY != sys_state) &&
                  (SYS_ST_STOPPED != sys_state)))
        {
            err = MQTT_ERR_BAD_STATE;
        }
        else if ((MQTT_CMD_SET == p_cmd->cmd) &&
                 (p_cmd->val > config_max_encoded(p_cmd->key)))
        {
            err = MQTT_ERR_OUT_OF_RANGE;
        }
        else
        {
            p_seq->last_seq   = p_cmd->seq;
            p_seq->b_have_seq = true;
        }
    }

    return err;
}

uint8_t
mqtt_fmt_pack_ack (uint8_t seq, bool b_ok, uint8_t err, uint8_t key,
                   uint16_t val, uint8_t * p_buf)
{
    uint16_t pos = 0u;

    (void)memset(p_buf, 0, MQTT_ACK_BYTES);
    bitpack_put(p_buf, &pos, MQTT_FMT_VER, VER_BITS);
    bitpack_put(p_buf, &pos, seq, 8u);
    bitpack_put(p_buf, &pos, b_ok ? 1u : 0u, 1u);
    bitpack_put(p_buf, &pos, err, 3u);
    bitpack_put(p_buf, &pos, key, 3u);
    bitpack_put(p_buf, &pos, (uint32_t)val & MQTT_ACK_VAL_MASK,
                MQTT_ACK_VAL_BITS);
    bitpack_put(p_buf, &pos, ((uint32_t)val & MQTT_ACK_VAL_MAX) >>
                MQTT_ACK_VAL_BITS, ACK_VALHI_BITS);
    bitpack_put(p_buf, &pos, 0u, ACK_PAD_BITS);

    return (uint8_t)bitpack_bytes(pos);
}

/*!
 * @brief Build byte 0 of an outgoing frame for the network task.
 */
/*!
 * @brief Pack one barcode trace frame (is05/car/bctrace).
 *
 * @param[in]  trace_id   Trace number (wraps), same in every frame of it.
 * @param[in]  chunk      This frame, 0..chunks-1.
 * @param[in]  chunks     Frames in the trace.
 * @param[in]  bin_um     Travel per sample, micrometres (12 bits).
 * @param[in]  p_samples  blackness:12 | black:1 per sample, oldest first.
 * @param[in]  count      Samples in this frame, 0..MQTT_TRACE_PER_FRAME
 *                        (the rest are sent as 0).
 *
 * @return MQTT_TRACE_BYTES.
 */
uint8_t
mqtt_fmt_pack_trace (uint8_t trace_id, uint8_t chunk, uint8_t chunks,
                     uint16_t bin_um, uint16_t const * p_samples,
                     uint8_t count, uint8_t * p_buf)
{
    uint16_t pos = 0u;

    (void)memset(p_buf, 0, MQTT_TRACE_BYTES);
    bitpack_put(p_buf, &pos, MQTT_FMT_VER, VER_BITS);
    bitpack_put(p_buf, &pos, trace_id, TRACE_ID_BITS);
    bitpack_put(p_buf, &pos, chunk, TRACE_ID_BITS);
    bitpack_put(p_buf, &pos, chunks, TRACE_ID_BITS);
    bitpack_put(p_buf, &pos, bitpack_limit(bin_um, TRACE_BIN_BITS, NULL),
                TRACE_BIN_BITS);

    for (uint8_t idx = 0u; idx < MQTT_TRACE_PER_FRAME; idx++)
    {
        uint32_t const sample = (idx < count) ?
                                ((uint32_t)p_samples[idx] &
                                 TRACE_SAMPLE_MASK) : 0u;

        bitpack_put(p_buf, &pos, sample, MQTT_TRACE_SAMPLE_BITS);
    }

    return MQTT_TRACE_BYTES;
}

uint8_t
mqtt_fmt_out_header (uint8_t topic, uint8_t qos, bool b_retain)
{
    return (uint8_t)(((uint32_t)topic & MQTT_FMT_HDR_TOPIC_MASK) |
                     (((uint32_t)qos & 1u) << MQTT_FMT_HDR_QOS_SHIFT) |
                     ((b_retain ? 1u : 0u) << MQTT_FMT_HDR_RET_SHIFT));
}

/*!
 * @brief Topic string for an outgoing frame header, or NULL if invalid.
 */
char const *
mqtt_fmt_out_topic (uint8_t header)
{
    static char const * const topics[MQTT_FMT_OUT_TOPICS] =
    {
        MQTT_TOPIC_TELEMETRY, MQTT_TOPIC_HEARTBEAT, MQTT_TOPIC_EVENT,
        MQTT_TOPIC_ACK, MQTT_TOPIC_STATUS, MQTT_TOPIC_BCTRACE
    };
    uint32_t const topic = (uint32_t)header & MQTT_FMT_HDR_TOPIC_MASK;

    return (topic < MQTT_FMT_OUT_TOPICS) ? topics[topic] : NULL;
}

uint8_t
mqtt_fmt_out_qos (uint8_t header)
{
    return (uint8_t)(((uint32_t)header >> MQTT_FMT_HDR_QOS_SHIFT) & 1u);
}

bool
mqtt_fmt_out_retain (uint8_t header)
{
    return (0u != (((uint32_t)header >> MQTT_FMT_HDR_RET_SHIFT) & 1u));
}

/*!
 * @brief Keys that may be set while driving or STOPPED: base_speed and the
 *        speed PID gains.
 */
static bool
is_live_key (uint8_t key)
{
    return (CONFIG_KEY_BASE_SPEED == key) || (CONFIG_KEY_KP == key) ||
           (CONFIG_KEY_KI == key) || (CONFIG_KEY_KD == key);
}

/*!
 * @brief true in the composite ACTIVE state (mission running).
 */
static bool
is_active (uint8_t sys_state)
{
    return (SYS_ST_LINE_FOLLOW == sys_state) ||
           (SYS_ST_BARCODE_ACTION == sys_state) ||
           (SYS_ST_LINE_SEARCH == sys_state) ||
           (SYS_ST_OBSTACLE == sys_state);
}

/*** end of file ***/
