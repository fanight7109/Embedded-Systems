/** @file mqtt_fmt.h
 *
 * @brief MQTT topics, binary frame layouts and command validation
 *        (Design Review 1.4, NFR11, A10).
 *
 * Every frame starts with ver:4 (= 1) and is packed MSB-first with
 * bitpack_put(), so the laptop decodes it with the same field table
 * (tools/dashboard.py mirrors the tables in this header).
 *
 *   telemetry  22 B: ver4 t19 st24 spdL10 spdR10 encL16 encR16 dist16 ir3
 *                    err7 bc2 bcdir1 hcur7 hmax7 obsd11 obsbrg5 flt14 drop4
 *   heartbeat   4 B: ver4 st4 t19 crit1 rcn4
 *   event   4..9 B: ver4 t19 ev3 data(<=40)
 *   cmd         4 B: ver4 seq8 cmd3 key3 val10 pad4
 *   ack         4 B: ver4 seq8 ok1 err3 key3 val10 valhi2 pad1
 *                    (value = valhi << 10 | val; valhi is 0 for every
 *                    config key, so older decoders still read those)
 *   status      1 B: 0x01 on connect, 0x00 as last will (retained)
 *
 * Owner: Buddy 1 (JONATHAN TAN JIANG MING).
 */

#ifndef MQTT_FMT_H
#define MQTT_FMT_H

#include <stdint.h>
#include <stdbool.h>

#define MQTT_FMT_VER            (1u)

#define MQTT_TOPIC_TELEMETRY    "is05/car/telemetry"
#define MQTT_TOPIC_HEARTBEAT    "is05/car/heartbeat"
#define MQTT_TOPIC_EVENT        "is05/car/event"
#define MQTT_TOPIC_CMD          "is05/car/cmd"
#define MQTT_TOPIC_ACK          "is05/car/ack"
#define MQTT_TOPIC_STATUS       "is05/car/status"
#define MQTT_TOPIC_BCTRACE      "is05/car/bctrace"

#define MQTT_TLM_BYTES          (22u)
#define MQTT_HB_BYTES           (4u)
#define MQTT_CMD_BYTES          (4u)
#define MQTT_ACK_BYTES          (4u)
#define MQTT_EVENT_MAX_BYTES    (9u)
/* Barcode trace frame: ver:4 id:8 chunk:8 chunks:8 bin_um:12, then
 * MQTT_TRACE_PER_FRAME samples of blackness:12 | black:1. */
#define MQTT_TRACE_BYTES        (23u)
#define MQTT_TRACE_PER_FRAME    (11u)
#define MQTT_TRACE_SAMPLE_BITS  (13u)
#define MQTT_TRACE_LEVEL_MASK   (0xFFFu)
#define MQTT_TRACE_BLACK_BIT    (0x1000u)
#define MQTT_TIME_BITS          (19u)
#define MQTT_TIME_MASK          (0x7FFFFu)

/*
 * Outgoing frame header (byte 0 of an mbf_mqtt_out message): topic:3 |
 * qos:1 | retain:1, so the network task knows where to publish.
 */
#define MQTT_FMT_OUT_TELEMETRY  (0u)
#define MQTT_FMT_OUT_HEARTBEAT  (1u)
#define MQTT_FMT_OUT_EVENT      (2u)
#define MQTT_FMT_OUT_ACK        (3u)
#define MQTT_FMT_OUT_STATUS     (4u)
#define MQTT_FMT_OUT_BCTRACE    (5u)
#define MQTT_FMT_OUT_TOPICS     (6u)
#define MQTT_FMT_HDR_TOPIC_MASK (0x7u)
#define MQTT_FMT_HDR_QOS_SHIFT  (3u)
#define MQTT_FMT_HDR_RET_SHIFT  (4u)

/* Command codes (3 bits). */
#define MQTT_CMD_START          (0u)
#define MQTT_CMD_STOP           (1u)
#define MQTT_CMD_ESTOP          (2u)
#define MQTT_CMD_SET            (3u)
#define MQTT_CMD_GET            (4u)
#define MQTT_CMD_DIAG           (5u)
#define MQTT_CMD_EVID           (6u)   /* EVIDENCE=1 builds only: key =  */
                                       /* EV_ACT_*, val = its parameter. */

/*
 * Read-only GET keys after the six config keys: the IR contrast (max - min
 * ADC counts, divided by 4 to fit 10 bits) each sensor saw in the last
 * CALIBRATE spin. Answered by T1 in any state; SET rejects them.
 */
#define MQTT_KEY_IR_CAL_L       (6u)
#define MQTT_KEY_IR_CAL_C       (7u)
#define MQTT_NUM_GET_KEYS       (8u)
#define MQTT_IR_CAL_DIV         (4u)

/*
 * DIAG keys (28 Oct): live read-only sensor values, answered by T1 in any
 * state, full 12 bits in the ack. Barcode sensor (GP28 AO), blackness in
 * ADC counts 0..4095 (raw AO while APP_IR_BLACK_IS_HIGH is 1).
 */
#define MQTT_DIAG_BC_RAW        (0u)   /* Live reading.                    */
#define MQTT_DIAG_BC_CAL_WHITE  (1u)   /* Lowest in the last CALIBRATE.    */
#define MQTT_DIAG_BC_CAL_BLACK  (2u)   /* Highest in the last CALIBRATE.   */
#define MQTT_DIAG_BC_BLACK_ON   (3u)   /* White -> black threshold.        */
#define MQTT_DIAG_BC_WHITE_ON   (4u)   /* Black -> white threshold.        */
#define MQTT_DIAG_BC_TRACE      (5u)   /* Publish the barcode trace now.   */
#define MQTT_NUM_DIAG_KEYS      (6u)
#define MQTT_ACK_VAL_BITS       (10u)
#define MQTT_ACK_VAL_MASK       (0x3FFu)
#define MQTT_ACK_VAL_MAX        (0xFFFu)   /* 12 bits with valhi.        */

/* Ack error codes (3 bits). */
#define MQTT_ERR_NONE           (0u)
#define MQTT_ERR_BAD_LEN        (1u)
#define MQTT_ERR_BAD_VER        (2u)
#define MQTT_ERR_UNKNOWN_CMD    (3u)
#define MQTT_ERR_BAD_STATE      (4u)
#define MQTT_ERR_OUT_OF_RANGE   (5u)
#define MQTT_ERR_DUP_SEQ        (6u)

/* Telemetry fields, already in their encoded (unsigned) form. */
typedef struct
{
    uint32_t t_ticks;      /* 10 ms ticks, 19 bits                        */
    uint32_t state_word;   /* 24 bits, see mqtt_fmt_state_word()          */
    uint16_t spd_l;        /* mm/s + 512, 10 bits                         */
    uint16_t spd_r;
    uint16_t enc_l;
    uint16_t enc_r;
    uint16_t dist_mm;
    uint8_t  ir_bits;      /* 3 bits                                      */
    uint8_t  err;          /* 0.5 mm + 64, 7 bits                         */
    uint8_t  bc_cmd;       /* last barcode command, 2 bits                */
    uint8_t  bc_dir;
    uint8_t  hump_cur;
    uint8_t  hump_max;
    uint16_t obs_d;        /* 11 bits, 2047 = none                        */
    uint8_t  obs_brg;      /* 5 bits                                      */
    uint16_t flt;          /* 14 bits                                     */
    uint8_t  drop;         /* 4 bits                                      */
} mqtt_fmt_tlm_t;

typedef struct
{
    uint8_t last_seq;
    bool    b_have_seq;
} mqtt_fmt_seq_t;

typedef struct
{
    uint8_t  seq;
    uint8_t  cmd;
    uint8_t  key;
    uint16_t val;
} mqtt_fmt_cmd_t;

uint32_t mqtt_fmt_state_word(uint8_t sys, uint8_t motion, uint8_t mode,
                             uint8_t line, uint8_t barcode, uint8_t obstacle,
                             uint8_t imu, uint8_t comms);
uint8_t  mqtt_fmt_pack_telemetry(mqtt_fmt_tlm_t const * p_tlm,
                                 uint8_t * p_buf);
uint8_t  mqtt_fmt_pack_heartbeat(uint8_t sys, uint32_t t_ticks,
                                 bool b_critical, uint8_t reconnects,
                                 uint8_t * p_buf);
uint8_t  mqtt_fmt_pack_event(uint32_t t_ticks, uint8_t mq_ev,
                             uint8_t const * p_src, uint16_t src_pos,
                             uint8_t * p_buf);
uint8_t  mqtt_fmt_check_cmd(uint8_t const * p_buf, uint16_t len,
                            uint8_t sys_state, mqtt_fmt_seq_t * p_seq,
                            mqtt_fmt_cmd_t * p_cmd);
uint8_t  mqtt_fmt_pack_ack(uint8_t seq, bool b_ok, uint8_t err, uint8_t key,
                           uint16_t val, uint8_t * p_buf);
uint8_t  mqtt_fmt_pack_trace(uint8_t trace_id, uint8_t chunk,
                             uint8_t chunks, uint16_t bin_um,
                             uint16_t const * p_samples, uint8_t count,
                             uint8_t * p_buf);
uint8_t  mqtt_fmt_out_header(uint8_t topic, uint8_t qos, bool b_retain);
char const * mqtt_fmt_out_topic(uint8_t header);
uint8_t  mqtt_fmt_out_qos(uint8_t header);
bool     mqtt_fmt_out_retain(uint8_t header);

#endif /* MQTT_FMT_H */

/*** end of file ***/
