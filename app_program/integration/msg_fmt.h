/** @file msg_fmt.h
 *
 * @brief Inter-task message layouts (Design Review Sections 1.5 and 4).
 *
 * This is the single interface header for every queued message, so each
 * buddy can build and test against it independently (NFR7). All layouts
 * use named SHIFT/MASK macros or MSB-first bit positions (rule R5); none
 * uses a struct overlay or bit-field.
 *
 * Messages and owners:
 *   mbf_nav        T1..T5 -> T6   event:4 + payload <= 40 bits (6 B max).
 *   mbf_motion_cmd T6 -> T2       one uint32_t (26 bits used).
 *   mbf_line_err   T3 -> T2       one uint16_t.
 *   mbf_scan_req   T6 -> T5       one uint8_t (7 bits used).
 *   mbf_tlm        T2..T6 -> T1   src:3 + snapshot <= 73 bits (10 B max).
 *
 * Owner: Integration (all buddies).
 */

#ifndef MSG_FMT_H
#define MSG_FMT_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------------
 * Units shared by every interface (NFR7).
 * --------------------------------------------------------------------- */
#define MSG_RANGE_NONE          (2047u)  /* 11-bit range sentinel.        */

/* ------------------------------------------------------------------------
 * mbf_nav: event code in the top 4 bits of byte 0, payload follows.
 * --------------------------------------------------------------------- */
#define MSG_NAV_MAX_BYTES       (6u)
#define MSG_NAV_EV_BITS         (4u)

#define MSG_EV_BARCODE          (0u)   /* cmd:2 dir:1 sym:7 nocmd:1        */
#define MSG_EV_JUNCTION         (1u)   /* dist:16 (odometer, mm)           */
#define MSG_EV_LINE_LOST        (2u)   /* dir:1 (side the line went)       */
#define MSG_EV_LINE_FOUND       (3u)   /* no payload                       */
#define MSG_EV_MOVE_DONE        (4u)   /* seq:4                            */
#define MSG_EV_STOPPED          (5u)   /* seq:4                            */
#define MSG_EV_REJECT           (6u)   /* seq:4                            */
#define MSG_EV_OBSTACLE         (7u)   /* d:11                             */
#define MSG_EV_PROFILE          (8u)   /* brg:5 d_min:11 w:8 cl:8 cr:8     */
#define MSG_EV_SCAN_FAULT       (9u)   /* no payload                       */
#define MSG_EV_HUMP             (10u)  /* h:7 n:4                          */
#define MSG_EV_COLLISION        (11u)  /* no payload                       */
#define MSG_EV_IMU_FAULT        (12u)  /* active:1                         */
#define MSG_EV_CMD_START        (13u)  /* drive_mm:10 (0 = mission)        */
#define MSG_EV_CMD_STOP         (14u)  /* no payload                       */
#define MSG_EV_CMD_SET          (15u)  /* key:3 val:10                     */

/* Barcode commands (2 bits) and read direction (1 bit). */
#define MSG_BC_LEFT             (0u)
#define MSG_BC_RIGHT            (1u)
#define MSG_BC_STRAIGHT         (2u)
#define MSG_BC_UTURN            (3u)

/* Barcode symbol (7-bit ASCII, 0 = not given) and no-command flag: every
 * decoded code is reported, so a valid Code 39 symbol outside A..D still
 * reaches the dashboard; T6 acts only when nocmd is 0. */
#define MSG_BC_SYM_BITS         (7u)
#define MSG_BC_SYM_MASK         (0x7Fu)

/*
 * Unpacked view of one mbf_nav message, task-local only. Fields not used by
 * the event stay zero.
 */
typedef struct
{
    uint8_t  event;
    uint8_t  small_a;    /* cmd, seq, key, active, bearing, hump count   */
    uint8_t  small_b;    /* dir, width/4                                  */
    uint8_t  small_c;    /* clearance left/4, barcode symbol              */
    uint8_t  small_d;    /* clearance right/4, barcode no-command flag    */
    uint16_t value;      /* dist, d, d_min, val, height                   */
} msg_nav_t;

uint8_t msg_nav_pack(msg_nav_t const * p_msg, uint8_t * p_buf);
bool    msg_nav_unpack(uint8_t const * p_buf, uint8_t len, msg_nav_t * p_msg);

/* ------------------------------------------------------------------------
 * mbf_motion_cmd: type:3 | speed:8 | param:10 | dir:1 | seq:4 (26 bits).
 * --------------------------------------------------------------------- */
#define MSG_MC_TYPE_SHIFT       (23u)
#define MSG_MC_TYPE_MASK        (0x7u)
#define MSG_MC_SPEED_SHIFT      (15u)
#define MSG_MC_SPEED_MASK       (0xFFu)  /* speed - MSG_SPEED_MIN mm/s    */
#define MSG_MC_PARAM_SHIFT      (5u)
#define MSG_MC_PARAM_MASK       (0x3FFu) /* mm or degrees                 */
#define MSG_MC_DIR_SHIFT        (4u)
#define MSG_MC_DIR_MASK         (0x1u)   /* turn: 0 left, 1 right         */
#define MSG_MC_SEQ_SHIFT        (0u)
#define MSG_MC_SEQ_MASK         (0xFu)

#define MSG_MC_STOP             (0u)
#define MSG_MC_LINE_FOLLOW      (1u)
#define MSG_MC_MOVE_DIST        (2u)
#define MSG_MC_TURN_ANGLE       (3u)
#define MSG_MC_REVERSE          (4u)
#define MSG_MC_EDGE_ALIGN       (5u)     /* Line follow at zero speed:    */
                                         /* pivot onto the edge (CAL).    */

#define MSG_SPEED_MIN           (45u)    /* 8-bit offset: 45..300 mm/s.   */
#define MSG_SPEED_MAX           (300u)
#define MSG_DIR_LEFT            (0u)
#define MSG_DIR_RIGHT           (1u)

uint32_t msg_mc_pack(uint8_t type, uint16_t speed_mm_s, uint16_t param,
                     uint8_t dir, uint8_t seq);
uint8_t  msg_mc_type(uint32_t cmd);
uint16_t msg_mc_speed(uint32_t cmd);
uint16_t msg_mc_param(uint32_t cmd);
uint8_t  msg_mc_dir(uint32_t cmd);
uint8_t  msg_mc_seq(uint32_t cmd);

/* ------------------------------------------------------------------------
 * mbf_line_err: err:7 | ir:3 | valid:1 | junction:1 | seq:4 (16 bits).
 * --------------------------------------------------------------------- */
#define MSG_LE_ERR_SHIFT        (9u)
#define MSG_LE_ERR_MASK         (0x7Fu)  /* 0.5 mm units + 64             */
#define MSG_LE_IR_SHIFT         (6u)
#define MSG_LE_IR_MASK          (0x7u)   /* bit2 L, bit1 C, bit0 BC       */
#define MSG_LE_VALID_SHIFT      (5u)
#define MSG_LE_JCT_SHIFT        (4u)
#define MSG_LE_SEQ_SHIFT        (0u)
#define MSG_LE_SEQ_MASK         (0xFu)
#define MSG_LE_ERR_OFFSET       (64)
#define MSG_LE_ERR_LIMIT        (63)

uint16_t msg_le_pack(int16_t err_half_mm, uint8_t ir_bits, bool b_valid,
                     bool b_junction, uint8_t seq);
int16_t  msg_le_err(uint16_t msg);
bool     msg_le_valid(uint16_t msg);
uint8_t  msg_le_ir(uint16_t msg);

/* ------------------------------------------------------------------------
 * mbf_scan_req: type:2 | centre:5 (7 bits).
 * --------------------------------------------------------------------- */
#define MSG_SR_TYPE_SHIFT       (5u)
#define MSG_SR_TYPE_MASK        (0x3u)
#define MSG_SR_CENTRE_MASK      (0x1Fu)

#define MSG_SCAN_COARSE         (0u)
#define MSG_SCAN_FINE           (1u)
#define MSG_SCAN_MONITOR        (2u)     /* RESUME_MONITOR                */
#define MSG_SCAN_HOLD           (3u)     /* Stop pinging during manoeuvres*/

uint8_t msg_sr_pack(uint8_t type, uint8_t centre_idx);
uint8_t msg_sr_type(uint8_t req);
uint8_t msg_sr_centre(uint8_t req);

/* ------------------------------------------------------------------------
 * mbf_tlm: src:3 then the source's snapshot, MSB-first.
 * --------------------------------------------------------------------- */
#define MSG_TLM_MAX_BYTES       (10u)
#define MSG_TLM_SRC_BITS        (3u)
#define MSG_TLM_NUM_SRC         (6u)

#define MSG_TLM_MOTION          (0u) /* st3 mode2 spdL10 spdR10 encL16
                                        encR16 odo16 = 73 bits            */
#define MSG_TLM_LINE            (1u) /* st3 bc2 ir3 err7 cmd2 dir1 new1
                                        rej4 = 23 bits                    */
#define MSG_TLM_IMU             (2u) /* st3 cls3 tilt1 hcur7 hmax7 n4
                                        pitch10 = 35 bits                 */
#define MSG_TLM_OBST            (3u) /* st4 range11 brg5 = 20 bits        */
#define MSG_TLM_NAV             (4u) /* sys4 = 4 bits                     */
#define MSG_TLM_EVENT           (5u) /* ev3 + MQTT event data <= 40 bits  */

/* MQTT event codes (3 bits), shared by mbf_tlm EVENT and is05/car/event. */
#define MSG_MQEV_BARCODE        (0u)   /* cmd:2 dir:1 sym:7 nocmd:1        */
#define MSG_MQEV_JUNCTION       (1u)   /* dist:16                          */
#define MSG_MQEV_HUMP           (2u)   /* h:7 n:4                          */
#define MSG_MQEV_OBSTACLE       (3u)   /* d:11                             */
#define MSG_MQEV_PROFILE        (4u)   /* brg5 d_min11 w8 cl8 cr8          */
#define MSG_MQEV_FAULT          (5u)   /* id:4 flt:14                      */
#define MSG_MQEV_MISSION_END    (6u)   /* peak7 n4 dist16 rej4             */

uint8_t msg_mqev_data_bits(uint8_t mq_ev);

#endif /* MSG_FMT_H */

/*** end of file ***/
