/** @file telemetry.c
 *
 * @brief T1: connection monitoring, telemetry, heartbeat, events, and
 *        validated command handling.
 *
 * Every 20 ms: read the radio and broker state, drain mbf_tlm into the
 * snapshot cache (queuing events), handle received commands, then queue
 * telemetry at 5 Hz and the heartbeat at 1 Hz, and send a waiting barcode
 * trace (APP_BC_TRACE_PER_CYCLE frames per cycle). Frames go to the kernel's
 * Wi-Fi service task through mbf_mqtt_out; that task publishes them
 * (mqtt_app.c). T1 itself never calls cyw43, lwIP or MQTT (TEAM.md 3).
 *
 * Frames are queued only while ONLINE; offline they are dropped, because
 * telemetry is never in a control path and stale frames have no value.
 * Reconnection (every 3 s) is done by the kernel's MQTT service.
 *
 * Owner: Buddy 1 (JONATHAN TAN JIANG MING).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "../integration/app_cfg.h"
#include "../integration/app_types.h"
#include "../integration/board.h"
#include "../integration/msg_fmt.h"
#include "../integration/config.h"
#include "../integration/fault.h"
#include "../integration/rtos_obj.h"
#include "../integration/sys_state.h"
#include "bitpack.h"
#include "mqtt_fmt.h"
#include "mqtt_app.h"
#include "telemetry.h"
#include "../line_barcode/ir_line.h"
#include "../line_barcode/bc_sensor.h"
#include "../evidence/evidence.h"

#if (1 == TM_WIFI_CYW43)
#include "cyw43_utk.h"
#endif

#define TICK_DIV_MS         (10u)       /* 10 ms telemetry ticks.         */
#define QOS0                (0u)
#define QOS1                (1u)
#define RCN_MAX             (15u)
#define NO_ECHO             (2047u)
#define LINK_UP             (3)         /* CYW43_LINK_UP (cyw43.h).       */
#define TRACE_CHUNKS        ((APP_BC_TRACE_BINS + MQTT_TRACE_PER_FRAME - 1u) \
                             / MQTT_TRACE_PER_FRAME)

/* Task-private context (rule R4 for the flags). */
typedef struct
{
    uint32_t comm_state  : 3;
    uint32_t reconnects  : 4;
    uint32_t b_was_online: 1;
    uint32_t reserved    : 24;
} tlm_flags_t;

APP_STATIC_ASSERT(sizeof(tlm_flags_t) == 4u, tlm_flags_size);

/* DIAG keys are passed to bc_sensor_diag() unchanged. */
APP_STATIC_ASSERT(MQTT_DIAG_BC_RAW == BC_DIAG_RAW, diag_raw);
APP_STATIC_ASSERT(MQTT_DIAG_BC_CAL_WHITE == BC_DIAG_CAL_WHITE, diag_white);
APP_STATIC_ASSERT(MQTT_DIAG_BC_CAL_BLACK == BC_DIAG_CAL_BLACK, diag_black);
APP_STATIC_ASSERT(MQTT_DIAG_BC_BLACK_ON == BC_DIAG_BLACK_ON, diag_on);
APP_STATIC_ASSERT(MQTT_DIAG_BC_WHITE_ON == BC_DIAG_WHITE_ON, diag_off);
APP_STATIC_ASSERT(MQTT_DIAG_BC_TRACE == BC_DIAG_TRACE, diag_trace);
APP_STATIC_ASSERT(MQTT_NUM_DIAG_KEYS == BC_DIAG_COUNT, diag_count);
APP_STATIC_ASSERT(TRACE_CHUNKS <= 255u, trace_chunks_fit);

typedef struct
{
    tlm_flags_t     flags;
    mqtt_fmt_tlm_t  cache;
    mqtt_fmt_seq_t  seq;
    uint32_t        last_tlm_ms;
    uint32_t        last_hb_ms;
    uint16_t        last_faults;
    uint8_t         sys;
    uint8_t         motion_st;
    uint8_t         mode;
    uint8_t         line_st;
    uint8_t         bc_st;
    uint8_t         obs_st;
    uint8_t         imu_st;
    uint8_t         trace_chunk;   /* Next frame of the trace waiting.  */
} tlm_ctx_t;

static tlm_ctx_t g_tlm;

static void     update_connection(tlm_ctx_t * p_ctx);
static uint8_t  link_state(void);
static void     drain_snapshots(tlm_ctx_t * p_ctx);
static void     absorb(tlm_ctx_t * p_ctx, uint8_t const * p_msg);
static void     drain_commands(tlm_ctx_t * p_ctx);
static void     handle_command(tlm_ctx_t * p_ctx, uint8_t const * p_buf,
                               uint8_t len);
static void     publish_periodic(tlm_ctx_t * p_ctx, uint32_t now_ms);
static void     publish_fault_change(tlm_ctx_t * p_ctx);
static void     publish_trace(tlm_ctx_t * p_ctx);
static bool     publish(tlm_ctx_t * p_ctx, uint8_t topic,
                        uint8_t const * p_data, uint8_t len, uint8_t qos);
static void     forward_to_nav(uint8_t event, uint8_t key, uint16_t val);
static uint32_t now_ticks(void);

/*!
 * @brief T1 body, every 20 ms at the lowest application priority.
 */
void
telemetry_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    g_tlm.cache.obs_d = NO_ECHO;
    g_tlm.sys         = SYS_ST_INIT;

    for (;;)
    {
        (void)tk_dly_tsk((RELTIM)APP_TLM_POLL_MS);
        update_connection(&g_tlm);
        drain_snapshots(&g_tlm);
        drain_commands(&g_tlm);
        publish_periodic(&g_tlm, board_time_ms());
        publish_trace(&g_tlm);
    }
}

/*!
 * @brief Track radio and broker state; F10 while the link is lost
 *        (non-critical, NFR5).
 */
static void
update_connection (tlm_ctx_t * p_ctx)
{
    uint8_t const state = link_state();

    if (TELEMETRY_ST_ONLINE == state)
    {
        if (0u == p_ctx->flags.b_was_online)
        {
            fault_clear(FAULT_F10_NETWORK);
        }

        p_ctx->flags.b_was_online = 1u;
    }
    else if (0u != p_ctx->flags.b_was_online)
    {
        fault_set(FAULT_F10_NETWORK);
        p_ctx->flags.b_was_online = 0u;
        p_ctx->flags.reconnects = (p_ctx->flags.reconnects < RCN_MAX) ?
                                  (p_ctx->flags.reconnects + 1u) : RCN_MAX;
    }
    else
    {
        /* Still offline. */
    }

    p_ctx->flags.comm_state = state & 0x7u;
    evidence_link(state);                               /* INT-09 */
}

/*!
 * @brief Current communication state from the kernel's radio status and
 *        the MQTT bridge.
 */
static uint8_t
link_state (void)
{
    uint8_t state = TELEMETRY_ST_INIT;

#if (1 == TM_WIFI_CYW43)
    T_CYW43_UTK_STATUS status;

    cyw43_utk_get_status(&status);

    if (0u == status.ready)
    {
        state = TELEMETRY_ST_WIFI_JOIN;
    }
    else if (LINK_UP != status.link_status)
    {
        state = TELEMETRY_ST_WIFI_WAIT;
    }
    else if (!mqtt_app_is_connected())
    {
        state = TELEMETRY_ST_MQTT_WAIT;
    }
    else
    {
        state = TELEMETRY_ST_ONLINE;
    }
#endif

    return state;
}

/*!
 * @brief Absorb every queued snapshot; events are queued as they come.
 */
static void
drain_snapshots (tlm_ctx_t * p_ctx)
{
    uint8_t msg[MSG_TLM_MAX_BYTES];

    while (rtos_obj_tlm_recv(msg, TMO_POL) > 0)
    {
        absorb(p_ctx, msg);
    }
}

/*!
 * @brief Copy one snapshot's fields into the telemetry cache.
 */
static void
absorb (tlm_ctx_t * p_ctx, uint8_t const * p_msg)
{
    uint8_t const src = (uint8_t)bitpack_get(p_msg, 0u, MSG_TLM_SRC_BITS);
    uint16_t      pos = MSG_TLM_SRC_BITS;

    switch (src)
    {
        case MSG_TLM_MOTION:
            p_ctx->motion_st     = (uint8_t)bitpack_get(p_msg, pos, 3u);
            p_ctx->mode          = (uint8_t)bitpack_get(p_msg, pos + 3u, 2u);
            p_ctx->cache.spd_l   = (uint16_t)bitpack_get(p_msg, pos + 5u, 10u);
            p_ctx->cache.spd_r   = (uint16_t)bitpack_get(p_msg, pos + 15u,
                                                         10u);
            p_ctx->cache.enc_l   = (uint16_t)bitpack_get(p_msg, pos + 25u,
                                                         16u);
            p_ctx->cache.enc_r   = (uint16_t)bitpack_get(p_msg, pos + 41u,
                                                         16u);
            p_ctx->cache.dist_mm = (uint16_t)bitpack_get(p_msg, pos + 57u,
                                                         16u);
        break;

        case MSG_TLM_LINE:
            p_ctx->line_st       = (uint8_t)bitpack_get(p_msg, pos, 3u);
            p_ctx->bc_st         = (uint8_t)bitpack_get(p_msg, pos + 3u, 2u);
            p_ctx->cache.ir_bits = (uint8_t)bitpack_get(p_msg, pos + 5u, 3u);
            p_ctx->cache.err     = (uint8_t)bitpack_get(p_msg, pos + 8u, 7u);
            p_ctx->cache.bc_cmd  = (uint8_t)bitpack_get(p_msg, pos + 15u,
                                                        2u);
            p_ctx->cache.bc_dir  = (uint8_t)bitpack_get(p_msg, pos + 17u,
                                                        1u);
        break;

        case MSG_TLM_IMU:
            p_ctx->imu_st         = (uint8_t)bitpack_get(p_msg, pos, 3u);
            p_ctx->cache.hump_cur = (uint8_t)bitpack_get(p_msg, pos + 7u,
                                                         7u);
            p_ctx->cache.hump_max = (uint8_t)bitpack_get(p_msg, pos + 14u,
                                                         7u);
        break;

        case MSG_TLM_OBST:
            p_ctx->obs_st        = (uint8_t)bitpack_get(p_msg, pos, 4u);
            p_ctx->cache.obs_d   = (uint16_t)bitpack_get(p_msg, pos + 4u,
                                                         11u);
            p_ctx->cache.obs_brg = (uint8_t)bitpack_get(p_msg, pos + 15u, 5u);
        break;

        case MSG_TLM_NAV:
            p_ctx->sys = (uint8_t)bitpack_get(p_msg, pos, 4u);
        break;

        case MSG_TLM_EVENT:
        {
            uint8_t       frame[MQTT_EVENT_MAX_BYTES];
            uint8_t const event = (uint8_t)bitpack_get(p_msg, pos, 3u);
            uint8_t const len   = mqtt_fmt_pack_event(now_ticks(), event,
                                                      p_msg,
                                                      (uint16_t)(pos + 3u),
                                                      frame);

            (void)publish(p_ctx, MQTT_FMT_OUT_EVENT, frame, len, QOS1);
        }
        break;

        default:
            /* Unknown source: ignore. */
        break;
    }
}

/*!
 * @brief Handle every command payload the network task received.
 */
static void
drain_commands (tlm_ctx_t * p_ctx)
{
    uint8_t buf[RTOS_MQ_IN_MAX];
    uint8_t len = rtos_obj_mq_in_recv(buf);

    while (0u != len)
    {
        handle_command(p_ctx, buf, len);
        len = rtos_obj_mq_in_recv(buf);
    }
}

/*!
 * @brief Validate, act on and acknowledge one command (FR13, FR9).
 *
 * ESTOP sets flg_sys directly (never queued, NFR9). START, STOP and SET go
 * to T6 through mbf_nav. GET is answered here from g_config (or the IR
 * contrast), DIAG from the barcode sensor (bc_sensor_diag()).
 */
static void
handle_command (tlm_ctx_t * p_ctx, uint8_t const * p_buf, uint8_t len)
{
    mqtt_fmt_cmd_t cmd;
    uint8_t        ack[MQTT_ACK_BYTES];
    uint16_t       val = 0u;
    uint8_t const  err = mqtt_fmt_check_cmd(p_buf, len, p_ctx->sys,
                                            &p_ctx->seq, &cmd);

    if (MQTT_ERR_NONE == err)
    {
        switch (cmd.cmd)
        {
            case MQTT_CMD_START:
                /* val > 0: straight barcode test drive of val mm. */
                forward_to_nav(MSG_EV_CMD_START, 0u, cmd.val);
            break;

            case MQTT_CMD_STOP:
                forward_to_nav(MSG_EV_CMD_STOP, 0u, 0u);
            break;

            case MQTT_CMD_ESTOP:
                /* Evidence build: stamps the trial and, if "ev fill" armed
                 * it, fills mbf_motion and sets ESTOP atomically. */
                evidence_stop_trigger(EV_SRC_ESTOP_MQTT);   /* INT-10 */
                rtos_obj_sys_set(RTOS_SYS_ESTOP);
                fault_set(FAULT_F13_ESTOP);
            break;

            case MQTT_CMD_SET:
                forward_to_nav(MSG_EV_CMD_SET, cmd.key, cmd.val);
                val = cmd.val;
            break;

            case MQTT_CMD_DIAG:
                val = bc_sensor_diag(cmd.key);
            break;

            case MQTT_CMD_EVID:
                /* Accepted by mqtt_fmt_check_cmd() only in EVIDENCE=1
                 * builds (INT-09 .. INT-15). */
                val = evidence_command(cmd.key, cmd.val);
            break;

            default:
                if (MQTT_KEY_IR_CAL_L == cmd.key)
                {
                    val = (uint16_t)(ir_line_cal_contrast(IR_SENSOR_L) /
                                     MQTT_IR_CAL_DIV);
                }
                else if (MQTT_KEY_IR_CAL_C == cmd.key)
                {
                    val = (uint16_t)(ir_line_cal_contrast(IR_SENSOR_C) /
                                     MQTT_IR_CAL_DIV);
                }
                else
                {
                    val = config_get_encoded(cmd.key);
                }
            break;
        }
    }

    (void)mqtt_fmt_pack_ack(cmd.seq, (MQTT_ERR_NONE == err), err, cmd.key,
                            val, ack);
    (void)publish(p_ctx, MQTT_FMT_OUT_ACK, ack, MQTT_ACK_BYTES, QOS1);
}

/*!
 * @brief 5 Hz telemetry, 1 Hz heartbeat, and FAULT events on change.
 */
static void
publish_periodic (tlm_ctx_t * p_ctx, uint32_t now_ms)
{
    uint8_t frame[MQTT_TLM_BYTES];

    publish_fault_change(p_ctx);

    if ((now_ms - p_ctx->last_tlm_ms) >= APP_TLM_PERIOD_MS)
    {
        uint8_t len = 0u;

        p_ctx->last_tlm_ms      = now_ms;
        p_ctx->cache.t_ticks    = now_ticks();
        p_ctx->cache.flt        = fault_get_mask();
        p_ctx->cache.drop       = rtos_obj_tlm_drops();
        p_ctx->cache.state_word = mqtt_fmt_state_word(
            p_ctx->sys, p_ctx->motion_st, p_ctx->mode, p_ctx->line_st,
            p_ctx->bc_st, p_ctx->obs_st, p_ctx->imu_st,
            (uint8_t)p_ctx->flags.comm_state);
        len = mqtt_fmt_pack_telemetry(&p_ctx->cache, frame);

        if (publish(p_ctx, MQTT_FMT_OUT_TELEMETRY, frame, len, QOS0))
        {
            evidence_tlm_sent();                        /* INT-09 */
        }
    }

    if ((now_ms - p_ctx->last_hb_ms) >= APP_HB_PERIOD_MS)
    {
        uint8_t const len = mqtt_fmt_pack_heartbeat(
            p_ctx->sys, now_ticks(), fault_is_critical(),
            (uint8_t)p_ctx->flags.reconnects, frame);

        p_ctx->last_hb_ms = now_ms;
        (void)publish(p_ctx, MQTT_FMT_OUT_HEARTBEAT, frame, len, QOS0);
    }
}

/*!
 * @brief Publish a FAULT event (id:4 + flt:14) when new fault bits appear.
 */
static void
publish_fault_change (tlm_ctx_t * p_ctx)
{
    uint16_t const mask    = fault_get_mask();
    uint16_t const new_bit = (uint16_t)(mask & (uint16_t)~p_ctx->last_faults);

    if (0u != new_bit)
    {
        uint8_t  data[3] = { 0u, 0u, 0u };
        uint8_t  frame[MQTT_EVENT_MAX_BYTES];
        uint16_t pos = 0u;
        uint8_t  len = 0u;

        bitpack_put(data, &pos, fault_first_id(new_bit), 4u);
        bitpack_put(data, &pos, mask, 14u);
        len = mqtt_fmt_pack_event(now_ticks(), MSG_MQEV_FAULT, data, 0u,
                                  frame);
        (void)publish(p_ctx, MQTT_FMT_OUT_EVENT, frame, len, QOS1);
    }

    p_ctx->last_faults = mask;
}

/*!
 * @brief Send the barcode trace copy, APP_BC_TRACE_PER_CYCLE frames per
 *        cycle, then hand it back to T3. A frame the full queue refuses
 *        is offered again next cycle; offline the trace is skipped.
 */
static void
publish_trace (tlm_ctx_t * p_ctx)
{
    uint8_t                trace_id = 0u;
    uint16_t const * const p_bins   = bc_sensor_trace_get(&trace_id);

    if (NULL != p_bins)
    {
        bool const b_online = (TELEMETRY_ST_ONLINE ==
                               p_ctx->flags.comm_state);
        bool       b_room   = true;
        uint8_t    frame[MQTT_TRACE_BYTES];

        for (uint8_t sent = 0u; (sent < APP_BC_TRACE_PER_CYCLE) && b_room &&
             (p_ctx->trace_chunk < TRACE_CHUNKS); sent++)
        {
            uint16_t const first = (uint16_t)((uint16_t)p_ctx->trace_chunk *
                                              MQTT_TRACE_PER_FRAME);
            uint16_t const left  = (uint16_t)(APP_BC_TRACE_BINS - first);
            /* Cast: at most MQTT_TRACE_PER_FRAME (11). */
            uint8_t const  count = (uint8_t)((left < MQTT_TRACE_PER_FRAME) ?
                                             left : MQTT_TRACE_PER_FRAME);
            uint8_t const  len   = mqtt_fmt_pack_trace(
                trace_id, p_ctx->trace_chunk, (uint8_t)TRACE_CHUNKS,
                bc_sensor_trace_bin_um(), &p_bins[first], count, frame);

            /* Offline: skip through the trace. Queue full: same frame
             * again next cycle. */
            b_room = (!b_online) ||
                     publish(p_ctx, MQTT_FMT_OUT_BCTRACE, frame, len, QOS0);

            if (b_room)
            {
                p_ctx->trace_chunk++;
            }
        }

        if (p_ctx->trace_chunk >= TRACE_CHUNKS)
        {
            p_ctx->trace_chunk = 0u;
            bc_sensor_trace_done();
        }
    }
}

/*!
 * @brief Queue one frame for the network task, only while ONLINE.
 */
static bool
publish (tlm_ctx_t * p_ctx, uint8_t topic, uint8_t const * p_data,
         uint8_t len, uint8_t qos)
{
    bool b_queued = false;

    if ((TELEMETRY_ST_ONLINE == p_ctx->flags.comm_state) && (0u != len) &&
        (len < RTOS_MQ_OUT_MAX))
    {
        uint8_t out[RTOS_MQ_OUT_MAX];

        out[0] = mqtt_fmt_out_header(topic, qos, false);

        for (uint8_t idx = 0u; idx < len; idx++)
        {
            out[idx + 1u] = p_data[idx];
        }

        /* A full queue drops the frame (best effort); the caller may
         * retry (the barcode trace does). */
        b_queued = rtos_obj_mq_out_send(out, (uint8_t)(len + 1u));
    }

    return b_queued;
}

static void
forward_to_nav (uint8_t event, uint8_t key, uint16_t val)
{
    msg_nav_t msg = { .event = event, .small_a = key, .value = val };
    uint8_t   buf[MSG_NAV_MAX_BYTES];
    uint8_t   len = msg_nav_pack(&msg, buf);

    rtos_obj_nav_send(buf, len);
}

/*!
 * @brief 19-bit telemetry time in 10 ms ticks.
 */
static uint32_t
now_ticks (void)
{
    return (board_time_ms() / TICK_DIV_MS) & MQTT_TIME_MASK;
}

/*** end of file ***/
