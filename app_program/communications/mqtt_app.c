/** @file mqtt_app.c
 *
 * @brief MQTT bridge between T1 and the kernel's MQTT service task.
 *
 * Everything except mqtt_app_is_connected() runs inside cyw43_utk_task,
 * called from lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c. Each poll
 * publishes at most MAX_PER_POLL queued frames, so a backlog cannot
 * exhaust lwIP's MQTT output buffer in one go. A frame lwIP cannot take
 * yet (ERR_MEM: output buffer full while TCP waits for the broker's ACK)
 * is held and offered again next poll instead of being lost (29 Oct: 84
 * of 94 barcode trace frames were dropped this way).
 *
 * Owner: Buddy 1 (JONATHAN TAN JIANG MING).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "../integration/rtos_obj.h"
#include "../evidence/evidence.h"
#include "mqtt_fmt.h"
#include "mqtt_app.h"

#if (1 == TM_MQTT)
#include "mqtt_utk.h"
#endif

#define MAX_PER_POLL        (6u)
#define KEEPALIVE_S         (10u)       /* Design Review 1.4.              */
#define STATUS_ONLINE       (0x01u)
#define LWIP_ERR_MEM        (-1)        /* lwIP err_t ERR_MEM.             */

/* Written only by the service task; read by T1 (one aligned byte). */
static volatile bool gb_connected = false;

#if (1 == TM_MQTT)
/* Frame lwIP refused with ERR_MEM, retried next poll: service task only. */
static uint8_t g_hold[RTOS_MQ_OUT_MAX];
static uint8_t g_hold_len = 0u;

static bool topic_is(char const * p_topic, char const * p_expected);
#endif

/*!
 * @brief true while the broker connection is up (called by T1).
 */
bool
mqtt_app_is_connected (void)
{
    return gb_connected;
}

#if (1 == TM_MQTT)

/*!
 * @brief Tell mqtt_utk.c to use these hooks instead of its demo heartbeat.
 */
int32_t
mqtt_app_enabled (void)
{
    return 1;
}

char const *
mqtt_app_sub_topic (void)
{
    return MQTT_TOPIC_CMD;
}

/*!
 * @brief The broker publishes 0x00 (retained) here if the car vanishes.
 */
char const *
mqtt_app_will_topic (void)
{
    return MQTT_TOPIC_STATUS;
}

uint16_t
mqtt_app_keepalive_s (void)
{
    return KEEPALIVE_S;
}

/*!
 * @brief Connection change. On connect, publish status 0x01 (retained).
 */
void
mqtt_app_on_status (int32_t connected)
{
    gb_connected = (0 != connected);

    if (gb_connected)
    {
        uint8_t const online = STATUS_ONLINE;

        (void)mqtt_utk_publish(MQTT_TOPIC_STATUS, &online, 1u, 1u, 1u);
    }
}

/*!
 * @brief Publish frames queued by T1 (every 20 ms while connected).
 */
void
mqtt_app_poll (void)
{
    uint8_t frame[RTOS_MQ_OUT_MAX];
    uint8_t sent = 0u;
    uint8_t len  = g_hold_len;

    if (0u != len)
    {
        for (uint8_t idx = 0u; idx < len; idx++)
        {
            frame[idx] = g_hold[idx];
        }
    }
    else
    {
        len = rtos_obj_mq_out_recv(frame);
    }

    g_hold_len = 0u;

    while ((len > 1u) && (sent < MAX_PER_POLL))
    {
        char const * const p_topic = mqtt_fmt_out_topic(frame[0]);
        int32_t            err     = 0;

        if (NULL != p_topic)
        {
            err = mqtt_utk_publish(p_topic, &frame[1], (uint16_t)(len - 1u),
                                   mqtt_fmt_out_qos(frame[0]),
                                   mqtt_fmt_out_retain(frame[0]) ? 1u : 0u);
        }

        if (LWIP_ERR_MEM == err)
        {
            /* lwIP is full: keep this frame for the next poll. */
            for (uint8_t idx = 0u; idx < len; idx++)
            {
                g_hold[idx] = frame[idx];
            }

            g_hold_len = len;
            len        = 0u;
        }
        else
        {
            sent++;
            len = (sent < MAX_PER_POLL) ? rtos_obj_mq_out_recv(frame) : 0u;
        }
    }
}

/*!
 * @brief A complete inbound publish. Command payloads go to T1, which
 *        validates them; one byte past the 4-byte frame is kept so T1
 *        can still report BAD_LEN for an oversized command.
 */
void
mqtt_app_on_message (char const * p_topic, uint8_t const * p_data,
                     uint16_t len)
{
    if (topic_is(p_topic, MQTT_TOPIC_CMD))
    {
        uint8_t const keep = (len > RTOS_MQ_IN_MAX) ? (uint8_t)RTOS_MQ_IN_MAX
                                                    : (uint8_t)len;

        evidence_cmd_rx(p_data, len);       /* INT-10: ESTOP/STOP arrival. */

        if (0u != keep)
        {
            rtos_obj_mq_in_send(p_data, keep);
        }
    }
}

/*!
 * @brief String equality without <string.h> (its size_t clashes with the
 *        kernel's in files that include tk/tkernel.h).
 */
static bool
topic_is (char const * p_topic, char const * p_expected)
{
    uint8_t idx  = 0u;
    bool    b_eq = (NULL != p_topic);

    while (b_eq && ('\0' != p_expected[idx]))
    {
        b_eq = (p_topic[idx] == p_expected[idx]);
        idx++;
    }

    return b_eq && ('\0' == p_topic[idx]);
}

#endif /* TM_MQTT */

/*** end of file ***/
