/** @file ss1_comms.c
 *
 * @brief Subsystem 1 entry points: start T1 and run S1's on-target tests.
 *
 * S1 owns no peripheral: the radio, lwIP and MQTT belong to the kernel's
 * Wi-Fi service task (TEAM.md section 3). Its two message buffers are
 * created with the other kernel objects in rtos_obj_create().
 *
 * Owner: Buddy 1 (JONATHAN TAN JIANG MING).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "../integration/app_cfg.h"
#include "../app_main.h"
#include "telemetry.h"
#include "ss1_comms.h"

#if (1 == APP_TEST)
#include "../integration/test_log.h"
#include "bitpack.h"
#include "mqtt_fmt.h"
#endif

ER
ss1_comms_init (void)
{
    return E_OK;
}

ID
ss1_comms_start (void)
{
    return app_main_start_task(telemetry_task, APP_PRI_TELEMETRY,
                               APP_STK_TELEMETRY);
}

#if (1 == APP_TEST)

#define BAD_TOPIC           (7u)
#define TEST_VALUE          (0x5A3u)
#define TEST_WIDTH          (11u)

/*!
 * @brief S1 unit tests: frame header and bit packing (TEST_LOG.md).
 */
void
ss1_comms_test (void)
{
    uint8_t       buf[4]  = { 0u, 0u, 0u, 0u };
    uint16_t      pos     = 3u;
    uint8_t const hdr     = mqtt_fmt_out_header(MQTT_FMT_OUT_ACK, 1u, true);

    test_log_begin();

    test_log_check("S1-U01",
                   (1u == mqtt_fmt_out_qos(hdr)) && mqtt_fmt_out_retain(hdr),
                   "out header keeps QoS and retain");
    test_log_check("S1-U02", (NULL != mqtt_fmt_out_topic(hdr)),
                   "known topic index maps to a topic");
    test_log_check("S1-U03", (NULL == mqtt_fmt_out_topic(BAD_TOPIC)),
                   "unknown topic index is rejected");

    bitpack_put(buf, &pos, TEST_VALUE, TEST_WIDTH);
    test_log_check("S1-U04",
                   (TEST_VALUE == bitpack_get(buf, 3u, TEST_WIDTH)) &&
                   ((3u + TEST_WIDTH) == pos),
                   "bitpack round-trips an 11-bit field at offset 3");

    test_log_summary("subsystem1");
}

#endif /* APP_TEST */

/*** end of file ***/
