/** @file telemetry.h
 *
 * @brief T1 Telemetry / MQTT task (Design Review 3 and 6.5; FR7, FR13).
 *
 * T1 is the lowest-priority task and the only network user. cyw43 runs in
 * poll mode inside T1, so all Wi-Fi and lwIP work happens here and can
 * always be pre-empted by T2..T6 (NFR3). Autonomous navigation never
 * depends on the network (NFR5, FR9).
 *
 * Owner: Buddy 1 (JONATHAN TAN JIANG MING).
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <tk/tkernel.h>

/* Communication states (7 values, 3 bits). */
#define TELEMETRY_ST_INIT       (0u)
#define TELEMETRY_ST_WIFI_JOIN  (1u)
#define TELEMETRY_ST_WIFI_WAIT  (2u)
#define TELEMETRY_ST_MQTT_CONN  (3u)
#define TELEMETRY_ST_MQTT_WAIT  (4u)
#define TELEMETRY_ST_ONLINE     (5u)
#define TELEMETRY_ST_BACKOFF    (6u)

void telemetry_task(INT stacd, void * p_exinf);

#endif /* TELEMETRY_H */

/*** end of file ***/
