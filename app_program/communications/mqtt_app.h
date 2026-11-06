/** @file mqtt_app.h
 *
 * @brief Application hooks called by the kernel's MQTT service.
 *
 * TEAM.md section 3: no application task may call cyw43, lwIP or MQTT
 * functions; only the kernel's Wi-Fi service task (cyw43_utk_task) may.
 * The template's mqtt_utk.c therefore calls these hooks from inside that
 * task (weak no-op defaults live in mqtt_utk.c). The hooks only move bytes
 * between the service task and T1 through two message buffers, always
 * with TMO_POL, so the service task never blocks on the application.
 *
 *   mbf_mqtt_out: T1 -> service task (binary frames, byte 0 = header)
 *   mbf_mqtt_in:  service task -> T1 (raw is05/car/cmd payloads)
 *
 * Only mqtt_app_is_connected() is called by T1 and T6 (boot
 * settle); every other function runs in the service task. The MQTT
 * hooks are built only with MQTT=1.
 *
 * Owner: Buddy 1 (JONATHAN TAN JIANG MING).
 */

#ifndef MQTT_APP_H
#define MQTT_APP_H

#include <stdint.h>
#include <stdbool.h>

/* Called by T1 and T6. */
bool         mqtt_app_is_connected(void);

/* Called by mqtt_utk.c in the service task (prototypes match it). */
int32_t      mqtt_app_enabled(void);
char const * mqtt_app_sub_topic(void);
char const * mqtt_app_will_topic(void);
uint16_t     mqtt_app_keepalive_s(void);
void         mqtt_app_on_status(int32_t connected);
void         mqtt_app_poll(void);
void         mqtt_app_on_message(char const * p_topic, uint8_t const * p_data,
                                 uint16_t len);

#endif /* MQTT_APP_H */

/*** end of file ***/
