/** @file config.h
 *
 * @brief Runtime configuration words g_config A and B (Section 1.5 area 6).
 *
 * Word A: kp:10 | ki:10 | kd:10 (each value x 100), bits 29..0.
 * Word B: base_speed - 100 :8 | obstacle_mm - 150 :8 | hump_pitch - 2 :4,
 *         bits 19..0.
 * Single writer: T6; every key in READY, and base_speed, kp, ki, kd also
 * while the mission runs (live tuning, 26 Oct). Readers copy a whole word
 * once per cycle, so a reader never sees a half-written word (NFR12
 * pattern 2); one MQTT set changes one gain.
 *
 * Owner: Integration; keys and ranges agreed with Buddy 1 (MQTT set/get).
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* MQTT set/get key codes (Design Review 1.4). */
#define CONFIG_KEY_BASE_SPEED   (0u)
#define CONFIG_KEY_KP           (1u)
#define CONFIG_KEY_KI           (2u)
#define CONFIG_KEY_KD           (3u)
#define CONFIG_KEY_OBSTACLE_MM  (4u)
#define CONFIG_KEY_HUMP_PITCH   (5u)
#define CONFIG_NUM_KEYS         (6u)

/* Word A fields. */
#define CONFIG_KP_SHIFT         (20u)
#define CONFIG_KI_SHIFT         (10u)
#define CONFIG_KD_SHIFT         (0u)
#define CONFIG_GAIN_MASK        (0x3FFu)

/* Word B fields. */
#define CONFIG_BASE_SHIFT       (12u)
#define CONFIG_BASE_MASK        (0xFFu)
#define CONFIG_OBST_SHIFT       (4u)
#define CONFIG_OBST_MASK        (0xFFu)
#define CONFIG_HUMP_SHIFT       (0u)
#define CONFIG_HUMP_MASK        (0xFu)

void     config_init_defaults(void);
bool     config_set_encoded(uint8_t key, uint16_t encoded_val);
uint16_t config_get_encoded(uint8_t key);
uint16_t config_max_encoded(uint8_t key);

uint32_t config_word_a(void);
uint32_t config_word_b(void);
uint16_t config_base_speed(uint32_t word_b);
uint16_t config_obstacle_mm(uint32_t word_b);
uint8_t  config_hump_pitch_deg(uint32_t word_b);
uint16_t config_gain_x100(uint32_t word_a, uint8_t shift);

#endif /* CONFIG_H */

/*** end of file ***/
