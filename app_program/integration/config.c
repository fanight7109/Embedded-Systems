/** @file config.c
 *
 * @brief Packed runtime configuration (single writer: T6 in READY).
 *
 * Owner: Integration.
 */

#include <stdint.h>
#include <stdbool.h>
#include "app_cfg.h"
#include "config.h"

#define BASE_OFFSET     (100u)
#define OBST_OFFSET     (150u)
#define HUMP_OFFSET     (2u)

/* Encoded maxima per key (Design Review 1.4 configuration table). */
static uint16_t const g_key_max[CONFIG_NUM_KEYS] =
{
    200u,   /* base_speed 100..300 -> v - 100 */
    1000u,  /* kp 0.00..10.00      -> v x 100 */
    1000u,  /* ki                              */
    1000u,  /* kd                              */
    250u,   /* obstacle_mm 150..400 -> v - 150 */
    8u      /* hump_pitch 2..10    -> v - 2    */
};

static volatile uint32_t g_config_a = 0u;
static volatile uint32_t g_config_b = 0u;

static uint32_t replace_field(uint32_t word, uint32_t value, uint32_t mask,
                              uint8_t shift);
static uint8_t  key_shift(uint8_t key);
static uint32_t key_mask(uint8_t key);

/*!
 * @brief Load the defaults from app_cfg.h. Called before tasks start.
 */
void
config_init_defaults (void)
{
    g_config_a = ((uint32_t)APP_DEF_KP_X100 << CONFIG_KP_SHIFT) |
                 ((uint32_t)APP_DEF_KI_X100 << CONFIG_KI_SHIFT) |
                 ((uint32_t)APP_DEF_KD_X100 << CONFIG_KD_SHIFT);
    g_config_b = ((uint32_t)(APP_DEF_BASE_SPEED - BASE_OFFSET)
                     << CONFIG_BASE_SHIFT) |
                 ((uint32_t)(APP_DEF_OBSTACLE_MM - OBST_OFFSET)
                     << CONFIG_OBST_SHIFT) |
                 ((uint32_t)(APP_DEF_HUMP_PITCH_DEG - HUMP_OFFSET)
                     << CONFIG_HUMP_SHIFT);
}

/*!
 * @brief Store one encoded value. Caller (T6) guarantees READY state.
 *
 * The new word is built in a local and published with one aligned store.
 *
 * @return false if the key is unknown or the value is out of range.
 */
bool
config_set_encoded (uint8_t key, uint16_t encoded_val)
{
    bool b_ok = false;

    if ((key < CONFIG_NUM_KEYS) && (encoded_val <= g_key_max[key]))
    {
        if ((CONFIG_KEY_KP <= key) && (CONFIG_KEY_KD >= key))
        {
            g_config_a = replace_field(g_config_a, encoded_val,
                                       CONFIG_GAIN_MASK, key_shift(key));
        }
        else
        {
            g_config_b = replace_field(g_config_b, encoded_val,
                                       key_mask(key), key_shift(key));
        }

        b_ok = true;
    }

    return b_ok;
}

/*!
 * @brief Read one encoded value (used for the MQTT get reply).
 */
uint16_t
config_get_encoded (uint8_t key)
{
    uint16_t value = 0u;

    if ((CONFIG_KEY_KP <= key) && (CONFIG_KEY_KD >= key))
    {
        value = config_gain_x100(g_config_a, key_shift(key));
    }
    else if (key < CONFIG_NUM_KEYS)
    {
        value = (uint16_t)((g_config_b >> key_shift(key)) & key_mask(key));
    }
    else
    {
        value = 0u;
    }

    return value;
}

uint16_t
config_max_encoded (uint8_t key)
{
    return (key < CONFIG_NUM_KEYS) ? g_key_max[key] : 0u;
}

uint32_t
config_word_a (void)
{
    return g_config_a;
}

uint32_t
config_word_b (void)
{
    return g_config_b;
}

uint16_t
config_base_speed (uint32_t word_b)
{
    return (uint16_t)(((word_b >> CONFIG_BASE_SHIFT) & CONFIG_BASE_MASK) +
                      BASE_OFFSET);
}

uint16_t
config_obstacle_mm (uint32_t word_b)
{
    return (uint16_t)(((word_b >> CONFIG_OBST_SHIFT) & CONFIG_OBST_MASK) +
                      OBST_OFFSET);
}

uint8_t
config_hump_pitch_deg (uint32_t word_b)
{
    return (uint8_t)(((word_b >> CONFIG_HUMP_SHIFT) & CONFIG_HUMP_MASK) +
                     HUMP_OFFSET);
}

uint16_t
config_gain_x100 (uint32_t word_a, uint8_t shift)
{
    return (uint16_t)((word_a >> shift) & CONFIG_GAIN_MASK);
}

static uint32_t
replace_field (uint32_t word, uint32_t value, uint32_t mask, uint8_t shift)
{
    return (word & ~(mask << shift)) | ((value & mask) << shift);
}

static uint8_t
key_shift (uint8_t key)
{
    uint8_t shift = 0u;

    switch (key)
    {
        case CONFIG_KEY_BASE_SPEED:
            shift = CONFIG_BASE_SHIFT;
        break;

        case CONFIG_KEY_KP:
            shift = CONFIG_KP_SHIFT;
        break;

        case CONFIG_KEY_KI:
            shift = CONFIG_KI_SHIFT;
        break;

        case CONFIG_KEY_KD:
            shift = CONFIG_KD_SHIFT;
        break;

        case CONFIG_KEY_OBSTACLE_MM:
            shift = CONFIG_OBST_SHIFT;
        break;

        default:
            shift = CONFIG_HUMP_SHIFT;
        break;
    }

    return shift;
}

static uint32_t
key_mask (uint8_t key)
{
    uint32_t mask = CONFIG_HUMP_MASK;

    if (CONFIG_KEY_BASE_SPEED == key)
    {
        mask = CONFIG_BASE_MASK;
    }
    else if (CONFIG_KEY_OBSTACLE_MM == key)
    {
        mask = CONFIG_OBST_MASK;
    }
    else
    {
        mask = CONFIG_HUMP_MASK;
    }

    return mask;
}

/*** end of file ***/
