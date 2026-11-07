/** @file fault.h
 *
 * @brief Fault bitmask g_fault_mask (Design Review Section 1.3).
 *
 * Fault Fnn is bit (nn - 1). Critical bits latch SAFE STOP until reset;
 * F09 (IMU runtime) and F10 (Wi-Fi/MQTT) are non-critical and cleared by
 * their owners on recovery.
 *
 * Tribunal 1 change: writers serialise through a priority-ceiling mutex
 * (ceiling = T2 priority) instead of a DI/EI section. With the ceiling, a
 * holder runs at T2's priority, so T2 can wait only for one short
 * read-modify-write and never for a lower task's normal work. Only tasks
 * write the mask; no ISR calls fault_set().
 *
 * Owner: Integration (all buddies).
 */

#ifndef FAULT_H
#define FAULT_H

#include <stdint.h>
#include <stdbool.h>

#define FAULT_F01_ENCODER       (0u)
#define FAULT_F02_MANOEUVRE     (1u)
#define FAULT_F03_LINE_SEARCH   (2u)
#define FAULT_F04_SELF_TEST     (3u)
#define FAULT_F05_ULTRASONIC    (4u)
#define FAULT_F06_NO_BYPASS     (5u)
#define FAULT_F07_REACQUIRE     (6u)
#define FAULT_F08_COLLISION     (7u)
#define FAULT_F09_IMU           (8u)
#define FAULT_F10_NETWORK       (9u)
#define FAULT_F11_MBF_FULL      (10u)
#define FAULT_F12_DEADLINE      (11u)
#define FAULT_F13_ESTOP         (12u)
#define FAULT_F14_CMD_TIMEOUT   (13u)
#define FAULT_NUM_BITS          (14u)

#define FAULT_CRITICAL_MASK     (0x3CFFu)   /* All except F09 and F10.    */
#define FAULT_USED_MASK         (0x3FFFu)

void     fault_set(uint8_t fault_bit);
void     fault_clear(uint8_t fault_bit);
uint16_t fault_get_mask(void);
bool     fault_is_critical(void);
uint8_t  fault_first_id(uint16_t mask);

#endif /* FAULT_H */

/*** end of file ***/
