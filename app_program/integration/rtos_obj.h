/** @file rtos_obj.h
 *
 * @brief Every micro T-Kernel object in the system and its access policy.
 *
 * One module owns all object IDs so the RTOS object table (Design Review
 * Section 4, NFR4) maps one-to-one onto this file: each wrapper applies the
 * capacity, timeout and full/timeout handling listed in that table, so no
 * task can forget a timeout or block where it must not.
 *
 * Owner: Integration (all buddies).
 */

#ifndef RTOS_OBJ_H
#define RTOS_OBJ_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

/* flg_sys bits (3 bits). ESTOP and FAULT are latched until reset. */
#define RTOS_SYS_ESTOP          (0x1u)
#define RTOS_SYS_FAULT          (0x2u)
#define RTOS_SYS_STOP_REQ       (0x4u)

/* flg_echo bit. */
#define RTOS_ECHO_DONE          (0x1u)

/* Periodic release identifiers for rtos_obj_release_us(). */
#define RTOS_REL_MOTION         (0u)
#define RTOS_REL_LINE           (1u)
#define RTOS_REL_IMU            (2u)

ER       rtos_obj_create(void);
ER       rtos_obj_start_cyclic(ID motion_tskid, ID line_tskid,
                               ID imu_tskid);
uint32_t rtos_obj_release_us(uint8_t which);
bool     rtos_obj_missed_deadline(uint32_t release_us, uint32_t start_us,
                                  uint32_t end_us, uint32_t period_us);
void     rtos_obj_arm_deadlines(void);

/* mbf_nav: T1..T5 -> T6, 6 B x 16, send TMO_POL, full is F11. */
void     rtos_obj_nav_send(uint8_t const * p_msg, uint8_t len);
INT      rtos_obj_nav_recv(uint8_t * p_msg, TMO tmout);

/* mbf_motion_cmd: T6 -> T2, 4 B x 4, send 5 ms, timeout is F11. */
void     rtos_obj_motion_send(uint32_t cmd);
bool     rtos_obj_motion_recv(uint32_t * p_cmd);

/* mbf_line_err: T3 -> T2, 2 B x 2, send TMO_POL, drop new if full. */
void     rtos_obj_line_send(uint16_t msg);
bool     rtos_obj_line_recv_latest(uint16_t * p_msg);

/* mbf_scan_req: T6 -> T5, 1 B x 2, send 5 ms, timeout is F11. */
void     rtos_obj_scan_send(uint8_t req);
bool     rtos_obj_scan_recv(uint8_t * p_req, TMO tmout);

/* mbf_tlm: T2..T6 -> T1, 10 B x 16, send TMO_POL, full only counted. */
void     rtos_obj_tlm_send(uint8_t src, uint8_t const * p_msg, uint8_t len);
INT      rtos_obj_tlm_recv(uint8_t * p_msg, TMO tmout);
uint8_t  rtos_obj_tlm_drops(void);

/*
 * mbf_mqtt_out: T1 -> kernel network task, 24 B x 16, both sides TMO_POL.
 * mbf_mqtt_in:  kernel network task -> T1, 9 B x 4, both sides TMO_POL.
 * Byte 0 of an outgoing frame is MQTT_FMT_OUT_* (topic, QoS, retain).
 */
#define RTOS_MQ_OUT_MAX         (24u)
#define RTOS_MQ_IN_MAX          (9u)

bool     rtos_obj_mq_out_send(uint8_t const * p_msg, uint8_t len);
uint8_t  rtos_obj_mq_out_recv(uint8_t * p_msg);
void     rtos_obj_mq_in_send(uint8_t const * p_msg, uint8_t len);
uint8_t  rtos_obj_mq_in_recv(uint8_t * p_msg);

/* flg_sys and flg_echo. */
void     rtos_obj_sys_set(UINT pattern);
void     rtos_obj_sys_clear(UINT pattern);
UINT     rtos_obj_sys_peek(void);
void     rtos_obj_echo_clear(void);
bool     rtos_obj_echo_wait(TMO tmout);
void     rtos_obj_echo_set(void);

/* mtx_fault: priority-ceiling mutex guarding g_fault_mask (Tribunal 1). */
bool     rtos_obj_fault_lock(void);
void     rtos_obj_fault_unlock(void);

/* Evidence builds only (INT-10): queue cmd into mbf_motion until it is
 * full, without raising F11. Returns the number queued. */
uint8_t  rtos_obj_ev_fill_motion(uint32_t cmd);

#endif /* RTOS_OBJ_H */

/*** end of file ***/
