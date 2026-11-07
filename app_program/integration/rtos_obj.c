/** @file rtos_obj.c
 *
 * @brief Creation of all kernel objects and their access wrappers.
 *
 * Message-buffer sizes follow the RTOS object table in Design Review
 * Section 4: capacity x (largest message rounded to a word + the kernel's
 * per-message size header). Only the packed length is ever sent.
 *
 * Owner: Integration (all buddies).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "board.h"
#include "app_cfg.h"
#include "msg_fmt.h"
#include "fault.h"
#include "button.h"
#include "../evidence/evidence.h"
#include "rtos_obj.h"

#define NAV_CAPACITY        (16)
#define MOTION_CAPACITY     (4)
#define LINE_CAPACITY       (2)
#define SCAN_CAPACITY       (2)
#define TLM_CAPACITY        (16)
#define MBF_HEADER_BYTES    (4)     /* Kernel stores an INT size per msg. */
#define WORD_BYTES          (4)
#define SEND_TMO_MS         (5)     /* mbf_motion_cmd and mbf_scan_req.   */
#define FAULT_LOCK_TMO_MS   (2)
#define SYS_ALL_BITS        (0x7u)
#define DROP_SAT            (15u)
#define NUM_RELEASES        (3u)
#define MQTT_OUT_CAPACITY   (16)
#define MQTT_IN_CAPACITY    (4)

static ID gh_mbf_nav     = 0;
static ID gh_mbf_motion  = 0;
static ID gh_mbf_line    = 0;
static ID gh_mbf_scan    = 0;
static ID gh_mbf_tlm     = 0;
static ID gh_flg_sys     = 0;
static ID gh_flg_echo    = 0;
static ID gh_mtx_fault   = 0;
static ID gh_cyc_ctrl    = 0;
static ID gh_cyc_line    = 0;
static ID gh_cyc_imu     = 0;
static ID gh_mbf_mq_out  = 0;
static ID gh_mbf_mq_in   = 0;
static ID gh_tsk_motion  = 0;
static ID gh_tsk_line    = 0;
static ID gh_tsk_imu     = 0;

/*
 * Release timestamps, written only by the cyclic handlers (single writer,
 * one aligned 32-bit store) and read by T2 and T3 (NFR12 pattern 2).
 */
static volatile uint32_t g_release_us[NUM_RELEASES] = { 0u, 0u, 0u };

/* F12 check armed after boot settle: written once by T6, read by T2/T3. */
static volatile bool gb_deadline_armed = false;

/* Telemetry drop counters, one per source; each written by one task. */
static volatile uint8_t g_tlm_drop[MSG_TLM_NUM_SRC] = { 0u };

static ID   create_mbf(INT capacity, INT max_msg_bytes);
static void cyc_ctrl_isr(void * p_exinf);
static void cyc_line_isr(void * p_exinf);
static void cyc_imu_isr(void * p_exinf);

/*!
 * @brief Create every message buffer, event flag and the fault mutex.
 *
 * Called once by the init task before any application task starts.
 *
 * @return E_OK, or the first kernel error.
 */
ER
rtos_obj_create (void)
{
    ER     err  = E_OK;
    T_CFLG cflg = { .exinf = NULL, .flgatr = TA_TFIFO | TA_WMUL,
                    .iflgptn = 0u };
    T_CMTX cmtx = { .exinf = NULL, .mtxatr = TA_CEILING,
                    .ceilpri = APP_PRI_MOTION };

    gh_mbf_nav    = create_mbf(NAV_CAPACITY, (INT)MSG_NAV_MAX_BYTES);
    gh_mbf_motion = create_mbf(MOTION_CAPACITY, (INT)sizeof(uint32_t));
    gh_mbf_line   = create_mbf(LINE_CAPACITY, (INT)sizeof(uint16_t));
    gh_mbf_scan   = create_mbf(SCAN_CAPACITY, (INT)sizeof(uint8_t));
    gh_mbf_tlm    = create_mbf(TLM_CAPACITY, (INT)MSG_TLM_MAX_BYTES);
    gh_mbf_mq_out = create_mbf(MQTT_OUT_CAPACITY, (INT)RTOS_MQ_OUT_MAX);
    gh_mbf_mq_in  = create_mbf(MQTT_IN_CAPACITY, (INT)RTOS_MQ_IN_MAX);
    gh_flg_sys    = tk_cre_flg(&cflg);

    cflg.flgatr   = TA_TFIFO | TA_WSGL;
    gh_flg_echo   = tk_cre_flg(&cflg);
    gh_mtx_fault  = tk_cre_mtx(&cmtx);

    if ((gh_mbf_nav < 0) || (gh_mbf_motion < 0) || (gh_mbf_line < 0) ||
        (gh_mbf_scan < 0) || (gh_mbf_tlm < 0) || (gh_flg_sys < 0) ||
        (gh_flg_echo < 0) || (gh_mtx_fault < 0) || (gh_mbf_mq_out < 0) ||
        (gh_mbf_mq_in < 0))
    {
        err = E_SYS;
    }

    return err;
}

/*!
 * @brief Create and start the two cyclic handlers that release T2 and T3.
 *
 * cyc_ctrl: 10 ms -> T2. cyc_line: 20 ms -> T3. cyc_imu: 20 ms -> T4.
 * Periods are whole multiples of the port's 10 ms kernel tick.
 */
ER
rtos_obj_start_cyclic (ID motion_tskid, ID line_tskid, ID imu_tskid)
{
    ER     err  = E_OK;
    T_CCYC ccyc = { .exinf = NULL, .cycatr = TA_HLNG | TA_STA | TA_PHS,
                    .cychdr = NULL, .cyctim = (RELTIM)APP_CTRL_PERIOD_MS,
                    .cycphs = 0u };

    gh_tsk_motion = motion_tskid;
    gh_tsk_line   = line_tskid;
    gh_tsk_imu    = imu_tskid;

    /* Cast: the kernel stores handlers as the generic FP type. */
    ccyc.cychdr = (FP)cyc_ctrl_isr;
    gh_cyc_ctrl = tk_cre_cyc(&ccyc);

    ccyc.cychdr = (FP)cyc_line_isr;
    ccyc.cyctim = (RELTIM)APP_LINE_PERIOD_MS;
    ccyc.cycphs = (RELTIM)APP_LINE_PHASE_MS;
    gh_cyc_line = tk_cre_cyc(&ccyc);

    ccyc.cychdr = (FP)cyc_imu_isr;
    ccyc.cyctim = (RELTIM)APP_IMU_PERIOD_MS;
    ccyc.cycphs = 0u;
    gh_cyc_imu  = tk_cre_cyc(&ccyc);

    if ((gh_cyc_ctrl < 0) || (gh_cyc_line < 0) || (gh_cyc_imu < 0))
    {
        err = E_SYS;
    }

    return err;
}

/*!
 * @brief Time (board_time_us) of the latest release of T2 or T3.
 */
uint32_t
rtos_obj_release_us (uint8_t which)
{
    uint32_t stamp = 0u;

    if (which < NUM_RELEASES)
    {
        stamp = g_release_us[which];
    }

    return stamp;
}

/*!
 * @brief F12 check for one cycle of a periodic task.
 *
 * Always false until T6 arms the check at the end of the boot settle
 * (APP_BOOT_SETTLE_MS), so the WiFi start-up cannot raise F12.
 *
 * @return true if the task started more than 2 ms after its release
 *         (NFR1) or finished after its next release.
 */
bool
rtos_obj_missed_deadline (uint32_t release_us, uint32_t start_us,
                          uint32_t end_us, uint32_t period_us)
{
    uint32_t const latency = start_us - release_us;   /* Wrap-safe. */
    uint32_t const elapsed = end_us - release_us;

    return gb_deadline_armed &&
           ((latency > APP_JITTER_LIMIT_US) || (elapsed > period_us));
}

/*!
 * @brief Arm the F12 deadline check (T6, once, after the boot settle).
 */
void
rtos_obj_arm_deadlines (void)
{
    gb_deadline_armed = true;
}

void
rtos_obj_nav_send (uint8_t const * p_msg, uint8_t len)
{
    ER const err = tk_snd_mbf(gh_mbf_nav, p_msg, (INT)len, TMO_POL);

    if (E_OK != err)
    {
        fault_set(FAULT_F11_MBF_FULL);
    }
}

INT
rtos_obj_nav_recv (uint8_t * p_msg, TMO tmout)
{
    return tk_rcv_mbf(gh_mbf_nav, p_msg, tmout);
}

void
rtos_obj_motion_send (uint32_t cmd)
{
    ER const err = tk_snd_mbf(gh_mbf_motion, &cmd, (INT)sizeof(cmd),
                              (TMO)SEND_TMO_MS);

    if (E_OK != err)
    {
        fault_set(FAULT_F11_MBF_FULL);
    }
}

/*!
 * @brief Take the newest pending motion command; a newer one pre-empts.
 *
 * @return true if at least one command was waiting.
 */
bool
rtos_obj_motion_recv (uint32_t * p_cmd)
{
    bool     b_got = false;
    uint32_t cmd   = 0u;

    while (tk_rcv_mbf(gh_mbf_motion, &cmd, TMO_POL) > 0)
    {
        *p_cmd = cmd;
        b_got  = true;
    }

    return b_got;
}

void
rtos_obj_line_send (uint16_t msg)
{
    /* Full: the oldest is kept and this one dropped; a fresh one follows. */
    (void)tk_snd_mbf(gh_mbf_line, &msg, (INT)sizeof(msg), TMO_POL);
}

/*!
 * @brief Drain mbf_line_err and keep only the latest line error.
 */
bool
rtos_obj_line_recv_latest (uint16_t * p_msg)
{
    bool     b_got = false;
    uint16_t msg   = 0u;

    while (tk_rcv_mbf(gh_mbf_line, &msg, TMO_POL) > 0)
    {
        *p_msg = msg;
        b_got  = true;
    }

    return b_got;
}

/*!
 * @brief Send a scan request to T5. Dropped when the ultrasonic sensor is
 *        not fitted (T5 is not running, so the buffer would fill: F11).
 */
void
rtos_obj_scan_send (uint8_t req)
{
    if (0 != APP_ULTRASONIC_FITTED)
    {
        ER const err = tk_snd_mbf(gh_mbf_scan, &req, (INT)sizeof(req),
                                  (TMO)SEND_TMO_MS);

        if (E_OK != err)
        {
            fault_set(FAULT_F11_MBF_FULL);
        }
    }
}

bool
rtos_obj_scan_recv (uint8_t * p_req, TMO tmout)
{
    return (tk_rcv_mbf(gh_mbf_scan, p_req, tmout) > 0);
}

/*!
 * @brief Queue a telemetry snapshot; a full buffer only counts a drop.
 *
 * @param[in] src  MSG_TLM_* source; selects the single-writer counter.
 */
void
rtos_obj_tlm_send (uint8_t src, uint8_t const * p_msg, uint8_t len)
{
    ER const err = tk_snd_mbf(gh_mbf_tlm, p_msg, (INT)len, TMO_POL);

    if ((E_OK != err) && (src < MSG_TLM_NUM_SRC) &&
        (g_tlm_drop[src] < DROP_SAT))
    {
        g_tlm_drop[src] = (uint8_t)(g_tlm_drop[src] + 1u);
    }
}

INT
rtos_obj_tlm_recv (uint8_t * p_msg, TMO tmout)
{
    return tk_rcv_mbf(gh_mbf_tlm, p_msg, tmout);
}

/*!
 * @brief Total telemetry drops, saturating at 15 (drop:4 field).
 */
uint8_t
rtos_obj_tlm_drops (void)
{
    uint32_t total = 0u;

    for (uint8_t src = 0u; src < MSG_TLM_NUM_SRC; src++)
    {
        total += g_tlm_drop[src];
    }

    return (uint8_t)((total > DROP_SAT) ? DROP_SAT : total);
}

void
rtos_obj_sys_set (UINT pattern)
{
    (void)tk_set_flg(gh_flg_sys, pattern);
}

void
rtos_obj_sys_clear (UINT pattern)
{
    /* tk_clr_flg ANDs the pattern, so pass the complement. */
    (void)tk_clr_flg(gh_flg_sys, ~pattern);
}

/*!
 * @brief Poll flg_sys without waiting (T2 calls this every cycle).
 */
UINT
rtos_obj_sys_peek (void)
{
    UINT pattern = 0u;

    if (E_OK != tk_wai_flg(gh_flg_sys, SYS_ALL_BITS, TWF_ORW, &pattern,
                           TMO_POL))
    {
        pattern = 0u;
    }

    return pattern;
}

void
rtos_obj_echo_clear (void)
{
    (void)tk_clr_flg(gh_flg_echo, ~RTOS_ECHO_DONE);
}

bool
rtos_obj_echo_wait (TMO tmout)
{
    UINT pattern = 0u;

    return (E_OK == tk_wai_flg(gh_flg_echo, RTOS_ECHO_DONE,
                               TWF_ORW | TWF_CLR, &pattern, tmout));
}

/*!
 * @brief Set ECHO_DONE; safe from the GPIO interrupt.
 */
void
rtos_obj_echo_set (void)
{
    (void)tk_set_flg(gh_flg_echo, RTOS_ECHO_DONE);
}

bool
rtos_obj_fault_lock (void)
{
    return (E_OK == tk_loc_mtx(gh_mtx_fault, (TMO)FAULT_LOCK_TMO_MS));
}

void
rtos_obj_fault_unlock (void)
{
    (void)tk_unl_mtx(gh_mtx_fault);
}

/*!
 * @brief INT-10 only: fill mbf_motion_cmd with cmd (TMO_POL, no F11), so
 *        a stop can be triggered with commands queued in front of it.
 *
 * @return Commands queued (0 in a normal build).
 */
uint8_t
rtos_obj_ev_fill_motion (uint32_t cmd)
{
    uint8_t queued = 0u;

#if (0 != APP_EVIDENCE)
    bool b_room = true;

    for (uint8_t slot = 0u; (slot <= MOTION_CAPACITY) && b_room; slot++)
    {
        b_room = (E_OK == tk_snd_mbf(gh_mbf_motion, &cmd, (INT)sizeof(cmd),
                                     TMO_POL));

        if (b_room)
        {
            queued++;
        }
    }
#else
    (void)cmd;
#endif

    return queued;
}

/*!
 * @brief Queue one outgoing MQTT frame for the kernel's network task.
 *
 * @return false if the queue is full (the caller counts a drop).
 */
bool
rtos_obj_mq_out_send (uint8_t const * p_msg, uint8_t len)
{
    return (E_OK == tk_snd_mbf(gh_mbf_mq_out, p_msg, (INT)len, TMO_POL));
}

/*!
 * @brief Take one outgoing frame (network task only, never waits).
 *
 * @return Frame length, or 0 if none is waiting.
 */
uint8_t
rtos_obj_mq_out_recv (uint8_t * p_msg)
{
    INT const len = tk_rcv_mbf(gh_mbf_mq_out, p_msg, TMO_POL);

    return (len > 0) ? (uint8_t)len : 0u;
}

/*!
 * @brief Hand a received command payload to T1 (network task, TMO_POL).
 */
void
rtos_obj_mq_in_send (uint8_t const * p_msg, uint8_t len)
{
    (void)tk_snd_mbf(gh_mbf_mq_in, p_msg, (INT)len, TMO_POL);
}

/*!
 * @brief Take one received command payload (T1, never waits).
 */
uint8_t
rtos_obj_mq_in_recv (uint8_t * p_msg)
{
    INT const len = tk_rcv_mbf(gh_mbf_mq_in, p_msg, TMO_POL);

    return (len > 0) ? (uint8_t)len : 0u;
}

static ID
create_mbf (INT capacity, INT max_msg_bytes)
{
    INT const rounded = ((max_msg_bytes + (WORD_BYTES - 1)) / WORD_BYTES) *
                        WORD_BYTES;
    T_CMBF    cmbf    = { .exinf = NULL, .mbfatr = TA_TFIFO, .bufsz = 0,
                          .maxmsz = 0, .bufptr = NULL };

    cmbf.bufsz  = (SZ)(capacity * (rounded + MBF_HEADER_BYTES));
    cmbf.maxmsz = max_msg_bytes;

    return tk_cre_mbf(&cmbf);
}

/*!
 * @brief cyc_ctrl handler: timestamp the release, wake T2, and sample the
 *        GP20/GP21 buttons (two GPIO reads, button.c).
 *
 * Runs in the timer interrupt: no waits and no floating point (NFR10).
 */
static void
cyc_ctrl_isr (void * p_exinf)
{
    (void)p_exinf;
    g_release_us[RTOS_REL_MOTION] = board_time_us();
    (void)tk_wup_tsk(gh_tsk_motion);
    button_sample_isr();
    evidence_button(BUTTON_STOP);       /* INT-10: stamps a new press. */
}

/*!
 * @brief cyc_imu handler: timestamp the release and wake T4.
 */
static void
cyc_imu_isr (void * p_exinf)
{
    (void)p_exinf;
    g_release_us[RTOS_REL_IMU] = board_time_us();

    if (gh_tsk_imu > 0)             /* 0: IMU not fitted, no T4. */
    {
        (void)tk_wup_tsk(gh_tsk_imu);
    }
}

/*!
 * @brief cyc_line handler: timestamp the release and wake T3.
 */
static void
cyc_line_isr (void * p_exinf)
{
    (void)p_exinf;
    g_release_us[RTOS_REL_LINE] = board_time_us();
    (void)tk_wup_tsk(gh_tsk_line);
}

/*** end of file ***/
