/** @file app_main.h
 *
 * @brief Start-up shared by the whole team (template file app.h, renamed
 *        so header and source share a root name, Barr C 4.2.a).
 *
 * usermain() is the kernel's entry point (declared by the kernel). Each
 * subsystem module ssN_<name>.c provides:
 *   ssN_<name>_init()   driver set-up for the peripherals it owns
 *   ssN_<name>_start()  create and start its task, returns the task ID
 *   ssN_<name>_test()   on-target tests, TEST=1 builds only
 * The module names are unique in their first eight characters (Barr C
 * 4.1.b), which subsystemN.c would not be.
 *
 * Owner: Integration (changed only by team agreement).
 */

#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <tk/tkernel.h>

/* Task body type for every application task. */
typedef void (* app_main_task_fn_t)(INT stacd, void * p_exinf);

ID app_main_start_task(app_main_task_fn_t p_fn, PRI priority, SZ stack_bytes);

#endif /* APP_MAIN_H */

/*** end of file ***/
