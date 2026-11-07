/** @file app_types.h
 *
 * @brief Project-wide basic types and compile-time checks.
 *
 * Barr C 5.4.b.i asks for the C99 float32_t name. C99 does not define it,
 * so it is defined here once for the whole project.
 *
 * Owner: Integration (all buddies).
 */

#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>
#include <stdbool.h>

typedef float float32_t;

/*
 * Compile-time assertion usable in C99 (static_assert is C11). A false
 * condition gives an array of negative size, which stops the build. It is a
 * macro because no function can run at compile time (Barr C 6.3.a).
 */
#define APP_STATIC_ASSERT(cond, name) \
    typedef uint8_t app_static_assert_##name##_t[(cond) ? 1 : -1]

#endif /* APP_TYPES_H */

/*** end of file ***/
