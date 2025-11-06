/*
 * Lightweight shims to reduce editor diagnostics when full Betaflight
 * include paths are unavailable. Real builds with proper includes will
 * bypass these via __has_include checks.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifndef __has_include
#define __has_include(x) 0
#endif

// Platform FAST_* attribute fallbacks
#if !defined(FAST_DATA_ZERO_INIT)
#define FAST_DATA_ZERO_INIT
#endif
#if !defined(FAST_DATA)
#define FAST_DATA
#endif
#if !defined(FAST_CODE)
#define FAST_CODE
#endif
#if !defined(FAST_CODE_NOINLINE)
# if defined(__GNUC__)
#  define FAST_CODE_NOINLINE __attribute__((noinline))
# else
#  define FAST_CODE_NOINLINE
# endif
#endif

// Common unit-test/build attributes fallbacks
#ifndef STATIC_UNIT_TESTED
#define STATIC_UNIT_TESTED static
#endif

#ifndef NOINLINE
# if defined(__GNUC__)
#  define NOINLINE __attribute__((noinline))
# else
#  define NOINLINE
# endif
#endif

// Static assert fallback for non-C11 environments or missing macro
#ifndef STATIC_ASSERT
#define STATIC_ASSERT(condition) ((void)0)
#endif

// constrainf fallback
#ifndef HAVE_CONSTRAINF_FALLBACK
#define HAVE_CONSTRAINF_FALLBACK 1
static inline float constrainf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
#endif

// Axis constants fallbacks
#if __has_include("common/axis.h")
#include "common/axis.h"
#else
#ifndef XYZ_AXIS_COUNT
#define XYZ_AXIS_COUNT 3
#endif
#ifndef RP_AXIS_COUNT
#define RP_AXIS_COUNT 2
#endif
#ifndef FD_ROLL
#define FD_ROLL 0
#define FD_PITCH 1
#define FD_YAW 2
#endif
#ifndef AI_ROLL
#define AI_ROLL 0
#define AI_PITCH 1
#define AI_YAW 2
typedef int angle_index_t;
#endif
#endif

// Filter type fallbacks
#if __has_include("common/filter.h")
#include "common/filter.h"
#else
typedef struct { float _s; } pt1Filter_t;
typedef struct { float _bq[5]; } biquadFilter_t;
typedef struct { float _p2[5]; } pt2Filter_t;
typedef struct { float _p3[5]; } pt3Filter_t;
typedef float (*filterApplyFnPtr)(void *filter, float sample);
static inline float nullFilterApply(void *filter, float sample) { (void)filter; return sample; }
#endif

// Optional piecewise-linear helper used by advanced TPA
#if __has_include("common/pwl.h")
#include "common/pwl.h"
#else
typedef struct { float *x; float *y; int n; } pwl_t;
#endif

// Optional CHIRP feature structs
#if __has_include("common/chirp.h")
#include "common/chirp.h"
#else
typedef struct { int _unused; } chirp_t;
typedef struct { int _unused; } phaseComp_t;
#endif

// Time typedefs used across PID
#if __has_include("common/time.h")
#include "common/time.h"
#else
typedef uint32_t timeUs_t;
typedef uint32_t timeDelta_t;
#endif

// Debug mode fallback declarations (used in conditional logic)
#ifndef DEBUG_AC_ERROR
#define DEBUG_AC_ERROR 0
#endif
extern int debugMode;

// Math constants fallback
#ifndef RAD
#define RAD 57.2957795f
#endif

// PG macros fallbacks (register/declare templates)
#if __has_include("pg/pg.h")
#include "pg/pg.h"
#else
#ifndef PG_DECLARE
#define PG_DECLARE(type, name) extern type name;
#endif
#ifndef PG_DECLARE_ARRAY
#define PG_DECLARE_ARRAY(type, count, name) extern type name[];
#endif
#ifndef PG_REGISTER_WITH_RESET_TEMPLATE
#define PG_REGISTER_WITH_RESET_TEMPLATE(type, var, id, version)
#endif
#ifndef PG_REGISTER_ARRAY_WITH_RESET_FN
#define PG_REGISTER_ARRAY_WITH_RESET_FN(type, count, name, id, version)
#endif
#ifndef PG_RESET_TEMPLATE
#define PG_RESET_TEMPLATE(type, var, ...)
#endif
#endif

// RESET_CONFIG fallback used in profile defaults
#if defined(__has_include) && __has_include("config/config_reset.h")
#include "config/config_reset.h"
#else
#ifndef RESET_CONFIG
#define RESET_CONFIG(type, var, ...) do { *(var) = (type){ __VA_ARGS__ }; } while (0)
#endif
#endif

// Flight-mode helpers used by PID gating—fallback to benign values
#ifndef FLIGHT_MODE
#define FLIGHT_MODE(x) 0
#endif
static inline bool isFixedWing(void) { return false; }
static inline bool gyroOverflowDetected(void) { return false; }