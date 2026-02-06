/**
 * @file ec_macros.h
 * @brief Common macro definitions for EtherCAT applications
 * 
 * This header provides commonly used macro definitions across all
 * EtherCAT example applications to avoid duplication.
 */

#ifndef EC_MACROS_H
#define EC_MACROS_H

#include <stdio.h>
#include <time.h>

/****************************************************************************/
// Timing and frequency macros
/****************************************************************************/

#define FREQUENCY 1000                    // 1000 Hz
#define MAX_WAIT_LOOP (FREQUENCY * 0.1)  // Maximum wait loop iterations
#define CLOCK_TO_USE CLOCK_MONOTONIC      // Clock type for timing

#define NSEC_PER_SEC (1000000000L)        // Nanoseconds per second
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)  // Period in nanoseconds

// Time conversion and difference macros
#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
#define DIFF_NS(A, B) \
  (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + (B).tv_nsec - (A).tv_nsec)

/****************************************************************************/
// Application configuration macros
/****************************************************************************/

#define MAX_TRAJECTORY_POINTS 10          // Maximum trajectory points

/****************************************************************************/
// EtherCAT communication macros
/****************************************************************************/

// PDO entry registration helper
#define PDO_ENTRY_REG(pos, alias, vid, pid, idx, subidx, offset, bit_pos) \
  {pos, alias, vid, pid, idx, subidx, offset, bit_pos}

// Domain registration terminator
#define PDO_ENTRY_REG_END {}

/****************************************************************************/
// Utility macros
/****************************************************************************/

// Array size calculation
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

// Min/Max macros (if not already defined)
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

// Bit manipulation macros
#define SET_BIT(reg, bit)    ((reg) |= (1U << (bit)))
#define CLEAR_BIT(reg, bit)  ((reg) &= ~(1U << (bit)))
#define TOGGLE_BIT(reg, bit) ((reg) ^= (1U << (bit)))
#define CHECK_BIT(reg, bit)  (((reg) >> (bit)) & 1U)

/****************************************************************************/
// Turbo mode parameter utilities
/****************************************************************************/

#define MAKE_TURBO_PARAM(interval, duration) \
  ((uint32_t)(((uint16_t)(interval) << 16) | ((uint16_t)(duration) & 0xFFFF)))

#define GET_TURBO_INTERVAL(param) ((uint16_t)((param) >> 16))
#define GET_TURBO_DURATION(param) ((uint16_t)((param) & 0xFFFF))

/****************************************************************************/
// Debug and logging macros
/****************************************************************************/

// Common time prefix helper for logs
static inline void EC_LOG_TIME_PREFIX(FILE *stream) {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  struct tm tm_now;
  localtime_r(&ts.tv_sec, &tm_now);
  int ms = ts.tv_nsec / 1000000;
  fprintf(stream, "[%02d:%02d:%02d.%03d] ",
          tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec, ms);
}

#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...)                          \
  do {                                                 \
    EC_LOG_TIME_PREFIX(stdout);                        \
    printf("[DEBUG] %s:%d: " fmt "\n",                 \
           __FILE__, __LINE__, ##__VA_ARGS__);         \
  } while (0)
#else
#define DEBUG_PRINT(fmt, ...) do {} while(0)
#endif

// Demo debug output (default: disabled)
// Enable with: -DDEBUG_DEMO or #define DEBUG_DEMO
#ifdef DEBUG_DEMO
#define DEMO_DEBUG(fmt, ...)                           \
  do {                                                 \
    EC_LOG_TIME_PREFIX(stdout);                        \
    printf("[Demo] " fmt "\n", ##__VA_ARGS__);         \
  } while (0)
#else
#define DEMO_DEBUG(fmt, ...) do {} while(0)
#endif

// Loop debug output (default: disabled)
// Enable with: -DDEBUG_LOOP or #define DEBUG_LOOP
#ifdef DEBUG_LOOP
#define LOOP_DEBUG(fmt, ...)                           \
  do {                                                 \
    EC_LOG_TIME_PREFIX(stdout);                        \
    printf("[Debug] " fmt "\n", ##__VA_ARGS__);        \
  } while (0)
#else
#define LOOP_DEBUG(fmt, ...) do {} while(0)
#endif

// Data send/receive debug output (default: disabled)
// Enable with: -DDEBUG_DATA or #define DEBUG_DATA
#ifdef DEBUG_DATA
#define DATA_DEBUG(fmt, ...)                           \
  do {                                                 \
    EC_LOG_TIME_PREFIX(stdout);                        \
    printf("[Debug] " fmt "\n", ##__VA_ARGS__);        \
  } while (0)
#else
#define DATA_DEBUG(fmt, ...) do {} while(0)
#endif

// Feedback data debug output (default: enabled)
// Disable with: -DNO_DEBUG_FEEDBACK or #define NO_DEBUG_FEEDBACK
// Print frequency can be controlled via FEEDBACK_PRINT_INTERVAL (default: 100 cycles = 100ms at 1kHz)
#ifndef FEEDBACK_PRINT_INTERVAL
#define FEEDBACK_PRINT_INTERVAL 2000  // Print every 2000 cycles (2000ms at 1kHz)
#endif
#ifndef NO_DEBUG_FEEDBACK
#define DEBUG_FEEDBACK
#endif
#ifdef DEBUG_FEEDBACK
#define FEEDBACK_DEBUG(fmt, ...)                       \
  do {                                                 \
    EC_LOG_TIME_PREFIX(stdout);                        \
    printf("[Feedback] " fmt "\n", ##__VA_ARGS__);     \
  } while (0)
#else
#define FEEDBACK_DEBUG(fmt, ...) do {} while(0)
#endif

// Error logging macro
#define ERROR_PRINT(fmt, ...)                          \
  do {                                                 \
    EC_LOG_TIME_PREFIX(stderr);                        \
    fprintf(stderr, "[ERROR] %s:%d: " fmt "\n",        \
            __FILE__, __LINE__, ##__VA_ARGS__);        \
  } while (0)

// Warning logging macro
#define WARN_PRINT(fmt, ...)                           \
  do {                                                 \
    EC_LOG_TIME_PREFIX(stdout);                        \
    printf("[WARN] %s:%d: " fmt "\n",                  \
           __FILE__, __LINE__, ##__VA_ARGS__);         \
  } while (0)

#endif // EC_MACROS_H
