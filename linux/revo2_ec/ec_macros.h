/**
 * @file ec_macros.h
 * @brief Common macro definitions for EtherCAT applications
 * 
 * This header provides commonly used macro definitions across all
 * EtherCAT example applications to avoid duplication.
 */

#ifndef EC_MACROS_H
#define EC_MACROS_H

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

#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) \
  printf("[DEBUG] %s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...) do {} while(0)
#endif

// Error logging macro
#define ERROR_PRINT(fmt, ...) \
  fprintf(stderr, "[ERROR] %s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__)

// Warning logging macro
#define WARN_PRINT(fmt, ...) \
  printf("[WARN] %s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__)

#endif // EC_MACROS_H
