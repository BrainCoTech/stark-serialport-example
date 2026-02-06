/**
 * @file dfu_common.h
 * @brief Common utility functions for DFU (Device Firmware Update) examples
 * 
 * This header provides shared utility functions used across multiple
 * DFU example programs, including callback setup and common DFU operations.
 */

#ifndef DFU_COMMON_H
#define DFU_COMMON_H

#include "stark-sdk.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
// Function declarations
/****************************************************************************/

/**
 * @brief Standard DFU state callback with timing
 * Prints DFU state and handles completion with elapsed time calculation
 * @param slave_id Slave device ID
 * @param state DFU state
 * @param start_time_ms Start time in milliseconds (for elapsed time calculation)
 * @param cleanup_func Optional cleanup function to call on completion (can be NULL)
 */
void on_dfu_state_with_timing(uint8_t slave_id, uint8_t state, int start_time_ms, void (*cleanup_func)(void));

/**
 * @brief Standard DFU state callback
 * Prints DFU state and exits on completion
 * @param slave_id Slave device ID
 * @param state DFU state
 */
void on_dfu_state(uint8_t slave_id, uint8_t state);

/**
 * @brief Standard DFU progress callback
 * Prints DFU progress as percentage
 * @param slave_id Slave device ID
 * @param progress Progress value (0.0 to 1.0)
 */
void on_dfu_progress(uint8_t slave_id, float progress);

/**
 * @brief Setup standard DFU callbacks
 * Sets up basic DFU state and progress callbacks
 */
void setup_dfu_callbacks(void);

/**
 * @brief Setup DFU callbacks with timing and cleanup
 * Sets up DFU callbacks with elapsed time calculation and optional cleanup
 * @param start_time_ms Start time in milliseconds
 * @param cleanup_func Optional cleanup function to call on completion (can be NULL)
 */
void setup_dfu_callbacks_with_timing(int start_time_ms, void (*cleanup_func)(void));

/**
 * @brief Wait for DFU completion with timeout
 * @param timeout_seconds Timeout in seconds (default: 60)
 */
void wait_for_dfu_completion(int timeout_seconds);

#ifdef __cplusplus
}
#endif

#endif // DFU_COMMON_H
