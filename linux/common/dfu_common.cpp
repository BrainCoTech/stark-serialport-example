/**
 * @file dfu_common.cpp
 * @brief Implementation of common utility functions for DFU examples
 */

#include "dfu_common.h"
#include "stark_common.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

/****************************************************************************/
// Static variables for callback state
/****************************************************************************/
static int g_dfu_start_time = 0;
static void (*g_cleanup_function)(void) = NULL;

/****************************************************************************/
// Internal callback functions
/****************************************************************************/

static void internal_dfu_state_callback(uint8_t slave_id, uint8_t state) {
    dfu_state_callback_with_timing(slave_id, state, g_dfu_start_time, g_cleanup_function);
}

static void internal_dfu_progress_callback(uint8_t slave_id, float progress) {
    dfu_progress_callback_standard(slave_id, progress);
}

/****************************************************************************/
// Public functions
/****************************************************************************/

void dfu_state_callback_with_timing(uint8_t slave_id, uint8_t state, int start_time_ms, void (*cleanup_func)(void)) {
    printf("DFU State: %hhu\n", state);
    if (state == 4) {
        if (start_time_ms > 0) {
            printf("DFU finished, elapsed time: %d ms\n", get_current_time_ms() - start_time_ms);
        } else {
            printf("DFU finished\n");
        }
        
        if (cleanup_func != NULL) {
            cleanup_func();
        }
        exit(0);
    }
}

void dfu_state_callback_simple(uint8_t slave_id, uint8_t state) {
    dfu_state_callback_with_timing(slave_id, state, 0, NULL);
}

void dfu_progress_callback_standard(uint8_t slave_id, float progress) {
    printf("DFU Progress: %.2f%%\n", progress * 100);
}

void setup_dfu_callbacks_simple(void) {
    set_dfu_state_callback(dfu_state_callback_simple);
    set_dfu_progress_callback(dfu_progress_callback_standard);
}

void setup_dfu_callbacks_with_timing(int start_time_ms, void (*cleanup_func)(void)) {
    g_dfu_start_time = start_time_ms;
    g_cleanup_function = cleanup_func;
    
    set_dfu_state_callback(internal_dfu_state_callback);
    set_dfu_progress_callback(internal_dfu_progress_callback);
}

void wait_for_dfu_completion(int timeout_seconds) {
    if (timeout_seconds <= 0) {
        timeout_seconds = 60; // Default 60 seconds
    }
    
    printf("Waiting for DFU to complete (timeout: %d seconds)...\n", timeout_seconds);
    useconds_t delay = timeout_seconds * 1000 * 1000; // Convert to microseconds
    usleep(delay);
}
