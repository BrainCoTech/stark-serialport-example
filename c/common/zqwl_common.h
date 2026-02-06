/**
 * @file zqwl_common.h
 * @brief Common utility functions for ZQWL USB CAN/CANFD examples
 *
 * This header provides shared utility functions for ZQWL USB CAN/CANFD device
 * initialization using the Rust SDK's built-in ZQWL protocol implementation.
 *
 * Unlike the callback-based approach, this uses the SDK's native ZQWL support
 * which is simpler and has better performance.
 */

#ifndef ZQWL_COMMON_H
#define ZQWL_COMMON_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "stark-sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
// ZQWL Device Functions
/****************************************************************************/

/**
 * @brief Initialize ZQWL CAN device (CAN 2.0 mode)
 *
 * Automatically finds the first ZQWL device and initializes it for CAN 2.0.
 * Used for Revo1 devices.
 *
 * @param arb_baudrate Arbitration domain baudrate (typically 1000000)
 * @return true if successful, false otherwise
 */
static inline bool init_zqwl_can_auto(uint32_t arb_baudrate) {
    // List ZQWL devices
    ZqwlDeviceList* device_list = list_zqwl_devices();
    if (device_list == NULL || device_list->count == 0) {
        printf("No ZQWL device found\n");
        if (device_list) free_zqwl_device_list(device_list);
        return false;
    }

    printf("Found %zu ZQWL device(s):\n", device_list->count);
    for (size_t i = 0; i < device_list->count; i++) {
        printf("  [%zu] %s (VID:0x%04X, PID:0x%04X, CANFD:%s)\n",
               i + 1,
               device_list->devices[i].port_name,
               device_list->devices[i].vid,
               device_list->devices[i].pid,
               device_list->devices[i].supports_canfd ? "yes" : "no");
    }

    // Use the first device
    const char* port_name = device_list->devices[0].port_name;
    printf("Using device: %s\n", port_name);

    // Initialize ZQWL CAN device
    int result = init_zqwl_can(port_name, arb_baudrate);
    free_zqwl_device_list(device_list);

    if (result != 0) {
        printf("Failed to initialize ZQWL CAN device\n");
        return false;
    }

    printf("ZQWL CAN device initialized (baudrate: %u)\n", arb_baudrate);
    return true;
}

/**
 * @brief Initialize ZQWL CANFD device
 *
 * Automatically finds the first ZQWL CANFD device and initializes it.
 * Used for Revo2 devices.
 *
 * @param arb_baudrate Arbitration domain baudrate (typically 1000000)
 * @param data_baudrate Data domain baudrate (typically 5000000)
 * @return true if successful, false otherwise
 */
static inline bool init_zqwl_canfd_auto(uint32_t arb_baudrate, uint32_t data_baudrate) {
    // List ZQWL devices
    ZqwlDeviceList* device_list = list_zqwl_devices();
    if (device_list == NULL || device_list->count == 0) {
        printf("No ZQWL device found\n");
        if (device_list) free_zqwl_device_list(device_list);
        return false;
    }

    // Find a device that supports CANFD
    const char* port_name = NULL;
    for (size_t i = 0; i < device_list->count; i++) {
        printf("  [%zu] %s (VID:0x%04X, PID:0x%04X, CANFD:%s)\n",
               i + 1,
               device_list->devices[i].port_name,
               device_list->devices[i].vid,
               device_list->devices[i].pid,
               device_list->devices[i].supports_canfd ? "yes" : "no");

        if (device_list->devices[i].supports_canfd && port_name == NULL) {
            port_name = device_list->devices[i].port_name;
        }
    }

    if (port_name == NULL) {
        printf("No ZQWL CANFD device found\n");
        free_zqwl_device_list(device_list);
        return false;
    }

    printf("Using CANFD device: %s\n", port_name);

    // Initialize ZQWL CANFD device
    int result = init_zqwl_canfd(port_name, arb_baudrate, data_baudrate);
    free_zqwl_device_list(device_list);

    if (result != 0) {
        printf("Failed to initialize ZQWL CANFD device\n");
        return false;
    }

    printf("ZQWL CANFD device initialized (arb: %u, data: %u)\n",
           arb_baudrate, data_baudrate);
    return true;
}

/**
 * @brief Scan CAN bus for devices
 *
 * @param candidate_ids Array of candidate slave IDs to try
 * @param count Number of candidate IDs
 * @param timeout_ms Timeout in milliseconds
 * @return Found slave ID, or 0 if not found
 */
static inline uint8_t scan_zqwl_can_devices(const uint8_t* candidate_ids,
                                             size_t count,
                                             uint64_t timeout_ms) {
    return scan_can_devices(candidate_ids, count, timeout_ms);
}

/**
 * @brief Cleanup ZQWL resources
 */
static inline void cleanup_zqwl_resources(void) {
    close_zqwl();
    printf("ZQWL device closed\n");
}

#ifdef __cplusplus
}
#endif

#endif // ZQWL_COMMON_H
