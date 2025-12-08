/**
 * @file stark_common.h
 * @brief Common utility functions for Stark SDK C examples
 *
 * This header provides shared utility functions used across multiple
 * example programs, including signal handling, hex printing, and device info.
 */

#ifndef STARK_COMMON_H
#define STARK_COMMON_H

#include "stark-sdk.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
// Function declarations
/****************************************************************************/

/**
 * @brief Setup signal handlers for crash debugging
 * Registers handlers for SIGSEGV and SIGABRT
 */
void setup_signal_handlers(void);

/**
 * @brief Print data in hexadecimal format
 * @param data Data buffer to print
 * @param len Length of data buffer
 */
void print_hex(const unsigned char *data, int len);

/**
 * @brief Get and print device information
 * Retrieves and displays device serial number, firmware version, and hardware
 * type
 * @param handle Device handler
 * @param slave_id Slave device ID
 * @return true if successful, false otherwise
 */
bool get_and_print_device_info(DeviceHandler *handle, uint8_t slave_id);

/**
 * @brief Get and print extended device information
 * Retrieves and displays baudrate, voltage, LED info, and button events
 * @param handle Device handler
 * @param slave_id Slave device ID
 */
void get_and_print_extended_info(DeviceHandler *handle, uint8_t slave_id);

/**
 * @brief Get device info, print it, and verify if it's Revo1
 * Gets device information, prints serial number, firmware version, and hardware
 * type, then verifies if the device is Revo1 (basic, advanced, or touch
 * version)
 * @param handle Device handler
 * @param slave_id Slave device ID
 * @return true if successful and device is Revo1, false otherwise
 */
bool verify_device_is_revo1(DeviceHandler *handle, uint8_t slave_id);

/**
 * @brief Get device info, print it, and verify if it's Revo2
 * Gets device information, prints serial number, firmware version, and hardware
 * type, then verifies if the device is Revo2 (basic or touch version)
 * @param handle Device handler
 * @param slave_id Slave device ID
 * @return true if successful and device is Revo2, false otherwise
 */
bool verify_device_is_revo2(DeviceHandler *handle, uint8_t slave_id);

/**
 * @brief Get current time in milliseconds
 * Returns the current system time in milliseconds since epoch
 * @return Current time in milliseconds
 */
int get_current_time_ms(void);

#ifdef __cplusplus
}
#endif

#endif // STARK_COMMON_H
