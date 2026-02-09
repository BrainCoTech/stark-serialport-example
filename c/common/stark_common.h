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
#include "platform_compat.h"
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
 * @brief Get device info and return hardware type
 * Gets device information, prints serial number, firmware version, and hardware
 * type, then returns the hardware type for caller to decide which API to use.
 * 
 * API selection guide:
 * - Motor API: Revo1 Basic/Touch use Revo1 API; Revo1 Advanced/AdvancedTouch and all Revo2 use Revo2 API
 * - Touch API: Revo1 Touch/AdvancedTouch use Revo1 Touch API; Revo2 Touch/TouchPressure use Revo2 Touch API
 * 
 * @param handle Device handler
 * @param slave_id Slave device ID
 * @return Hardware type (StarkHardwareType), or 0 (UNKNOWN) on failure
 */
StarkHardwareType get_device_hardware_type(DeviceHandler *handle, uint8_t slave_id);

/**
 * @brief Check if hardware type uses Revo1 motor API
 * Revo1 Basic/Touch use Revo1 motor API (position only, 0-100 range)
 * @param hw_type Hardware type
 * @return true if uses Revo1 motor API
 */
bool hw_uses_revo1_motor_api(StarkHardwareType hw_type);

/**
 * @brief Check if hardware type uses Revo2 motor API
 * Revo1 Advanced and all Revo2 devices use Revo2 motor API (position+time, position+speed)
 * @param hw_type Hardware type
 * @return true if uses Revo2 motor API
 */
bool hw_uses_revo2_motor_api(StarkHardwareType hw_type);

/**
 * @brief Check if hardware type uses Revo1 touch API
 * Revo1 Touch and Revo1 Advanced Touch use Revo1 touch API (different sensor counts per finger)
 * @param hw_type Hardware type
 * @return true if uses Revo1 touch API
 */
bool hw_uses_revo1_touch_api(StarkHardwareType hw_type);

/**
 * @brief Check if hardware type uses Revo2 touch API
 * Revo2 Touch and Revo2 Touch Pressure use Revo2 touch API (uniform sensor structure)
 * @param hw_type Hardware type
 * @return true if uses Revo2 touch API
 */
bool hw_uses_revo2_touch_api(StarkHardwareType hw_type);

/**
 * @brief Check if hardware type has touch sensor
 * @param hw_type Hardware type
 * @return true if device has touch sensor
 */
bool hw_has_touch_sensor(StarkHardwareType hw_type);

// Legacy functions (deprecated, use get_device_hardware_type instead)
bool verify_device_is_revo1(DeviceHandler *handle, uint8_t slave_id);
bool verify_device_is_revo2(DeviceHandler *handle, uint8_t slave_id);

/**
 * @brief Get current time in milliseconds
 * Returns the current system time in milliseconds since epoch
 * @return Current time in milliseconds
 */
int get_current_time_ms(void);

/****************************************************************************/
// Auto-detect and initialize device
/****************************************************************************/

/**
 * @brief Touch sensor type enumeration
 * Derived from StarkHardwareType:
 * - REVO1_TOUCH, REVO1_ADVANCED_TOUCH, REVO2_TOUCH -> Capacitive
 * - REVO2_TOUCH_PRESSURE -> Pressure (Modulus)
 * - Others -> None
 */
typedef enum {
  TOUCH_TYPE_NONE = 0,       // No touch sensor
  TOUCH_TYPE_CAPACITIVE = 1, // Capacitive touch (Revo1 Touch, Revo2 Touch)
  TOUCH_TYPE_PRESSURE = 2    // Pressure touch / Modulus (Revo2 only)
} TouchSensorType;

/**
 * @brief Device context for data collection
 * Contains all information needed to communicate with a device.
 * 
 * This is a lightweight wrapper around DeviceHandler that caches frequently-accessed
 * device information to avoid repeated queries.
 * 
 * Note: protocol, port_name, and baudrate can be queried via:
 *   - stark_get_protocol_type(handle)
 *   - stark_get_port_name(handle)
 *   - stark_get_baudrate(handle)
 */
typedef struct {
  DeviceHandler* handle;           // Device handler (opaque)
  uint8_t slave_id;                // Current slave ID
  
  // Cached device information (to avoid repeated queries)
  StarkHardwareType hw_type;       // Detected or overridden hardware type
  StarkHardwareType hw_type_override; // Manual hardware type override (0 = auto-detect)
  char serial_number[20];          // Device serial number (e.g., "BCMER247J250888EF")
  
  // Application-level recommendations
  uint32_t motor_freq;             // Recommended motor polling frequency (Hz)
  uint32_t touch_freq;             // Recommended touch polling frequency (Hz)
} DeviceContext;

/**
 * @brief Auto-detect devices and let user select one
 * Scans all protocols (Modbus, CAN, CANFD) and displays available devices.
 * If multiple devices found, prompts user to select one.
 * 
 * @param ctx Output: Device context to fill
 * @param require_touch If true, only show devices with touch sensors
 * @return true if device selected and initialized, false otherwise
 */
bool auto_detect_and_init(DeviceContext* ctx, bool require_touch);

/**
 * @brief Cleanup device context
 * Closes device connection and frees resources
 * 
 * @param ctx Device context to cleanup
 */
void cleanup_device_context(DeviceContext* ctx);

/****************************************************************************/
// Manual initialization functions
/****************************************************************************/

/**
 * @brief Initialize device via Modbus
 * @param ctx Output: Device context to fill
 * @param port Serial port path
 * @param baudrate Baudrate
 * @param slave_id Slave ID
 * @return true if successful
 */
bool init_modbus(DeviceContext* ctx, const char* port, uint32_t baudrate, uint8_t slave_id);

/**
 * @brief Initialize device via Protobuf protocol
 * Protobuf uses fixed baudrate 115200 and slave_id range 10-254.
 * Position range is 0-100 internally (SDK converts from 0-1000).
 * 
 * @param ctx Output: Device context to fill
 * @param port Serial port path
 * @param slave_id Slave ID (default 10, range 10-254)
 * @return true if successful
 */
bool init_protobuf(DeviceContext* ctx, const char* port, uint8_t slave_id);

/**
 * @brief Initialize device via ZQWL CAN/CANFD
 * @param ctx Output: Device context to fill
 * @param port Serial port path
 * @param arb_baudrate Arbitration baudrate (CAN 2.0 uses this as main baudrate)
 * @param data_baudrate Data baudrate (CANFD only, ignored for CAN 2.0)
 * @param slave_id Slave ID
 * @param is_canfd true for CANFD, false for CAN 2.0
 * @return true if successful
 */
bool init_zqwl_device(DeviceContext* ctx, const char* port, uint32_t arb_baudrate, 
                      uint32_t data_baudrate, uint8_t slave_id, bool is_canfd);

#ifdef __linux__
/**
 * @brief Initialize device via SocketCAN (using example's can_common.cpp implementation)
 * 
 * This uses the example's own SocketCAN implementation in can_common.cpp.
 * All backends compiled by default on Linux, select at runtime.
 * 
 * @param ctx Output: Device context to fill
 * @param iface CAN interface name (e.g., "can0")
 * @param slave_id Slave ID
 * @param is_canfd true for CANFD, false for CAN 2.0
 * @return true if successful
 */
bool init_socketcan_device(DeviceContext* ctx, const char* iface, uint8_t slave_id, bool is_canfd);

/**
 * @brief Initialize device via SocketCAN (using SDK built-in implementation)
 * 
 * This uses the SDK's built-in SocketCAN support (init_socketcan_can/init_socketcan_canfd).
 * No additional compile flags needed, works out of the box on Linux.
 * 
 * @param ctx Output: Device context to fill
 * @param iface CAN interface name (e.g., "can0"). If NULL, uses "can0"
 * @param slave_id Slave ID
 * @param is_canfd true for CANFD, false for CAN 2.0
 * @return true if successful
 */
bool init_socketcan_device_builtin(DeviceContext* ctx, const char* iface, uint8_t slave_id, bool is_canfd);

/**
 * @brief Initialize device via ZLG USB-CANFD
 * @param ctx Output: Device context to fill
 * @param slave_id Slave ID
 * @param is_canfd true for CANFD, false for CAN 2.0
 * @return true if successful
 */
bool init_zlg_device(DeviceContext* ctx, uint8_t slave_id, bool is_canfd);
#endif

/**
 * @brief Parse command line arguments and initialize device
 * Supports: auto-detect, -m (Modbus), -c (CAN), -f (CANFD), 
 *           -s/-S (SocketCAN via can_common.cpp), -b/-B (SocketCAN via SDK built-in),
 *           -z/-Z (ZLG)
 * @param ctx Output: Device context to fill
 * @param argc Argument count
 * @param argv Argument values
 * @param arg_idx Output: Index of next argument after init options
 * @return true if successful, false if error or help requested
 */
bool parse_args_and_init(DeviceContext* ctx, int argc, const char* argv[], int* arg_idx);

/**
 * @brief Print common initialization usage
 * @param prog_name Program name
 */
void print_init_usage(const char* prog_name);

/**
 * @brief Get protocol name string
 * @param protocol Protocol type
 * @return Protocol name string
 */
const char* get_protocol_name_str(uint8_t protocol);

/**
 * @brief Get hardware type name string
 * @param hw_type Hardware type
 * @return Hardware type name string
 */
const char* get_hardware_type_name_str(uint8_t hw_type);

/**
 * @brief Get touch sensor type name string
 * @param touch_type Touch sensor type
 * @return Touch type name string
 */
const char* get_touch_type_name_str(TouchSensorType touch_type);

/**
 * @brief Get touch sensor type from hardware type
 * Uses SDK function to determine touch sensor type.
 * @param hw_type Hardware type
 * @return Touch sensor type
 */
TouchSensorType get_touch_sensor_type(StarkHardwareType hw_type);

#ifdef __cplusplus
}
#endif

#endif // STARK_COMMON_H
