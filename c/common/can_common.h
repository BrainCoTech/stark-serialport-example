/**
 * @file can_common.h
 * @brief Common utility functions for CAN/CANFD examples
 *
 * This header provides shared utility functions for CAN/CANFD device
 * initialization, channel configuration, and callback setup.
 *
 * ============================================================================
 * CAN Backend Options (Runtime Selection)
 * ============================================================================
 *
 * All backends are compiled by default on Linux. Select at runtime via
 * environment variable or CLI options.
 *
 * 1. SocketCAN (Linux default, no 3rd party deps):
 *    - CLI options: -s (CAN 2.0), -S (CANFD)
 *    - Environment: STARK_CAN_BACKEND=socketcan
 *    - Interface: STARK_SOCKETCAN_IFACE=can0 (default)
 *
 * 2. ZLG USB-CANFD adapter (dynamic loading):
 *    - CLI options: -z (CAN 2.0), -Z (CANFD)
 *    - Environment: STARK_CAN_BACKEND=zlg
 *    - Requires: libusbcanfd.so/.dll at runtime
 *    - Platforms: Linux, Windows
 *
 * 3. ZQWL (SDK built-in, cross-platform):
 *    - CLI options: -c (CAN 2.0), -f (CANFD)
 *    - No external dependencies
 *
 * 4. Disable CAN support:
 *    - Compile: make STARK_NO_CAN=1
 *
 * ============================================================================
 * Build & Run Examples
 * ============================================================================
 *
 *   # Build (all backends compiled by default on Linux)
 *   make
 *   make STARK_NO_CAN=1       # Disable CAN support
 *
 *   # Run with different backends
 *   ./hand_demo.exe                              # Default: SocketCAN
 *   STARK_CAN_BACKEND=zlg ./hand_demo.exe        # Use ZLG
 *   ./hand_demo.exe -s can0 1                    # SocketCAN explicit
 *   ./hand_demo.exe -z 1                         # ZLG explicit
 *
 * ============================================================================
 */

#ifndef CAN_COMMON_H
#define CAN_COMMON_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
// Common macro definitions for ZLG CANFD
/****************************************************************************/

#define ZCANFD_TYPE_USBCANFD 41 // ZLG USB-CANFD device type
#define ZCANFD_CARD_INDEX 0     // Card index
#define ZCANFD_CHANNEL_INDEX 0  // Channel index
#define RX_WAIT_TIME 100        // Receive wait time (ms)
#define RX_BUFF_SIZE 1000       // Receive buffer size

/****************************************************************************/
// Function declarations
/****************************************************************************/

/**
 * @brief Initialize CAN device
 * Opens the CAN device (SocketCAN or ZLG depending on backend)
 * @return true if successful, false otherwise
 */
bool init_can_device(void);

/**
 * @brief Initialize CANFD device
 * Opens the CANFD device (SocketCAN or ZLG depending on backend)
 * @return true if successful, false otherwise
 */
bool init_canfd_device(void);

/**
 * @brief Start CAN channel
 * Configures and starts CAN channel with standard baud rates:
 * - Arbitration domain: 1 Mbps
 * - Data domain: 5 Mbps (for CANFD)
 * @return true if successful, false otherwise
 */
bool start_can_channel(void);

/**
 * @brief Start CANFD channel
 * Configures and starts CANFD channel (same configuration as start_can_channel)
 * @return true if successful, false otherwise
 */
bool start_canfd_channel(void);

/**
 * @brief Setup CAN callbacks
 * Registers CAN transmit and receive callbacks for standard CAN frames
 */
void setup_can_callbacks(void);

/**
 * @brief Setup CANFD callbacks
 * Registers CANFD transmit and receive callbacks for CANFD frames
 */
void setup_canfd_callbacks(void);

/**
 * @brief Setup CAN (all-in-one initialization)
 * Performs complete CAN setup: device init, channel start, and callback setup
 * @return true if successful, false otherwise
 */
bool setup_can(void);

/**
 * @brief Setup CANFD (all-in-one initialization)
 * Performs complete CANFD setup: device init, channel start, and callback setup
 * @return true if successful, false otherwise
 */
bool setup_canfd(void);

/**
 * @brief Cleanup CAN/CANFD resources
 * Resets channel and closes device
 */
void cleanup_can_resources(void);

/**
 * @brief Select ZLG backend explicitly
 * Call before setup_can/setup_canfd to use ZLG adapter
 */
void set_can_backend_zlg(void);

/**
 * @brief Select SocketCAN backend explicitly
 * Call before setup_can/setup_canfd to use SocketCAN
 */
void set_can_backend_socketcan(void);

#ifdef __cplusplus
}
#endif

#endif // CAN_COMMON_H
