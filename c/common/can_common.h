/**
 * @file can_common.h
 * @brief Common utility functions for CAN/CANFD examples
 * 
 * This header provides shared utility functions for CAN/CANFD device
 * initialization, channel configuration, and callback setup.
 *
 * Supported CAN backends:
 * 1. ZQWL (default): SDK built-in support, no configuration needed
 * 2. ZLG: Zhou Ligong USB-CANFD adapter
 *    - Compile: make CAN_BACKEND=zlg
 *    - Device type: 41 (USB-CANFD)
 *    - Default config: Arbitration 1 Mbps, Data 5 Mbps
 * 3. SocketCAN (Linux only):
 *    - Compile: make CAN_BACKEND=socketcan
 *    - Environment: STARK_CAN_BACKEND=socketcan
 *    - Interface: STARK_SOCKETCAN_IFACE=can0 (default)
 *
 * Backend selection priority:
 * - Environment variable STARK_CAN_BACKEND (socketcan/zlg)
 * - Compile-time flag (STARK_USE_ZLG / STARK_USE_SOCKETCAN)
 * - Default: ZLG if compiled with STARK_USE_ZLG=1
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
 * Opens the ZLG USB-CANFD device
 * @return true if successful, false otherwise
 */
bool init_can_device(void);

/**
 * @brief Initialize CANFD device
 * Opens the ZLG USB-CANFD device (same as init_can_device)
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

#ifdef __cplusplus
}
#endif

#endif // CAN_COMMON_H
