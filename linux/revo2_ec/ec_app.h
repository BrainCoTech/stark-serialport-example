/**
 * @file ec_app.h
 * @brief EtherCAT application-level initialization and management
 *
 * This header provides high-level functions to simplify EtherCAT application
 * initialization for both PDO and SDO programs.
 */

#ifndef EC_APP_H
#define EC_APP_H

#include "ec_constants.h"
#include "ec_types.h"
#include <ecrt.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
// Application types
/****************************************************************************/

// Application type enum
typedef enum {
  EC_APP_PDO_ONLY,   // PDO-only application (no SDO operations)
  EC_APP_SDO_ONLY,   // SDO-only application (no PDO operations)
  EC_APP_PDO_AND_SDO // Both PDO and SDO operations
} ec_app_type_t;

// Max supported slaves in multi-slave demo
#define EC_MAX_SLAVES 8

// Per-slave runtime info (offsets, positions, config)
typedef struct {
  uint16_t slave_pos;
  pdo_config_type_t pdo_type;
  unsigned int off_in;
  unsigned int off_out;
  ec_pdo_entry_reg_t domain_regs[3]; // two entries + terminator
  ec_sync_info_t *syncs;
} ec_slave_runtime_t;

// Application context structure
typedef struct {
  ec_master_t *master;
  ec_app_type_t app_type;

  ec_domain_t *domain;
  uint8_t *domain_data;

  ec_slave_config_t *slave_config; // Used for SDO operations
  
  // Multi-slave PDO runtime info
  ec_slave_runtime_t slaves[EC_MAX_SLAVES];
  int slave_count;
} ec_app_context_t;

/****************************************************************************/
// High-level application functions
/****************************************************************************/

/**
 * @brief Get application context for global access
 * @return Pointer to the global application context
 */
ec_app_context_t *ec_app_get_context(void);

/**
 * @brief Check if shutdown has been requested (e.g., by Ctrl+C)
 * @return 1 if shutdown requested, 0 otherwise
 */
int is_shutdown_requested(void);

/****************************************************************************/
// Simplified main function helpers
/****************************************************************************/

/**
 * @brief Simplified main function for SDO applications
 * @param slave_pos Slave position
 * @param demo_func Demo function to run after initialization
 * @return Exit code
 */
int ec_app_main_sdo(uint16_t slave_pos, ec_callback_t demo_func);

/**
 * @brief Simplified main function for PDO applications
 * @param pdo_type PDO configuration type
 * @param slave_pos Slave position
 * @param cyclic_func Cyclic function to run
 * @return Exit code
 *
 * Note: Real-time priority is fixed at 49
 */
int ec_app_main_pdo(pdo_config_type_t pdo_type, uint16_t slave_pos, ec_callback_t cyclic_func);

/**
 * @brief Initialize multi-slave PDO application (no built-in demo)
 * @param ctx Application context
 * @param slave_cfgs Array of slave configs (slave_pos + pdo_type + syncs)
 * @param slave_count Number of slaves (<= EC_MAX_SLAVES)
 * @param realtime_priority Real-time priority
 * @return 0 on success, -1 on failure
 */
int ec_app_init_pdo_multi(ec_app_context_t *ctx,
                          const ec_slave_runtime_t *slave_cfgs,
                          int slave_count,
                          int realtime_priority);

/**
 * @brief Generic cyclic task implementation with optional feedback callback
 * @param read_feedback_func Optional callback for reading additional feedback
 * data (can be NULL)
 *
 * This function provides a common cyclic task implementation that handles:
 * - EtherCAT communication cycle
 * - Demo control modes (position, speed, current, PWM, gestures)
 * - Joint feedback reading
 * - Optional additional feedback reading (touch, pressure, etc.)
 */
void ec_app_run_cyclic_task(ec_callback_t read_feedback_func);

/**
 * @brief Cleanup and release EtherCAT master/context
 */
void ec_app_cleanup(ec_app_context_t *ctx);

#ifdef __cplusplus
}
#endif

#endif // EC_APP_H
