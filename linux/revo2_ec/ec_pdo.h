/**
 * @file ec_pdo.h
 * @brief EtherCAT PDO configuration and management functions
 * 
 * This header provides functions for PDO (Process Data Object) configuration,
 * domain registration, and PDO mapping management.
 */

#ifndef EC_PDO_H
#define EC_PDO_H

#include <ecrt.h>
#include "ec_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
// PDO Configuration Functions
/****************************************************************************/

/**
 * @brief Get domain registration for PDO entries based on configuration type
 * @param config_type PDO configuration type
 * @param off_in_ptr Pointer to store input offset
 * @param off_out_ptr Pointer to store output offset
 * @return Pointer to domain registration array
 */
const ec_pdo_entry_reg_t* get_domain_regs(pdo_config_type_t config_type, unsigned int *off_in_ptr, unsigned int *off_out_ptr);

/**
 * @brief Get sync manager configuration for specific PDO type
 * @param config_type Type of PDO configuration needed
 * @return Pointer to sync manager configuration array
 */
ec_sync_info_t* get_slave_pdo_syncs(pdo_config_type_t config_type);

#ifdef __cplusplus
}
#endif

#endif // EC_PDO_H
