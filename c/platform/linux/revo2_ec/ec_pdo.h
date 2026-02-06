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
 * @param slave_pos Slave position
 * @return Pointer to domain registration array
 */
const ec_pdo_entry_reg_t* get_domain_regs(pdo_config_type_t config_type, unsigned int *off_in_ptr, unsigned int *off_out_ptr, uint16_t slave_pos);

/**
 * @brief Build domain registration entries into caller-provided buffer
 *        (used for multi-slave support; buffer must have size >=3)
 * @param config_type PDO configuration type
 * @param slave_pos slave position
 * @param off_in_ptr pointer to store input offset
 * @param off_out_ptr pointer to store output offset
 * @param out_regs caller-provided array (length >=3)
 */
void build_domain_regs(pdo_config_type_t config_type, uint16_t slave_pos,
                       unsigned int *off_in_ptr, unsigned int *off_out_ptr,
                       ec_pdo_entry_reg_t *out_regs);

/**
 * @brief Get sync manager configuration for specific PDO type
 * @param config_type Type of PDO configuration needed
 * @return Pointer to sync manager configuration array
 */
ec_sync_info_t* get_slave_pdo_syncs(pdo_config_type_t config_type);

/**
 * @brief Read PDO mappings from slave's object dictionary via SDO
 * @param master EtherCAT master pointer (must be initialized)
 * @param slave_rt Slave runtime info (contains slave_pos, off_in, off_out, domain_regs)
 * @return Number of entries written on success, -1 on failure
 * 
 * This function reads PDO mappings from the slave device's object dictionary (0x1600, 0x1601, etc.)
 * via SDO and builds domain registration entries dynamically.
 * 
 * Note: The slave must be in PREOP or higher state for SDO access.
 * Note: This reads the object dictionary configuration, not the currently active sync managers.
 */
int read_pdo_from_object_dictionary(ec_master_t *master, void *slave_rt);

/**
 * @brief Read PDO mappings from slave's configured sync managers
 * @param master EtherCAT master pointer (must be initialized)
 * @param slave_rt Slave runtime info (contains slave_pos, off_in, off_out, domain_regs)
 * @return Number of entries written on success, -1 on failure
 * 
 * This function reads PDO mapping information from the slave's currently configured
 * sync managers using ecrt_master_get_sync_manager(), ecrt_master_get_pdo(),
 * and ecrt_master_get_pdo_entry() APIs. This is similar to Rust's ioctl method.
 * 
 * This method reads the currently active sync manager configuration, which may differ
 * from the object dictionary entries (e.g., if PDOs have been reassigned).
 * 
 * Note: This requires the master to be activated and the slave to be configured.
 */
int read_pdo_from_sync_managers(ec_master_t *master, void *slave_rt);

#ifdef __cplusplus
}
#endif

#endif // EC_PDO_H
