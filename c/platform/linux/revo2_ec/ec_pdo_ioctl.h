/**
 * @file ec_pdo_ioctl.h
 * @brief Reference header for ioctl-based PDO reading (not currently used)
 * 
 * This file contains structure definitions for ioctl-based PDO reading,
 * similar to Rust's implementation. However, the current C++ implementation
 * uses ecrt_master_get_* APIs instead of ioctl for better compatibility.
 * 
 * This file is kept for reference purposes only.
 * 
 * Note: This requires EtherCAT kernel module and /dev/EtherCAT device
 * The ioctl method reads from the slave's configured sync managers,
 * while SDO method reads from the object dictionary.
 */

#ifndef EC_PDO_IOCTL_H
#define EC_PDO_IOCTL_H

#include <stdint.h>
#include <sys/ioctl.h>

// These structures should match the kernel definitions
// Check /usr/include/linux/ethercat.h or similar for exact definitions

typedef struct {
    uint16_t slave_position;
    uint32_t sync_index;
    uint32_t pdo_pos;
    uint16_t index;
    uint8_t entry_count;
    char name[64];
} ec_ioctl_slave_sync_pdo_t;

typedef struct {
    uint16_t slave_position;
    uint32_t sync_index;
    uint32_t pdo_pos;
    uint32_t entry_pos;
    uint16_t index;
    uint8_t subindex;
    uint8_t bit_length;
    char name[64];
} ec_ioctl_slave_sync_pdo_entry_t;

typedef struct {
    uint16_t slave_position;
    uint32_t sync_index;
    uint32_t physical_start_address;
    uint16_t default_size;
    uint16_t control_register;
    uint8_t enable;
    uint8_t pdo_count;
} ec_ioctl_slave_sync_t;

// ioctl commands (these need to match kernel definitions)
#define EC_IOCTL_SLAVE_SYNC         0x9001
#define EC_IOCTL_SLAVE_SYNC_PDO     0x9002
#define EC_IOCTL_SLAVE_SYNC_PDO_ENTRY 0x9003

/**
 * Read sync manager information using ioctl
 */
int ec_ioctl_get_sync(int master_fd, uint16_t slave_pos, uint8_t sync_index, 
                      ec_ioctl_slave_sync_t *sync_info);

/**
 * Read PDO information using ioctl
 */
int ec_ioctl_get_pdo(int master_fd, uint16_t slave_pos, uint8_t sync_index, 
                     uint8_t pdo_pos, ec_ioctl_slave_sync_pdo_t *pdo_info);

/**
 * Read PDO entry information using ioctl
 */
int ec_ioctl_get_pdo_entry(int master_fd, uint16_t slave_pos, uint8_t sync_index,
                           uint8_t pdo_pos, uint8_t entry_pos,
                           ec_ioctl_slave_sync_pdo_entry_t *entry_info);

#endif // EC_PDO_IOCTL_H

