/**
 * @file ec_pdo.cpp
 * @brief Implementation of EtherCAT PDO configuration and management functions
 */

#include "ec_pdo.h"
#include "ec_constants.h"
#include "ec_sdo.h"
#include "ec_app.h"  // Include here for ec_slave_runtime_t definition
#include "ec_types.h"  // For pdo_config_type_t
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>  // for usleep

/****************************************************************************/
// PDO Entry Definitions
/****************************************************************************/

// Common joint control PDO entries (shared by all configurations)
static const ec_pdo_entry_info_t common_joint_pdo_entries[] = {
    // RxPDO: 0x1600 (master -> slave) multi-finger control mode
    {0x7000, 0x01, 16}, // mult_joint_ctrl_mode
    {0x7000, 0x02, 96}, // joint_param1 (positions)  (6 x uint16)
    {0x7000, 0x03, 96}, // joint_param2 (durations/speeds) (6 x uint16)

    // RxPDO: 0x1601 (master -> slave) single-finger control mode
    {0x7010, 0x01, 8},  // single_joint_ctrl_mode
    {0x7010, 0x02, 8},  // single_joint_id
    {0x7010, 0x03, 16}, // single_joint_param1
    {0x7010, 0x04, 16}, // single_joint_param2

    // TxPDO: 0x1A00 (slave -> master)
    {0x6000, 0x01, 96}, // joint_pos (6 x uint16)
    {0x6000, 0x02, 96}, // joint_spd (6 x uint16)
    {0x6000, 0x03, 96}, // joint_cur (6 x int16)
    {0x6000, 0x04, 96}, // joint_status (6 x uint16)
};

// Touch-specific PDO entries (additional touch data)
static const ec_pdo_entry_info_t touch_specific_pdo_entries[] = {
    // TxPDO: 0x1A01 (slave -> master) touch data
    {0x6010, 0x01, 80},  // force_normal (5 x uint16)
    {0x6010, 0x02, 80},  // force_tangential (5 x uint16)
    {0x6010, 0x03, 80},  // force_direction (5 x uint16)
    {0x6010, 0x04, 160}, // proximity (5 x uint32)
    {0x6010, 0x05, 80},  // touch_status (5 x uint16)
};

// Pressure touch-specific PDO entries (additional pressure touch data)
static const ec_pdo_entry_info_t pressure_touch_specific_pdo_entries[] = {
    // TxPDO: 0x1A01 (slave -> master) pressure-sensing touch data
    {0x6010, 0x01, 144}, // force_normal (9 x uint16)
    {0x6010, 0x02, 144}, // force_index (9 x uint16)
    {0x6010, 0x03, 144}, // force_mid (9 x uint16)
    {0x6010, 0x04, 144}, // force_ring (9 x uint16)
    {0x6010, 0x05, 144}, // force_pinky (9 x uint16)
    {0x6010, 0x06,
     240}, // force_palm (46 x uint16), TODO: firmware currently uses 47*16
    {0x0000, 0x00, 240}, // Gap
    {0x0000, 0x00, 240}, // Gap
    {0x0000, 0x00, 32},  // Gap
    {0x6010, 0x07, 96},  // force_total (6 x uint16)
};

/****************************************************************************/
// PDO Mapping Definitions
/****************************************************************************/

// Basic PDO mapping (only joint control, no touch)
static ec_pdo_info_t basic_slave_pdos[] = {
    {0x1600, 3,
     (ec_pdo_entry_info_t
          *)&common_joint_pdo_entries[0]}, // RxPDO: 0x7000:01-03 multi-finger
    {0x1601, 4,
     (ec_pdo_entry_info_t
          *)&common_joint_pdo_entries[3]}, // RxPDO: 0x7010:01-04 single-finger
    {0x1A00, 4,
     (ec_pdo_entry_info_t
          *)&common_joint_pdo_entries[7]}, // TxPDO: 0x6000:01-04 joint feedback
};

// Touch PDO mapping (joint control + touch)
static ec_pdo_info_t touch_slave_pdos[] = {
    {0x1600, 3,
     (ec_pdo_entry_info_t
          *)&common_joint_pdo_entries[0]}, // RxPDO: 0x7000:01-03 multi-finger
    {0x1601, 4,
     (ec_pdo_entry_info_t
          *)&common_joint_pdo_entries[3]}, // RxPDO: 0x7010:01-04 single-finger
    {0x1A00, 4,
     (ec_pdo_entry_info_t
          *)&common_joint_pdo_entries[7]}, // TxPDO: 0x6000:01-04 joint feedback
    {0x1A01, 5,
     (ec_pdo_entry_info_t *)&touch_specific_pdo_entries
         [0]}, // TxPDO: 0x6010:01-05 touch feedback
};

// Touch pressure PDO mapping (joint control + pressure touch)
static ec_pdo_info_t touch_pressure_slave_pdos[] = {
    {0x1600, 3,
     (ec_pdo_entry_info_t
          *)&common_joint_pdo_entries[0]}, // RxPDO: 0x7000:01-03 multi-finger
    {0x1601, 4,
     (ec_pdo_entry_info_t
          *)&common_joint_pdo_entries[3]}, // RxPDO: 0x7010:01-04 single-finger
    {0x1A00, 4,
     (ec_pdo_entry_info_t
          *)&common_joint_pdo_entries[7]}, // TxPDO: 0x6000:01-04 joint feedback
    {0x1A01, 10,
     (ec_pdo_entry_info_t *)&pressure_touch_specific_pdo_entries
         [0]}, // TxPDO: 0x6010:01-07 pressure touch
};

/****************************************************************************/
// Sync Manager Configuration Definitions
/****************************************************************************/

// Basic sync manager configuration (1 TxPDO)
static ec_sync_info_t basic_slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, basic_slave_pdos, EC_WD_ENABLE},     // SM2: RxPDO
    {3, EC_DIR_INPUT, 1, &basic_slave_pdos[2], EC_WD_DISABLE}, // SM3: TxPDO
    {0xff}};

// Touch sync manager configuration (2 TxPDOs)
static ec_sync_info_t touch_slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, touch_slave_pdos,
     EC_WD_ENABLE}, // SM2: RxPDO (2 PDOs for touch version)
    {3, EC_DIR_INPUT, 2, &touch_slave_pdos[2],
     EC_WD_DISABLE}, // SM3: TxPDO (2 PDOs for touch version)
    {0xff}};

// Touch pressure sync manager configuration (2 TxPDOs)
static ec_sync_info_t touch_pressure_slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, touch_pressure_slave_pdos,
     EC_WD_ENABLE}, // SM2: RxPDO (2 PDOs for pressure touch version)
    {3, EC_DIR_INPUT, 2, &touch_pressure_slave_pdos[2],
     EC_WD_DISABLE}, // SM3: TxPDO (2 PDOs for pressure touch version)
    {0xff}};

/****************************************************************************/
// Domain Registration
/****************************************************************************/

// Domain registrations for different configurations (templates)
// Note: offset pointers are placeholders; build_domain_regs will override
static ec_pdo_entry_reg_t basic_domain_regs[] = {
    {0, 0, STARK_VENDOR_ID, STARK_PRODUCT_CODE, 0x7000, 0x01, NULL, NULL},
    {0, 0, STARK_VENDOR_ID, STARK_PRODUCT_CODE, 0x6000, 0x01, NULL, NULL},
    {}};

// Touch domain registration (joint data + basic touch data)
static ec_pdo_entry_reg_t touch_domain_regs[] = {
    {0, 0, STARK_VENDOR_ID, STARK_PRODUCT_CODE, 0x7000, 0x01, NULL, NULL},
    {0, 0, STARK_VENDOR_ID, STARK_PRODUCT_CODE, 0x6000, 0x01, NULL, NULL},
    {}};

// Pressure touch domain registration (joint data + pressure touch data)
static ec_pdo_entry_reg_t pressure_touch_domain_regs[] = {
    {0, 0, STARK_VENDOR_ID, STARK_PRODUCT_CODE, 0x7000, 0x01, NULL, NULL},
    {0, 0, STARK_VENDOR_ID, STARK_PRODUCT_CODE, 0x6000, 0x01, NULL, NULL},
    {}};

/****************************************************************************/
// PDO Configuration Functions Implementation
/****************************************************************************/

const ec_pdo_entry_reg_t *get_domain_regs(pdo_config_type_t config_type,
                                          unsigned int *off_in_ptr,
                                          unsigned int *off_out_ptr,
                                          uint16_t slave_pos) {
  static unsigned int off_in, off_out;
  if (off_in_ptr)
    *off_in_ptr = off_in;
  if (off_out_ptr)
    *off_out_ptr = off_out;

  ec_pdo_entry_reg_t *regs = NULL;

  switch (config_type) {
  case PDO_CONFIG_BASIC:
    regs = basic_domain_regs;
    break;

  case PDO_CONFIG_TOUCH:
    regs = touch_domain_regs;
    break;

  case PDO_CONFIG_TOUCH_PRESSURE:
    regs = pressure_touch_domain_regs;
    break;

  default:
    regs = basic_domain_regs; // fallback to basic
    break;
  }

  // Set slave position for all entries
  if (regs) {
    for (int i = 0; regs[i].index != 0 || regs[i].subindex != 0; i++) {
      regs[i].alias = 0; // Reset alias
      regs[i].position = slave_pos; // Set slave position
    }
  }

  return regs;
}

// Build caller-provided domain_regs buffer (length >=3) with proper offsets
void build_domain_regs(pdo_config_type_t config_type, uint16_t slave_pos,
                       unsigned int *off_in_ptr, unsigned int *off_out_ptr,
                       ec_pdo_entry_reg_t *out_regs) {
  // Choose template
  ec_pdo_entry_reg_t *tmpl = NULL;
  switch (config_type) {
  case PDO_CONFIG_BASIC:
    tmpl = basic_domain_regs;
    break;
  case PDO_CONFIG_TOUCH:
    tmpl = touch_domain_regs;
    break;
  case PDO_CONFIG_TOUCH_PRESSURE:
    tmpl = pressure_touch_domain_regs;
    break;
  default:
    tmpl = basic_domain_regs;
    break;
  }

  int i = 0;
  for (; tmpl[i].index != 0 || tmpl[i].subindex != 0; ++i) {
    out_regs[i] = tmpl[i];
    out_regs[i].alias = 0;
    out_regs[i].position = slave_pos;
    // Set offset pointer based on index (0x7000 Rx -> off_out, 0x6000 Tx -> off_in)
    if (tmpl[i].index == 0x7000) {
      out_regs[i].offset = off_out_ptr;
    } else {
      out_regs[i].offset = off_in_ptr;
    }
  }
  // terminator
  out_regs[i] = {};

  // offsets will be populated by ecrt_domain_reg_pdo_entry_list
}

ec_sync_info_t *get_slave_pdo_syncs(pdo_config_type_t config_type) {
  switch (config_type) {
  case PDO_CONFIG_BASIC:
    return basic_slave_syncs;

  case PDO_CONFIG_TOUCH:
    return touch_slave_syncs;

  case PDO_CONFIG_TOUCH_PRESSURE:
    return touch_pressure_slave_syncs;

  default:
    return NULL;
  }
}

/****************************************************************************/
// Dynamic PDO Reading from Slave
/****************************************************************************/

#define MAX_PDO_MAPPINGS 16  // Maximum PDO mappings per direction (Rx or Tx)

// Helper: Read PDO mapping count from subindex 0
static int read_pdo_mapping_count(ec_master_t *master, uint16_t slave_pos,
                                   uint16_t mapping_index, uint8_t *count) {
  size_t result_size;
  uint32_t abort_code;
  int ret = ecrt_master_sdo_upload(master, slave_pos, mapping_index, 0,
                                   (uint8_t *)count, sizeof(uint8_t),
                                   &result_size, &abort_code);
  if (ret) {
    return -1; // Mapping doesn't exist or error
  }
  if (result_size != sizeof(uint8_t)) {
    return -1;
  }
  return 0;
}

// Helper: Read PDO entry from mapping subindex
static int read_pdo_entry(ec_master_t *master, uint16_t slave_pos,
                           uint16_t mapping_index, uint8_t subindex,
                           uint32_t *entry) {
  size_t result_size;
  uint32_t abort_code;
  uint32_t raw_entry;
  int ret = ecrt_master_sdo_upload(master, slave_pos, mapping_index, subindex,
                                   (uint8_t *)&raw_entry, sizeof(uint32_t),
                                   &result_size, &abort_code);
  if (ret) {
    return -1;
  }
  if (result_size != sizeof(uint32_t)) {
    return -1;
  }
  // EtherCAT PDO entry format: [31:24] bit_length, [23:16] subindex, [15:0] index
  // The data from ecrt_master_sdo_upload is in network byte order (big-endian)
  // Convert entire 32-bit value from network to host byte order
  *entry = ntohl(raw_entry);
  return 0;
}

// Parse 32-bit PDO entry: bits 0-15=index, 16-23=subindex, 24-31=bit_length
// EtherCAT PDO entry format: [31:24] bit_length, [23:16] subindex, [15:0] index
// After ntohl, the entry is in host byte order, so we can extract fields directly
static void parse_pdo_entry(uint32_t entry, uint16_t *index, uint8_t *subindex,
                            uint8_t *bit_length) {
  // Extract fields (entry is already in host byte order after ntohl)
  *index = (uint16_t)(entry & 0xFFFF);
  *subindex = (uint8_t)((entry >> 16) & 0xFF);
  *bit_length = (uint8_t)((entry >> 24) & 0xFF);
}

// Note: read_pdo_mappings function removed - not used in current implementation
// We directly read specific PDO mappings (0x1600, 0x1601, 0x1A00, 0x1A01) instead

// Helper: Read all entries from a PDO mapping and construct ec_pdo_entry_info_t array
static int read_pdo_mapping_entries(ec_master_t *master, uint16_t slave_pos,
                                     uint16_t pdo_mapping_idx, uint8_t entry_count,
                                     ec_pdo_entry_info_t *out_entries, int max_entries) {
  if (entry_count == 0 || entry_count > max_entries) {
    return 0;
  }
  
  int read_count = 0;
  for (uint8_t subidx = 1; subidx <= entry_count && read_count < max_entries; subidx++) {
    uint32_t pdo_entry;
    if (read_pdo_entry(master, slave_pos, pdo_mapping_idx, subidx, &pdo_entry) != 0) {
      // Error reading entry - log but continue (some entries may be optional)
      EC_LOG_TIME_PREFIX(stdout);
      printf("    Warning: Failed to read PDO entry 0x%04X:%u, skipping\n", pdo_mapping_idx, subidx);
      continue; // Skip this entry and continue
    }
    
    uint16_t entry_index;
    uint8_t entry_subindex, entry_bit_length;
    parse_pdo_entry(pdo_entry, &entry_index, &entry_subindex, &entry_bit_length);
    
    // Skip invalid entries
    if (entry_index == 0 || entry_bit_length == 0) {
      continue;
    }
    
    out_entries[read_count].index = entry_index;
    out_entries[read_count].subindex = entry_subindex;
    out_entries[read_count].bit_length = entry_bit_length;
    
    // Debug: Print each entry
    EC_LOG_TIME_PREFIX(stdout);
    printf("      Entry[%d]: 0x%04X:0x%02X (bit_length=%u)\n", 
           read_count, entry_index, entry_subindex, entry_bit_length);
    
    read_count++;
  }
  
  return read_count;
}

// Helper: Read first PDO entry from a specific PDO mapping (simplified approach)
// Only reads the first entry (subindex 1) to match hardcoded config pattern
static int read_pdo_mapping_first_entry(ec_master_t *master, uint16_t slave_pos,
                                         uint16_t pdo_mapping_idx, uint8_t *entry_count,
                                         ec_pdo_entry_reg_t *out_reg, 
                                         uint32_t vendor_id,
                                         uint32_t product_code) {
  // Read PDO mapping count (subindex 0)
  if (read_pdo_mapping_count(master, slave_pos, pdo_mapping_idx, entry_count) != 0) {
    return 0; // Mapping doesn't exist
  }

  if (*entry_count == 0) {
    return 0; // Empty mapping
  }

  // Only read the first entry (subindex 1) - matches hardcoded config pattern
  uint32_t pdo_entry;
  if (read_pdo_entry(master, slave_pos, pdo_mapping_idx, 1, &pdo_entry) != 0) {
    return 0; // Error reading first entry
  }

  // Debug: Print raw entry value
  EC_LOG_TIME_PREFIX(stdout);
  printf("    Raw PDO entry (0x%04X:1): 0x%08X\n", pdo_mapping_idx, pdo_entry);

  uint16_t entry_index;
  uint8_t entry_subindex, entry_bit_length;
  parse_pdo_entry(pdo_entry, &entry_index, &entry_subindex, &entry_bit_length);
  
  // Debug: Print parsed values before conversion
  EC_LOG_TIME_PREFIX(stdout);
  printf("    Parsed (before ntohs): index=0x%04X, subindex=0x%02X, bit_length=%u\n",
         (uint16_t)(pdo_entry & 0xFFFF), entry_subindex, entry_bit_length);

  // Skip invalid entries (index 0 or bit_length 0)
  if (entry_index == 0 || entry_bit_length == 0) {
    return 0;
  }

  // Add to domain registration (only first entry, matching hardcoded pattern)
  // Note: Match hardcoded config exactly: alias=0, position=0 (will be set by caller)
  // The position field in the template is 0, and build_domain_regs sets it to slave_pos
  // But for domain registration, position should match the slave position used in ecrt_master_slave_config
  out_reg->alias = 0;
  out_reg->position = slave_pos;
  out_reg->vendor_id = vendor_id;
  out_reg->product_code = product_code;
  out_reg->index = entry_index;
  out_reg->subindex = entry_subindex;

  EC_LOG_TIME_PREFIX(stdout);
  printf("    PDO entry: 0x%04X:0x%02X (bit_length=%u)\n", 
         entry_index, entry_subindex, entry_bit_length);

  return 1;
}

/**
 * @brief Dynamically read PDO mappings from slave device via SDO (simplified)
 * 
 * This function reads PDO mappings similar to Rust implementation:
 * - SM2 (RxPDO): typically 0x1600, 0x1601, etc.
 * - SM3 (TxPDO): typically 0x1A00, 0x1A01, etc.
 * 
 * It reads the first few PDO mappings (0x1600-0x160F for Rx, 0x1A00-0x1A0F for Tx)
 * instead of scanning the entire range, making it faster and more efficient.
 * 
 * Usage example:
 * @code
 *   ec_pdo_entry_reg_t domain_regs[64];
 *   unsigned int off_in, off_out;
 *   
 *   // Ensure slave is in PREOP or higher state for SDO access
 *   int count = read_pdo_from_object_dictionary(master, slave_rt);
 *   if (count > 0) {
 *     // Register PDO entries
 *     ecrt_domain_reg_pdo_entry_list(domain, domain_regs);
 *   }
 * @endcode
 * 
 * @note The slave must be in PREOP or higher state before calling this function.
 */
int read_pdo_from_object_dictionary(ec_master_t *master, void *slave_rt_ptr) {
  ec_slave_runtime_t *slave_rt = (ec_slave_runtime_t *)slave_rt_ptr;
  if (!master || !slave_rt) {
    fprintf(stderr, "Invalid parameters for read_pdo_from_object_dictionary\n");
    return -1;
  }

  // Set master for SDO operations
  ec_sdo_set_master(master);

  EC_LOG_TIME_PREFIX(stdout);
  printf("=== Reading PDO mappings from slave %u (using SDO method) ===\n", slave_rt->slave_pos);
  printf("Note: SDO method reads from object dictionary (0x1600, 0x1601, etc.)\n");
  printf("      This is different from ioctl method which reads configured sync managers\n");

  int reg_count = 0;
  uint8_t entry_count;

  // Read first RxPDO mapping (SM2: typically 0x1600)
  // Only register first entry to match hardcoded config pattern (two entries + terminator)
  if (read_pdo_mapping_first_entry(master, slave_rt->slave_pos, 0x1600, &entry_count,
                                   &slave_rt->domain_regs[reg_count],
                                   STARK_VENDOR_ID, STARK_PRODUCT_CODE) > 0) {
    slave_rt->domain_regs[reg_count].offset = &slave_rt->off_out;
    slave_rt->domain_regs[reg_count].position = slave_rt->slave_pos; // Set position here

    EC_LOG_TIME_PREFIX(stdout);
    printf("  RxPDO 0x1600: %u entries (registered first entry: 0x%04X:0x%02X)\n", 
           entry_count, slave_rt->domain_regs[reg_count].index, 
           slave_rt->domain_regs[reg_count].subindex);
    EC_LOG_TIME_PREFIX(stdout);
    printf("    domain_regs[%d]: alias=%u, pos=%u, vendor=0x%08X, product=0x%08X, "
           "index=0x%04X, subindex=0x%02X, offset=%p\n",
           reg_count, slave_rt->domain_regs[reg_count].alias,
           slave_rt->domain_regs[reg_count].position,
           slave_rt->domain_regs[reg_count].vendor_id,
           slave_rt->domain_regs[reg_count].product_code,
           slave_rt->domain_regs[reg_count].index,
           slave_rt->domain_regs[reg_count].subindex,
           slave_rt->domain_regs[reg_count].offset);
    reg_count++;
  }

  // Read first TxPDO mapping (SM3: typically 0x1A00)
  // Only register first entry to match hardcoded config pattern
  if (read_pdo_mapping_first_entry(master, slave_rt->slave_pos, 0x1A00, &entry_count,
                                   &slave_rt->domain_regs[reg_count],
                                   STARK_VENDOR_ID, STARK_PRODUCT_CODE) > 0) {
    slave_rt->domain_regs[reg_count].offset = &slave_rt->off_in;
    slave_rt->domain_regs[reg_count].position = slave_rt->slave_pos; // Set position here

    EC_LOG_TIME_PREFIX(stdout);
    printf("  TxPDO 0x1A00: %u entries (registered first entry: 0x%04X:0x%02X)\n", 
           entry_count, slave_rt->domain_regs[reg_count].index, 
           slave_rt->domain_regs[reg_count].subindex);
    EC_LOG_TIME_PREFIX(stdout);
    printf("    domain_regs[%d]: alias=%u, pos=%u, vendor=0x%08X, product=0x%08X, "
           "index=0x%04X, subindex=0x%02X, offset=%p\n",
           reg_count, slave_rt->domain_regs[reg_count].alias,
           slave_rt->domain_regs[reg_count].position,
           slave_rt->domain_regs[reg_count].vendor_id,
           slave_rt->domain_regs[reg_count].product_code,
           slave_rt->domain_regs[reg_count].index,
           slave_rt->domain_regs[reg_count].subindex,
           slave_rt->domain_regs[reg_count].offset);
    reg_count++;
  }

  if (reg_count != 2) {
    fprintf(stderr, "Warning: Expected 2 PDO entries, got %d\n", reg_count);
    return -1;
  }

  // Add terminator entry
  slave_rt->domain_regs[reg_count] = {};
  
  EC_LOG_TIME_PREFIX(stdout);
  printf("=== Domain registration entries ===\n");
  for (int i = 0; i < 3; i++) {
    if (slave_rt->domain_regs[i].index == 0 && slave_rt->domain_regs[i].subindex == 0) {
      EC_LOG_TIME_PREFIX(stdout);
      printf("  domain_regs[%d]: terminator\n", i);
      break;
    }
    EC_LOG_TIME_PREFIX(stdout);
    printf("  domain_regs[%d]: alias=%u, pos=%u, vendor=0x%08X, product=0x%08X, "
           "index=0x%04X, subindex=0x%02X, offset=%p\n",
           i, slave_rt->domain_regs[i].alias,
           slave_rt->domain_regs[i].position,
           slave_rt->domain_regs[i].vendor_id,
           slave_rt->domain_regs[i].product_code,
           slave_rt->domain_regs[i].index,
           slave_rt->domain_regs[i].subindex,
           slave_rt->domain_regs[i].offset);
  }

  // Read specific PDO mappings to construct syncs: 0x1600, 0x1601 (RxPDO), 0x1A00, 0x1A01 (TxPDO)
  // Note: Basic version may not have 0x1A01, so we handle missing PDOs gracefully
  // Note: Using static storage - if multiple slaves, they will share the same syncs
  // For multi-slave support, consider using per-slave storage
  static ec_pdo_info_t rx_pdo_infos[2];  // 0x1600, 0x1601
  static ec_pdo_info_t tx_pdo_infos[2];  // 0x1A00, 0x1A01 (0x1A01 may not exist in basic version)
  static ec_sync_info_t dynamic_syncs[5];  // SM0-3 + terminator
  static ec_pdo_entry_info_t rx_pdo_entries[2][64];  // Max 64 entries per PDO
  static ec_pdo_entry_info_t tx_pdo_entries[2][64];  // Max 64 entries per PDO
  
  uint16_t rx_pdo_indices[] = {0x1600, 0x1601};
  uint16_t tx_pdo_indices[] = {0x1A00, 0x1A01};
  int rx_count = 0;
  int tx_count = 0;
  
  // Read RxPDO mappings (0x1600, 0x1601)
  // Both should exist in all versions, but handle errors gracefully
  // Note: Each PDO mapping (0x1600, 0x1601) must be a separate ec_pdo_info_t
  // even if 0x1600 contains all entries, we need to split them by entry index
  // Note: Read sequentially with small delays to avoid overwhelming the slave
  for (int i = 0; i < 2; i++) {
    uint8_t entry_count;
    if (read_pdo_mapping_count(master, slave_rt->slave_pos, rx_pdo_indices[i], &entry_count) == 0 && entry_count > 0) {
      // Small delay between PDO reads to avoid I/O errors
      if (i > 0) {
        usleep(10000);  // 10ms delay between different PDO mappings
      }
      int read_entries = read_pdo_mapping_entries(master, slave_rt->slave_pos, rx_pdo_indices[i], 
                                                   entry_count, rx_pdo_entries[rx_count], 64);
      if (read_entries > 0) {
        rx_pdo_infos[rx_count].index = rx_pdo_indices[i];
        rx_pdo_infos[rx_count].n_entries = read_entries;
        rx_pdo_infos[rx_count].entries = rx_pdo_entries[rx_count];
        rx_count++;
        EC_LOG_TIME_PREFIX(stdout);
        printf("  RxPDO 0x%04X: %d entries (expected %u)\n", rx_pdo_indices[i], read_entries, entry_count);
        // Print all entries for this PDO
        for (int j = 0; j < read_entries; j++) {
          EC_LOG_TIME_PREFIX(stdout);
          printf("    [%d] index=0x%04X, subindex=0x%02X, bit_length=%u\n",
                 j, rx_pdo_entries[rx_count-1][j].index,
                 rx_pdo_entries[rx_count-1][j].subindex,
                 rx_pdo_entries[rx_count-1][j].bit_length);
        }
      } else {
        EC_LOG_TIME_PREFIX(stdout);
        printf("  RxPDO 0x%04X: failed to read entries (count=%u)\n", rx_pdo_indices[i], entry_count);
      }
    } else {
      EC_LOG_TIME_PREFIX(stdout);
      printf("  RxPDO 0x%04X: not found or empty\n", rx_pdo_indices[i]);
    }
  }
  
  // If 0x1600 has many entries but 0x1601 is empty, we may need to split 0x1600's entries
  // Check if 0x1600 has entries that should belong to 0x1601 (e.g., 0x7010 entries)
  // This handles the case where from station has all entries in 0x1600 but we need to split them
  // to match hardcoded configuration (0x1600 with 0x7000 entries, 0x1601 with 0x7010 entries)
  if (rx_count == 1 && rx_pdo_infos[0].index == 0x1600 && (unsigned int)rx_pdo_infos[0].n_entries > 3) {
    // Check if entries contain both 0x7000 and 0x7010 patterns
    bool has_0x7000 = false, has_0x7010 = false;
    for (unsigned int i = 0; i < (unsigned int)rx_pdo_infos[0].n_entries; i++) {
      if (rx_pdo_entries[0][i].index == 0x7000) has_0x7000 = true;
      if (rx_pdo_entries[0][i].index == 0x7010) has_0x7010 = true;
    }
    
    if (has_0x7000 && has_0x7010) {
      // Split entries: 0x7000 entries go to 0x1600, 0x7010 entries go to 0x1601
      int count_0x1600 = 0, count_0x1601 = 0;
      for (unsigned int i = 0; i < (unsigned int)rx_pdo_infos[0].n_entries; i++) {
        if (rx_pdo_entries[0][i].index == 0x7000) {
          rx_pdo_entries[0][count_0x1600] = rx_pdo_entries[0][i];
          count_0x1600++;
        } else if (rx_pdo_entries[0][i].index == 0x7010) {
          rx_pdo_entries[1][count_0x1601] = rx_pdo_entries[0][i];
          count_0x1601++;
        }
      }
      
      if (count_0x1600 > 0 && count_0x1601 > 0) {
        // Update 0x1600
        rx_pdo_infos[0].n_entries = count_0x1600;
        rx_pdo_infos[0].entries = rx_pdo_entries[0];
        
        // Create 0x1601
        rx_pdo_infos[1].index = 0x1601;
        rx_pdo_infos[1].n_entries = count_0x1601;
        rx_pdo_infos[1].entries = rx_pdo_entries[1];
        rx_count = 2;
        
        EC_LOG_TIME_PREFIX(stdout);
        printf("  Split RxPDO 0x1600 into 0x1600 (%d entries) and 0x1601 (%d entries)\n", 
               count_0x1600, count_0x1601);
      }
    }
  }
  
  // Read TxPDO mappings (0x1A00, 0x1A01)
  // 0x1A00 should exist in all versions, 0x1A01 may not exist in basic version
  // Add delay after RxPDO reads
  usleep(10000);  // 10ms delay between Rx and Tx PDO reads
  for (int i = 0; i < 2; i++) {
    uint8_t entry_count;
    if (read_pdo_mapping_count(master, slave_rt->slave_pos, tx_pdo_indices[i], &entry_count) == 0 && entry_count > 0) {
      // Small delay between PDO reads
      if (i > 0) {
        usleep(10000);  // 10ms delay between different PDO mappings
      }
      int read_entries = read_pdo_mapping_entries(master, slave_rt->slave_pos, tx_pdo_indices[i], 
                                                   entry_count, tx_pdo_entries[tx_count], 64);
      if (read_entries > 0) {
        tx_pdo_infos[tx_count].index = tx_pdo_indices[i];
        tx_pdo_infos[tx_count].n_entries = read_entries;
        tx_pdo_infos[tx_count].entries = tx_pdo_entries[tx_count];
        tx_count++;
        EC_LOG_TIME_PREFIX(stdout);
        printf("  TxPDO 0x%04X: %d entries (expected %u)\n", tx_pdo_indices[i], read_entries, entry_count);
        // Print all entries for this PDO
        for (int j = 0; j < read_entries; j++) {
          EC_LOG_TIME_PREFIX(stdout);
          printf("    [%d] index=0x%04X, subindex=0x%02X, bit_length=%u\n",
                 j, tx_pdo_entries[tx_count-1][j].index,
                 tx_pdo_entries[tx_count-1][j].subindex,
                 tx_pdo_entries[tx_count-1][j].bit_length);
        }
      } else {
        EC_LOG_TIME_PREFIX(stdout);
        printf("  TxPDO 0x%04X: failed to read entries (count=%u)\n", tx_pdo_indices[i], entry_count);
      }
    } else {
      if (i == 1) {
        // 0x1A01 doesn't exist (basic version), this is normal
        EC_LOG_TIME_PREFIX(stdout);
        printf("  TxPDO 0x1A01: not found (basic version, skipping)\n");
      } else {
        EC_LOG_TIME_PREFIX(stdout);
        printf("  TxPDO 0x%04X: not found or empty\n", tx_pdo_indices[i]);
      }
    }
  }
  
  EC_LOG_TIME_PREFIX(stdout);
  printf("Found %d RxPDO mappings and %d TxPDO mappings (compatible with basic/touch versions)\n", 
         rx_count, tx_count);
  
  // Construct syncs array
  dynamic_syncs[0] = {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE};
  dynamic_syncs[1] = {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE};
  dynamic_syncs[2] = {2, EC_DIR_OUTPUT, (unsigned int)rx_count, rx_count > 0 ? rx_pdo_infos : NULL, EC_WD_ENABLE};
  dynamic_syncs[3] = {3, EC_DIR_INPUT, (unsigned int)tx_count, tx_count > 0 ? tx_pdo_infos : NULL, EC_WD_DISABLE};
  dynamic_syncs[4] = {0xff};
  
  // Debug: Print constructed syncs
  EC_LOG_TIME_PREFIX(stdout);
  printf("=== Constructed syncs array ===\n");
  printf("  SM0: index=0, dir=OUTPUT, n_pdos=0\n");
  printf("  SM1: index=1, dir=INPUT, n_pdos=0\n");
  printf("  SM2: index=2, dir=OUTPUT, n_pdos=%d\n", rx_count);
  for (int i = 0; i < rx_count; i++) {
    printf("    PDO[%d]: index=0x%04X, n_entries=%u\n", 
           i, rx_pdo_infos[i].index, rx_pdo_infos[i].n_entries);
    for (unsigned int j = 0; j < rx_pdo_infos[i].n_entries; j++) {
      printf("      Entry[%u]: 0x%04X:0x%02X (bit_length=%u)\n",
             j, rx_pdo_infos[i].entries[j].index,
             rx_pdo_infos[i].entries[j].subindex,
             rx_pdo_infos[i].entries[j].bit_length);
    }
  }
  printf("  SM3: index=3, dir=INPUT, n_pdos=%d\n", tx_count);
  for (int i = 0; i < tx_count; i++) {
    printf("    PDO[%d]: index=0x%04X, n_entries=%u\n", 
           i, tx_pdo_infos[i].index, tx_pdo_infos[i].n_entries);
    for (unsigned int j = 0; j < tx_pdo_infos[i].n_entries; j++) {
      printf("      Entry[%u]: 0x%04X:0x%02X (bit_length=%u)\n",
             j, tx_pdo_infos[i].entries[j].index,
             tx_pdo_infos[i].entries[j].subindex,
             tx_pdo_infos[i].entries[j].bit_length);
    }
  }
  printf("  Terminator: 0xff\n");
  
  // Assign to slave_rt (using static storage, so it persists)
  slave_rt->syncs = dynamic_syncs;

  EC_LOG_TIME_PREFIX(stdout);
  printf("Successfully read %d PDO entries from slave %u\n", reg_count, slave_rt->slave_pos);
  printf("Constructed syncs: SM2 has %d RxPDOs, SM3 has %d TxPDOs\n", rx_count, tx_count);
  
  return reg_count;
}

/**
 * read_pdo implementation using ecrt_master_get_* APIs
 * This reads from configured sync managers (similar to Rust's ioctl method)
 */
int read_pdo_from_sync_managers(ec_master_t *master, void *slave_rt_ptr) {
  ec_slave_runtime_t *slave_rt = (ec_slave_runtime_t *)slave_rt_ptr;
  if (!master || !slave_rt) {
    fprintf(stderr, "Invalid parameters for read_pdo_from_sync_managers\n");
    return -1;
  }

  EC_LOG_TIME_PREFIX(stdout);
  printf("=== Reading PDO mappings from slave %u (using master API method) ===\n", slave_rt->slave_pos);
  printf("Note: This method reads from configured sync managers (similar to Rust ioctl)\n");

  // Static storage for syncs (same as SDO method)
  static ec_pdo_info_t rx_pdo_infos[2];
  static ec_pdo_info_t tx_pdo_infos[2];
  static ec_sync_info_t dynamic_syncs[5];
  static ec_pdo_entry_info_t rx_pdo_entries[2][64];
  static ec_pdo_entry_info_t tx_pdo_entries[2][64];

  int rx_count = 0;
  int tx_count = 0;

  // Read SM2 (RxPDO) sync manager
  ec_sync_info_t sm2_info;
  if (ecrt_master_get_sync_manager(master, slave_rt->slave_pos, 2, &sm2_info) == 0) {
    EC_LOG_TIME_PREFIX(stdout);
    printf("  SM2 (RxPDO): n_pdos=%u\n", sm2_info.n_pdos);
    
    // Read each PDO in SM2
    for (unsigned int i = 0; i < sm2_info.n_pdos && rx_count < 2; i++) {
      ec_pdo_info_t pdo_info;
      if (ecrt_master_get_pdo(master, slave_rt->slave_pos, 2, i, &pdo_info) == 0) {
        EC_LOG_TIME_PREFIX(stdout);
        printf("    PDO[%u]: index=0x%04X, n_entries=%u\n", i, pdo_info.index, pdo_info.n_entries);
        
        // Read all entries for this PDO
        int entry_count = 0;
        for (unsigned int j = 0; j < pdo_info.n_entries && entry_count < 64; j++) {
          ec_pdo_entry_info_t entry_info;
          if (ecrt_master_get_pdo_entry(master, slave_rt->slave_pos, 2, i, j, &entry_info) == 0) {
            rx_pdo_entries[rx_count][entry_count] = entry_info;
            entry_count++;
            // Note: ec_pdo_entry_info_t doesn't have a name field
            // Attempting to read name via SDO from object dictionary is not reliable
            // (many devices don't support reading names this way)
            EC_LOG_TIME_PREFIX(stdout);
            printf("      Entry[%u]: 0x%04X:0x%02X (bit_length=%u)\n",
                   j, entry_info.index, entry_info.subindex, entry_info.bit_length);
          }
        }
        
        if (entry_count > 0) {
          rx_pdo_infos[rx_count].index = pdo_info.index;
          rx_pdo_infos[rx_count].n_entries = entry_count;
          rx_pdo_infos[rx_count].entries = rx_pdo_entries[rx_count];
          rx_count++;
        }
      }
    }
  } else {
    EC_LOG_TIME_PREFIX(stdout);
    printf("  SM2 (RxPDO): failed to read\n");
  }

  // Read SM3 (TxPDO) sync manager
  ec_sync_info_t sm3_info;
  if (ecrt_master_get_sync_manager(master, slave_rt->slave_pos, 3, &sm3_info) == 0) {
    EC_LOG_TIME_PREFIX(stdout);
    printf("  SM3 (TxPDO): n_pdos=%u\n", sm3_info.n_pdos);
    
    // Read each PDO in SM3
    for (unsigned int i = 0; i < sm3_info.n_pdos && tx_count < 2; i++) {
      ec_pdo_info_t pdo_info;
      if (ecrt_master_get_pdo(master, slave_rt->slave_pos, 3, i, &pdo_info) == 0) {
        EC_LOG_TIME_PREFIX(stdout);
        printf("    PDO[%u]: index=0x%04X, n_entries=%u\n", i, pdo_info.index, pdo_info.n_entries);
        
        // Read all entries for this PDO
        int entry_count = 0;
        for (unsigned int j = 0; j < pdo_info.n_entries && entry_count < 64; j++) {
          ec_pdo_entry_info_t entry_info;
          if (ecrt_master_get_pdo_entry(master, slave_rt->slave_pos, 3, i, j, &entry_info) == 0) {
            tx_pdo_entries[tx_count][entry_count] = entry_info;
            entry_count++;
            // Note: ec_pdo_entry_info_t doesn't have a name field
            // Attempting to read name via SDO from object dictionary is not reliable
            // (many devices don't support reading names this way)
            EC_LOG_TIME_PREFIX(stdout);
            printf("      Entry[%u]: 0x%04X:0x%02X (bit_length=%u)\n",
                   j, entry_info.index, entry_info.subindex, entry_info.bit_length);
          }
        }
        
        if (entry_count > 0) {
          tx_pdo_infos[tx_count].index = pdo_info.index;
          tx_pdo_infos[tx_count].n_entries = entry_count;
          tx_pdo_infos[tx_count].entries = tx_pdo_entries[tx_count];
          tx_count++;
        }
      }
    }
  } else {
    EC_LOG_TIME_PREFIX(stdout);
    printf("  SM3 (TxPDO): failed to read\n");
  }

  // Auto-detect pdo_config_type_t based on read PDO information
  // Logic:
  // - BASIC: Only 0x1A00 (no 0x1A01)
  // - TOUCH: 0x1A00 + 0x1A01 with 5 entries
  // - TOUCH_PRESSURE: 0x1A00 + 0x1A01 with 10 entries
  pdo_config_type_t detected_type = PDO_CONFIG_BASIC;
  if (tx_count >= 2) {
    // Check if 0x1A01 exists and its entry count
    for (int i = 0; i < tx_count; i++) {
      if (tx_pdo_infos[i].index == 0x1A01) {
        if (tx_pdo_infos[i].n_entries == 5) {
          detected_type = PDO_CONFIG_TOUCH;
          EC_LOG_TIME_PREFIX(stdout);
          printf("Detected PDO type: TOUCH (0x1A01 has 5 entries)\n");
        } else if (tx_pdo_infos[i].n_entries == 10) {
          detected_type = PDO_CONFIG_TOUCH_PRESSURE;
          EC_LOG_TIME_PREFIX(stdout);
          printf("Detected PDO type: TOUCH_PRESSURE (0x1A01 has 10 entries)\n");
        } else {
          EC_LOG_TIME_PREFIX(stdout);
          printf("Warning: 0x1A01 has unexpected entry count (%u), defaulting to BASIC\n",
                 tx_pdo_infos[i].n_entries);
        }
        break;
      }
    }
  } else if (tx_count == 1) {
    EC_LOG_TIME_PREFIX(stdout);
    printf("Detected PDO type: BASIC (only 0x1A00, no 0x1A01)\n");
  }

  // Update slave_rt->pdo_type with detected type
  slave_rt->pdo_type = detected_type;

  // Construct syncs array
  dynamic_syncs[0] = {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE};
  dynamic_syncs[1] = {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE};
  dynamic_syncs[2] = {2, EC_DIR_OUTPUT, (unsigned int)rx_count, rx_count > 0 ? rx_pdo_infos : NULL, EC_WD_ENABLE};
  dynamic_syncs[3] = {3, EC_DIR_INPUT, (unsigned int)tx_count, tx_count > 0 ? tx_pdo_infos : NULL, EC_WD_DISABLE};
  dynamic_syncs[4] = {0xff};

  // Assign to slave_rt
  slave_rt->syncs = dynamic_syncs;

  EC_LOG_TIME_PREFIX(stdout);
  printf("Constructed syncs: SM2 has %d RxPDOs, SM3 has %d TxPDOs\n", rx_count, tx_count);

  // For domain registration, we still need to register entries
  // This is a simplified version - you may need to adapt based on your needs
  int reg_count = 0;
  if (rx_count > 0 && rx_pdo_infos[0].n_entries > 0) {
    slave_rt->domain_regs[reg_count] = {
        0, slave_rt->slave_pos, STARK_VENDOR_ID, STARK_PRODUCT_CODE,
        rx_pdo_infos[0].entries[0].index,
        rx_pdo_infos[0].entries[0].subindex,
        &slave_rt->off_out
    };
    reg_count++;
  }
  if (tx_count > 0 && tx_pdo_infos[0].n_entries > 0) {
    slave_rt->domain_regs[reg_count] = {
        0, slave_rt->slave_pos, STARK_VENDOR_ID, STARK_PRODUCT_CODE,
        tx_pdo_infos[0].entries[0].index,
        tx_pdo_infos[0].entries[0].subindex,
        &slave_rt->off_in
    };
    reg_count++;
  }
  slave_rt->domain_regs[reg_count] = {};

  return reg_count;
}
