/**
 * @file ec_pdo.cpp
 * @brief Implementation of EtherCAT PDO configuration and management functions
 */

#include "ec_pdo.h"
#include "ec_constants.h"
#include <stdio.h>

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
