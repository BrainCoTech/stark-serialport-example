/**
 * @file ec_constants.h
 * @brief EtherCAT constants and macro definitions
 * 
 * This header provides shared constants used across multiple
 * EtherCAT example programs.
 */

#ifndef EC_CONSTANTS_H
#define EC_CONSTANTS_H

#include <stdint.h>
#include "ec_macros.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
// Device identification constants
/****************************************************************************/

#define STARK_VENDOR_ID 0x00bc0000        // BrainCo vendor ID
#define STARK_PRODUCT_CODE 0x00009252     // REVO2 product code

/****************************************************************************/
// SDO object indices
/****************************************************************************/

#define CONFIG_OBJECT_INDEX 0x8000                    // General configuration and protection current
#define TOUCH_OBJECT_INDEX 0x8010                     // Touch configuration  
#define PRESSURE_TOUCH_OBJECT_INDEX 0x8010            // Pressure-sensitive touch configuration (same as touch)

/****************************************************************************/
// General configuration sub-indices (SDO 0x8000)
/****************************************************************************/

#define HAND_TYPE_SUBINDEX 0x01
#define LED_SWITCH_SUBINDEX 0x02
#define BUZZER_SWITCH_SUBINDEX 0x03
#define VIBRATOR_SWITCH_SUBINDEX 0x04
#define UNIT_MODE_SUBINDEX 0x05
#define POS_CALIBRATION_SUBINDEX 0x06
#define AUTO_CALIBRATION_SUBINDEX 0x07
#define TURBO_MODE_SUBINDEX 0x08
#define TURBO_PARAM_SUBINDEX 0x09

/****************************************************************************/
// Protection current configuration sub-indices (SDO 0x8000)
/****************************************************************************/

#define THUMB_FLEX_PRO_CUR_SUBINDEX 0x0A
#define THUMB_AUX_PRO_CUR_SUBINDEX 0x0B
#define INDEX_PRO_CUR_SUBINDEX 0x0C
#define MIDDLE_PRO_CUR_SUBINDEX 0x0D
#define RING_PRO_CUR_SUBINDEX 0x0E
#define PINKY_PRO_CUR_SUBINDEX 0x0F
#define THUMB_AUX_LOCK_CUR_SUBINDEX 0x10

/****************************************************************************/
// Firmware information sub-indices (SDO 0x8000)
/****************************************************************************/

#define CTRL_WRIST_FW_VERSION_SUBINDEX 0x11
#define CTRL_SN_SUBINDEX 0x12
#define WRIST_FW_VERSION_SUBINDEX 0x13
#define WRIST_SN_SUBINDEX 0x14

/****************************************************************************/
// Touch configuration sub-indices (SDO 0x8010)
/****************************************************************************/

#define THUMB_CALIBRATION_SUBINDEX 0x01
#define INDEX_CALIBRATION_SUBINDEX 0x02
#define MID_CALIBRATION_SUBINDEX 0x03
#define RING_CALIBRATION_SUBINDEX 0x04
#define PINKY_CALIBRATION_SUBINDEX 0x05
#define TOUCH_VENDOR_SUBINDEX 0x06
#define THUMB_ACQ_PARAM_UPDATE_SUBINDEX 0x07
#define INDEX_ACQ_PARAM_UPDATE_SUBINDEX 0x08
#define MID_ACQ_PARAM_UPDATE_SUBINDEX 0x09
#define RING_ACQ_PARAM_UPDATE_SUBINDEX 0x0A
#define PINKY_ACQ_PARAM_UPDATE_SUBINDEX 0x0B
#define RESERVED1_SUBINDEX 0x0C
#define THUMB_TOUCH_FW_VERSION_SUBINDEX 0x0D
#define INDEX_TOUCH_FW_VERSION_SUBINDEX 0x0E
#define MID_TOUCH_FW_VERSION_SUBINDEX 0x0F
#define RING_TOUCH_FW_VERSION_SUBINDEX 0x10
#define PINK_TOUCH_FW_VERSION_SUBINDEX 0x11

/****************************************************************************/
// Pressure-sensitive touch configuration sub-indices (SDO 0x8010)
/****************************************************************************/

// Calibration sub-indices
#define PRESSURE_THUMB_CALIBRATION_SUBINDEX 0x01
#define PRESSURE_INDEX_CALIBRATION_SUBINDEX 0x02
#define PRESSURE_MID_CALIBRATION_SUBINDEX 0x03
#define PRESSURE_RING_CALIBRATION_SUBINDEX 0x04
#define PRESSURE_PINKY_CALIBRATION_SUBINDEX 0x05
#define PRESSURE_CALIBRATION_PALM_SUBINDEX 0x06

// Switch control sub-indices
#define PRESSURE_SWITCH_THUMB_SUBINDEX 0x07
#define PRESSURE_SWITCH_INDEX_SUBINDEX 0x08
#define PRESSURE_SWITCH_MIDDLE_SUBINDEX 0x09
#define PRESSURE_SWITCH_RING_SUBINDEX 0x0A
#define PRESSURE_SWITCH_PINKY_SUBINDEX 0x0B
#define PRESSURE_SWITCH_PALM_SUBINDEX 0x0C

// Vendor and data type
#define PRESSURE_VENDOR_SUBINDEX 0x0D
#define PRESSURE_DATA_TYPE_SUBINDEX 0x0E

// Device serial number sub-indices
#define PRESSURE_DEVICE_SN_THUMB_SUBINDEX 0x0F
#define PRESSURE_DEVICE_SN_INDEX_SUBINDEX 0x10
#define PRESSURE_DEVICE_SN_MIDDLE_SUBINDEX 0x11
#define PRESSURE_DEVICE_SN_RING_SUBINDEX 0x12
#define PRESSURE_DEVICE_SN_PINKY_SUBINDEX 0x13
#define PRESSURE_DEVICE_SN_PALM_SUBINDEX 0x14

// Firmware version sub-indices
#define PRESSURE_FW_VERSION_THUMB_SUBINDEX 0x15
#define PRESSURE_FW_VERSION_INDEX_SUBINDEX 0x16
#define PRESSURE_FW_VERSION_MIDDLE_SUBINDEX 0x17
#define PRESSURE_FW_VERSION_RING_SUBINDEX 0x18
#define PRESSURE_FW_VERSION_PINKY_SUBINDEX 0x19
#define PRESSURE_FW_VERSION_PALM_SUBINDEX 0x1A

#ifdef __cplusplus
}
#endif

#endif // EC_CONSTANTS_H
