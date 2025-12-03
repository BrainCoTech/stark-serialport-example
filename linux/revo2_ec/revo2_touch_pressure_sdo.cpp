#include "ecrt.h"
#include <arpa/inet.h> // For htonl/ntohl
#include <errno.h>
#include <execinfo.h>
#include <malloc.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <thread>
#include <time.h>
#include <unistd.h>

/****************************************************************************/
// Macro definitions
/****************************************************************************/
#define FREQUENCY 1000 // 1000 Hz
#define MAX_WAIT_LOOP FREQUENCY * 0.1
#define CLOCK_TO_USE CLOCK_MONOTONIC

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define DIFF_NS(A, B)                                                          \
  (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

#define STARK_VENDOR_ID 0x00bc0000
#define STARK_PRODUCT_CODE 0x00009252

// SDO configuration object index definitions
#define CONFIG_OBJECT_INDEX 0x8000
#define TOUCH_OBJECT_INDEX 0x8010

// Pressure-sensitive tactile configuration sub-indices
#define THUMB_CALIBRATION_SUBINDEX 0x01
#define INDEX_CALIBRATION_SUBINDEX 0x02
#define MID_CALIBRATION_SUBINDEX 0x03
#define RING_CALIBRATION_SUBINDEX 0x04
#define PINKY_CALIBRATION_SUBINDEX 0x05
#define CALIBRATION_PALM_SUBINDEX 0x06
#define SWITCH_THUMB_SUBINDEX 0x07
#define SWITCH_INDEX_SUBINDEX 0x08
#define SWITCH_MIDDLE_SUBINDEX 0x09
#define SWITCH_RING_SUBINDEX 0x0A
#define SWITCH_PINKY_SUBINDEX 0x0B
#define SWITCH_PALM_SUBINDEX 0x0C
#define VENDOR_SUBINDEX 0x0D
#define DATA_TYPE_SUBINDEX 0x0E
#define DEVICE_SN_THUMB_SUBINDEX 0x0F
#define DEVICE_SN_INDEX_SUBINDEX 0x10
#define DEVICE_SN_MIDDLE_SUBINDEX 0x11
#define DEVICE_SN_RING_SUBINDEX 0x12
#define DEVICE_SN_PINKY_SUBINDEX 0x13
#define DEVICE_SN_PALM_SUBINDEX 0x14
#define FW_VERSION_THUMB_SUBINDEX 0x15
#define FW_VERSION_INDEX_SUBINDEX 0x16
#define FW_VERSION_MIDDLE_SUBINDEX 0x17
#define FW_VERSION_RING_SUBINDEX 0x18
#define FW_VERSION_PINKY_SUBINDEX 0x19
#define FW_VERSION_PALM_SUBINDEX 0x1A

// General configuration sub-indices
#define HAND_TYPE_SUBINDEX 0x01
#define LED_SWITCH_SUBINDEX 0x02
#define BUZZER_SWITCH_SUBINDEX 0x03
#define VIBRATOR_SWITCH_SUBINDEX 0x04
#define UNIT_MODE_SUBINDEX 0x05
#define POS_CALIBRATION_SUBINDEX 0x06
#define AUTO_CALIBRATION_SUBINDEX 0x07
#define TURBO_MODE_SUBINDEX 0x08
#define TURBO_PARAM_SUBINDEX 0x09

// Protection current sub-indices
#define THUMB_FLEX_PRO_CUR_SUBINDEX 0x0A
#define THUMB_AUX_PRO_CUR_SUBINDEX 0x0B
#define INDEX_PRO_CUR_SUBINDEX 0x0C
#define MID_PRO_CUR_SUBINDEX 0x0D
#define RING_PRO_CUR_SUBINDEX 0x0E
#define PINK_PRO_CUR_SUBINDEX 0x0F
#define THUMB_AUX_LOCK_CUR_SUBINDEX 0x10

// Firmware information sub-indices
#define CTRL_WRIST_FW_VERSION_SUBINDEX 0x11
#define CTRL_SN_SUBINDEX 0x12
#define WRIST_FW_VERSION_SUBINDEX 0x13
#define WRIST_SN_SUBINDEX 0x14

/****************************************************************************/
// Enum definitions
/****************************************************************************/

// Hand type enum
typedef enum { LEFT_HAND = 0, RIGHT_HAND = 1 } hand_type_t;

// Switch state enum
typedef enum { SWITCH_OFF = 0, SWITCH_ON = 1 } switch_state_t;

// Unit mode enum
typedef enum {
  NORMAL = 0,  // Dimensionless
  PHYSICAL = 1 // Physical units
} unit_mode_t;

// Pressure-sensitive data type enum
typedef enum {
  RAW = 0,        // Raw ADC value
  CALIBRATED = 1, // Calibrated value (pressure/kPa)
  FORCE = 2       // Converted to force value (unit: mN)
} touch_data_type_t;

/****************************************************************************/
// Struct definitions
/****************************************************************************/

// Protection current configuration struct
typedef struct {
  uint16_t thumb_flex_pro_cur; // Thumb fingertip protection current
  uint16_t thumb_aux_pro_cur;  // Thumb base protection current
  uint16_t index_pro_cur;      // Index finger protection current
  uint16_t mid_pro_cur;        // Middle finger protection current
  uint16_t ring_pro_cur;       // Ring finger protection current
  uint16_t pink_pro_cur;       // Pinky finger protection current
  uint16_t thumb_aux_lock_cur; // Thumb auxiliary joint lock current
} protection_current_config_t;

// General configuration struct
typedef struct {
  uint8_t hand_type;        // Hand type
  uint8_t led_switch;       // LED switch
  uint8_t buzzer_switch;    // Buzzer switch
  uint8_t vibrator_switch;  // Vibration motor switch
  uint8_t unit_mode;        // Unit mode
  uint8_t auto_calibration; // Auto calibration
  uint8_t turbo_mode;       // Turbo mode
  uint32_t turbo_param;     // Turbo parameter (32-bit: duration[16] + interval[16])
} general_config_t;

// Tactile sensor - pressure-sensitive configuration struct
typedef struct {
  uint8_t vendor_id;               // Vendor information
  uint8_t data_type;               // Data type

  char thumb_touch_fw_version[21]; // Thumb tactile firmware version, 20 bytes + null terminator
  char index_touch_fw_version[21]; // Index finger tactile firmware version, 20 bytes + null terminator
  char mid_touch_fw_version[21];   // Middle finger tactile firmware version, 20 bytes + null terminator
  char ring_touch_fw_version[21];  // Ring finger tactile firmware version, 20 bytes + null terminator
  char pinky_touch_fw_version[21]; // Pinky finger tactile firmware version, 20 bytes + null terminator
  char palm_touch_fw_version[21];  // Palm tactile firmware version, 20 bytes + null terminator

  char thumb_touch_sn[21]; // Thumb tactile sensor serial number, 20 bytes + null terminator
  char index_touch_sn[21]; // Index finger tactile sensor serial number, 20 bytes + null terminator
  char mid_touch_sn[21];   // Middle finger tactile sensor serial number, 20 bytes + null terminator
  char ring_touch_sn[21];  // Ring finger tactile sensor serial number, 20 bytes + null terminator
  char pinky_touch_sn[21]; // Pinky finger tactile sensor serial number, 20 bytes + null terminator
  char palm_touch_sn[21];  // Palm tactile sensor serial number, 20 bytes + null terminator
} touch_config_t;

// Helper function: create turbo parameter
uint32_t make_turbo_param(uint16_t interval, uint16_t duration) {
  return ((uint32_t)interval << 16) | duration;
}

// Helper function: parse turbo parameter
void parse_turbo_param(uint32_t turbo_param, uint16_t *interval, uint16_t *duration)
{
  *interval = (turbo_param >> 16) & 0xFFFF;
  *duration = turbo_param & 0xFFFF;
}

// Firmware information struct
typedef struct {
  char ctrl_wrist_fw_version[21]; // Controller board firmware version, 20 bytes + null terminator
  char wrist_fw_version[21];      // Wrist board firmware version, 20 bytes + null terminator

  char ctrl_sn[19];               // Controller board serial number, 18 bytes + null terminator
  char wrist_sn[19];              // Wrist board serial number, 18 bytes + null terminator
} firmware_info_t;

/****************************************************************************/
// Global variables
/****************************************************************************/

static ec_master_t *master = NULL;
static ec_slave_config_t *slave_config = NULL;

/****************************************************************************/
// SDO read functions
/****************************************************************************/

// Generic SDO read function
int read_sdo_generic(uint16_t index, uint8_t subindex, void *value, size_t size, bool big_endian)
{
  size_t result_size;
  uint32_t abort_code;

  int ret = ecrt_master_sdo_upload(master, 0, index, subindex,
                                   (uint8_t *)value, size,
                                   &result_size, &abort_code);

  if (ret)
  {
    printf("Failed to read SDO 0x%04X:0x%02X - Error: %d, Abort code: 0x%08X\n",
           index, subindex, ret, abort_code);
    return ret;
  }

  if (result_size != size)
  {
    printf("Unexpected data size for SDO 0x%04X:0x%02X - Expected: %zu, Got: %zu\n",
           index, subindex, size, result_size);
    return -1;
  }

  // Byte order conversion (for multi-byte types only)
  if (big_endian && size > 1)
  {
    if (size == 2)
      *(uint16_t *)value = ntohs(*(uint16_t *)value);
    else if (size == 4)
      *(uint32_t *)value = ntohl(*(uint32_t *)value);
  }

  // Print result
  if (size == 1)
    printf("Read SDO 0x%04X:0x%02X = %u\n", index, subindex, *(uint8_t *)value);
  else if (size == 2)
    printf("Read SDO 0x%04X:0x%02X = %u\n", index, subindex, *(uint16_t *)value);
  else if (size == 4)
    printf("Read SDO 0x%04X:0x%02X = 0x%08X\n", index, subindex, *(uint32_t *)value);

  return 0;
}

// Read UINT8 SDO
int read_sdo_uint8(uint16_t index, uint8_t subindex, uint8_t *value)
{
  return read_sdo_generic(index, subindex, value, sizeof(uint8_t), false);
}

// Read UINT16 SDO
int read_sdo_uint16(uint16_t index, uint8_t subindex, uint16_t *value)
{
  return read_sdo_generic(index, subindex, value, sizeof(uint16_t), false);
}

// Read UINT32 SDO - little endian
int read_sdo_uint32_little_endian(uint16_t index, uint8_t subindex, uint32_t *value)
{
  return read_sdo_generic(index, subindex, value, sizeof(uint32_t), false);
}

// Read UINT32 SDO - big endian
int read_sdo_uint32_big_endian(uint16_t index, uint8_t subindex, uint32_t *value)
{
  return read_sdo_generic(index, subindex, value, sizeof(uint32_t), true);
}

// Read STRING SDO
int read_sdo_string(uint16_t index, uint8_t subindex, char *value,
                    size_t max_size) {
  size_t result_size;
  uint32_t abort_code;

  int ret = ecrt_master_sdo_upload(master, 0, index, subindex, (uint8_t *)value,
                                   max_size - 1, &result_size, &abort_code);

  if (ret) {
    printf("Failed to read SDO 0x%04X:0x%02X - Error: %d, Abort code: 0x%08X\n",
           index, subindex, ret, abort_code);
    return ret;
  }

  value[result_size] = '\0'; // Ensure null termination
  // printf("Read SDO 0x%04X:0x%02X = \"%s\"\n", index, subindex, value);
  return 0;
}

/****************************************************************************/
// SDO write functions
/****************************************************************************/

// Write UINT8 SDO
int write_sdo_uint8(uint16_t index, uint8_t subindex, uint8_t value) {
  uint32_t abort_code;

  int ret =
      ecrt_master_sdo_download(master, 0, index, subindex, (uint8_t *)&value,
                               sizeof(uint8_t), &abort_code);

  if (ret) {
    printf(
        "Failed to write SDO 0x%04X:0x%02X - Error: %d, Abort code: 0x%08X\n",
        index, subindex, ret, abort_code);
    return ret;
  }

  printf("Write SDO 0x%04X:0x%02X = %u\n", index, subindex, value);
  return 0;
}

// Write UINT16 SDO
int write_sdo_uint16(uint16_t index, uint8_t subindex, uint16_t value) {
  uint32_t abort_code;

  int ret =
      ecrt_master_sdo_download(master, 0, index, subindex, (uint8_t *)&value,
                               sizeof(uint16_t), &abort_code);

  if (ret) {
    printf(
        "Failed to write SDO 0x%04X:0x%02X - Error: %d, Abort code: 0x%08X\n",
        index, subindex, ret, abort_code);
    return ret;
  }

  printf("Write SDO 0x%04X:0x%02X = %u\n", index, subindex, value);
  return 0;
}

// Write UINT32 SDO
int write_sdo_uint32(uint16_t index, uint8_t subindex, uint32_t value) {
  uint32_t abort_code;
  uint32_t network_value = htonl(value); // Host to network byte order (big-endian)

  int ret = ecrt_master_sdo_download(master, 0, index, subindex,
                                     (uint8_t *)&network_value,
                                     sizeof(uint32_t), &abort_code);

  if (ret) {
    printf(
        "Failed to write SDO 0x%04X:0x%02X - Error: %d, Abort code: 0x%08X\n",
        index, subindex, ret, abort_code);
    return ret;
  }

  printf("Write SDO 0x%04X:0x%02X = 0x%08X\n", index, subindex, value);
  return 0;
}

// Write STRING SDO
int write_sdo_string(uint16_t index, uint8_t subindex, const char *value) {
  uint32_t abort_code;
  size_t len = strlen(value);

  int ret = ecrt_master_sdo_download(master, 0, index, subindex,
                                     (uint8_t *)value, len, &abort_code);

  if (ret) {
    printf(
        "Failed to write SDO 0x%04X:0x%02X - Error: %d, Abort code: 0x%08X\n",
        index, subindex, ret, abort_code);
    return ret;
  }

  printf("Write SDO 0x%04X:0x%02X = \"%s\"\n", index, subindex, value);
  return 0;
}

/****************************************************************************/
// High-level configuration functions
/****************************************************************************/

// Read tactile configuration
int read_touch_config(touch_config_t *config) {
  int ret = 0;
  printf("=== Reading Touch Configuration ===\n");
  ret |= read_sdo_uint8(TOUCH_OBJECT_INDEX, VENDOR_SUBINDEX,
                        &config->vendor_id);
                        ret |= read_sdo_uint8(TOUCH_OBJECT_INDEX, DATA_TYPE_SUBINDEX,
                        &config->data_type);

  ret |= read_sdo_string(TOUCH_OBJECT_INDEX, FW_VERSION_THUMB_SUBINDEX,
                         config->thumb_touch_fw_version,
                         sizeof(config->thumb_touch_fw_version));
  ret |= read_sdo_string(TOUCH_OBJECT_INDEX, FW_VERSION_INDEX_SUBINDEX,
                         config->index_touch_fw_version,
                         sizeof(config->index_touch_fw_version));
  ret |= read_sdo_string(TOUCH_OBJECT_INDEX, FW_VERSION_MIDDLE_SUBINDEX,
                         config->mid_touch_fw_version,
                         sizeof(config->mid_touch_fw_version));
  ret |= read_sdo_string(TOUCH_OBJECT_INDEX, FW_VERSION_RING_SUBINDEX,
                         config->ring_touch_fw_version,
                         sizeof(config->ring_touch_fw_version));
  ret |= read_sdo_string(TOUCH_OBJECT_INDEX, FW_VERSION_PINKY_SUBINDEX,
                         config->pinky_touch_fw_version,
                         sizeof(config->pinky_touch_fw_version));
  ret |= read_sdo_string(TOUCH_OBJECT_INDEX, FW_VERSION_PALM_SUBINDEX,
                         config->palm_touch_fw_version,
                         sizeof(config->palm_touch_fw_version));

  ret |= read_sdo_string(TOUCH_OBJECT_INDEX, DEVICE_SN_THUMB_SUBINDEX,
                         config->thumb_touch_sn,
                         sizeof(config->thumb_touch_sn));
  ret |= read_sdo_string(TOUCH_OBJECT_INDEX, DEVICE_SN_INDEX_SUBINDEX,
                         config->index_touch_sn,
                         sizeof(config->index_touch_sn));
  ret |= read_sdo_string(TOUCH_OBJECT_INDEX, DEVICE_SN_MIDDLE_SUBINDEX,
                         config->mid_touch_sn,
                         sizeof(config->mid_touch_sn));
  ret |= read_sdo_string(TOUCH_OBJECT_INDEX, DEVICE_SN_RING_SUBINDEX,
                         config->ring_touch_sn,
                         sizeof(config->ring_touch_sn));
  ret |= read_sdo_string(TOUCH_OBJECT_INDEX, DEVICE_SN_PINKY_SUBINDEX,
                         config->pinky_touch_sn,
                         sizeof(config->pinky_touch_sn));
  ret |= read_sdo_string(TOUCH_OBJECT_INDEX, DEVICE_SN_PALM_SUBINDEX,
                         config->palm_touch_sn,
                         sizeof(config->palm_touch_sn));
  printf("===================================\n");
  return ret;
}

// Read general configuration
int read_general_config(general_config_t *config) {
  int ret = 0;

  printf("=== Reading General Configuration ===\n");

  ret |= read_sdo_uint8(CONFIG_OBJECT_INDEX, HAND_TYPE_SUBINDEX,
                        &config->hand_type);
  ret |= read_sdo_uint8(CONFIG_OBJECT_INDEX, LED_SWITCH_SUBINDEX,
                        &config->led_switch);
  ret |= read_sdo_uint8(CONFIG_OBJECT_INDEX, BUZZER_SWITCH_SUBINDEX,
                        &config->buzzer_switch);
  ret |= read_sdo_uint8(CONFIG_OBJECT_INDEX, VIBRATOR_SWITCH_SUBINDEX,
                        &config->vibrator_switch);
  ret |= read_sdo_uint8(CONFIG_OBJECT_INDEX, UNIT_MODE_SUBINDEX,
                        &config->unit_mode);
  ret |= read_sdo_uint8(CONFIG_OBJECT_INDEX, AUTO_CALIBRATION_SUBINDEX,
                        &config->auto_calibration);
  ret |= read_sdo_uint8(CONFIG_OBJECT_INDEX, TURBO_MODE_SUBINDEX,
                        &config->turbo_mode);

  // Read Turbo parameter (32-bit, big-endian)
  ret |= read_sdo_uint32_big_endian(CONFIG_OBJECT_INDEX, TURBO_PARAM_SUBINDEX, &config->turbo_param);

  printf("=====================================\n");
  return ret;
}

// Write general configuration
int write_general_config(const general_config_t *config) {
  int ret = 0;

  printf("=== Writing General Configuration ===\n");

  ret |= write_sdo_uint8(CONFIG_OBJECT_INDEX, HAND_TYPE_SUBINDEX,
                         config->hand_type);
  ret |= write_sdo_uint8(CONFIG_OBJECT_INDEX, LED_SWITCH_SUBINDEX,
                         config->led_switch);
  ret |= write_sdo_uint8(CONFIG_OBJECT_INDEX, BUZZER_SWITCH_SUBINDEX,
                         config->buzzer_switch);
  ret |= write_sdo_uint8(CONFIG_OBJECT_INDEX, VIBRATOR_SWITCH_SUBINDEX,
                         config->vibrator_switch);
  ret |= write_sdo_uint8(CONFIG_OBJECT_INDEX, UNIT_MODE_SUBINDEX,
                         config->unit_mode);
  ret |= write_sdo_uint8(CONFIG_OBJECT_INDEX, AUTO_CALIBRATION_SUBINDEX,
                         config->auto_calibration);
  ret |= write_sdo_uint8(CONFIG_OBJECT_INDEX, TURBO_MODE_SUBINDEX,
                         config->turbo_mode);

  // Write Turbo parameter (32-bit, big-endian)
  ret |= write_sdo_uint32(CONFIG_OBJECT_INDEX, TURBO_PARAM_SUBINDEX,
                          config->turbo_param);

  printf("=====================================\n");
  return ret;
}

// Read protection current configuration
int read_protection_current_config(protection_current_config_t *config) {
  int ret = 0;

  printf("=== Reading Protection Current Configuration ===\n");

  ret |= read_sdo_uint16(CONFIG_OBJECT_INDEX, THUMB_FLEX_PRO_CUR_SUBINDEX,
                         &config->thumb_flex_pro_cur);
  ret |= read_sdo_uint16(CONFIG_OBJECT_INDEX, THUMB_AUX_PRO_CUR_SUBINDEX,
                         &config->thumb_aux_pro_cur);
  ret |= read_sdo_uint16(CONFIG_OBJECT_INDEX, INDEX_PRO_CUR_SUBINDEX,
                         &config->index_pro_cur);
  ret |= read_sdo_uint16(CONFIG_OBJECT_INDEX, MID_PRO_CUR_SUBINDEX,
                         &config->mid_pro_cur);
  ret |= read_sdo_uint16(CONFIG_OBJECT_INDEX, RING_PRO_CUR_SUBINDEX,
                         &config->ring_pro_cur);
  ret |= read_sdo_uint16(CONFIG_OBJECT_INDEX, PINK_PRO_CUR_SUBINDEX,
                         &config->pink_pro_cur);
  ret |= read_sdo_uint16(CONFIG_OBJECT_INDEX, THUMB_AUX_LOCK_CUR_SUBINDEX,
                         &config->thumb_aux_lock_cur);

  printf("===============================================\n");
  return ret;
}

// Write protection current configuration
int write_protection_current_config(const protection_current_config_t *config) {
  int ret = 0;

  printf("=== Writing Protection Current Configuration ===\n");

  ret |= write_sdo_uint16(CONFIG_OBJECT_INDEX, THUMB_FLEX_PRO_CUR_SUBINDEX,
                          config->thumb_flex_pro_cur);
  ret |= write_sdo_uint16(CONFIG_OBJECT_INDEX, THUMB_AUX_PRO_CUR_SUBINDEX,
                          config->thumb_aux_pro_cur);
  ret |= write_sdo_uint16(CONFIG_OBJECT_INDEX, INDEX_PRO_CUR_SUBINDEX,
                          config->index_pro_cur);
  ret |= write_sdo_uint16(CONFIG_OBJECT_INDEX, MID_PRO_CUR_SUBINDEX,
                          config->mid_pro_cur);
  ret |= write_sdo_uint16(CONFIG_OBJECT_INDEX, RING_PRO_CUR_SUBINDEX,
                          config->ring_pro_cur);
  ret |= write_sdo_uint16(CONFIG_OBJECT_INDEX, PINK_PRO_CUR_SUBINDEX,
                          config->pink_pro_cur);
  ret |= write_sdo_uint16(CONFIG_OBJECT_INDEX, THUMB_AUX_LOCK_CUR_SUBINDEX,
                          config->thumb_aux_lock_cur);

  printf("===============================================\n");
  return ret;
}

// Read firmware information
int read_firmware_info(firmware_info_t *info) {
  int ret = 0;

  printf("=== Reading Firmware Information ===\n");

  ret |= read_sdo_string(CONFIG_OBJECT_INDEX, CTRL_WRIST_FW_VERSION_SUBINDEX,
                         info->ctrl_wrist_fw_version,
                         sizeof(info->ctrl_wrist_fw_version));
  ret |= read_sdo_string(CONFIG_OBJECT_INDEX, CTRL_SN_SUBINDEX, info->ctrl_sn,
                         sizeof(info->ctrl_sn));
  ret |=
      read_sdo_string(CONFIG_OBJECT_INDEX, WRIST_FW_VERSION_SUBINDEX,
                      info->wrist_fw_version, sizeof(info->wrist_fw_version));
  ret |= read_sdo_string(CONFIG_OBJECT_INDEX, WRIST_SN_SUBINDEX, info->wrist_sn,
                         sizeof(info->wrist_sn));

  printf("====================================\n");
  return ret;
}

// Write serial numbers
int write_serial_numbers(const char *ctrl_sn, const char *wrist_sn) {
  int ret = 0;

  printf("=== Writing Serial Numbers ===\n");

  if (ctrl_sn) {
    ret |= write_sdo_string(CONFIG_OBJECT_INDEX, CTRL_SN_SUBINDEX, ctrl_sn);
  }

  if (wrist_sn) {
    ret |= write_sdo_string(CONFIG_OBJECT_INDEX, WRIST_SN_SUBINDEX, wrist_sn);
  }

  printf("==============================\n");
  return ret;
}

/****************************************************************************/
// Specialized functions
/****************************************************************************/

// Set hand type
int set_hand_type(hand_type_t hand_type) {
  printf("Setting hand type to: %s\n",
         hand_type == LEFT_HAND ? "LEFT_HAND" : "RIGHT_HAND");
  return write_sdo_uint8(CONFIG_OBJECT_INDEX, HAND_TYPE_SUBINDEX, hand_type);
}

// Control LED
int control_led(switch_state_t state) {
  printf("Setting LED to: %s\n", state == SWITCH_ON ? "ON" : "OFF");
  return write_sdo_uint8(CONFIG_OBJECT_INDEX, LED_SWITCH_SUBINDEX, state);
}

// Control buzzer
int control_buzzer(switch_state_t state) {
  printf("Setting buzzer to: %s\n", state == SWITCH_ON ? "ON" : "OFF");
  return write_sdo_uint8(CONFIG_OBJECT_INDEX, BUZZER_SWITCH_SUBINDEX, state);
}

// Control vibration motor
int control_vibrator(switch_state_t state) {
  printf("Setting vibrator to: %s\n", state == SWITCH_ON ? "ON" : "OFF");
  return write_sdo_uint8(CONFIG_OBJECT_INDEX, VIBRATOR_SWITCH_SUBINDEX, state);
}

// Set unit mode
int set_unit_mode(unit_mode_t mode) {
  printf("Setting unit mode to: %s\n", mode == NORMAL ? "NORMAL" : "PHYSICAL");
  return write_sdo_uint8(CONFIG_OBJECT_INDEX, UNIT_MODE_SUBINDEX, mode);
}

// Set pressure-sensitive data type
int set_touch_data_type(touch_data_type_t type) {
  printf("Setting touch data type to: %s\n", type == RAW ? "RAW" : type == CALIBRATED ? "CALIBRATED" : "FORCE");
  return write_sdo_uint8(TOUCH_OBJECT_INDEX, DATA_TYPE_SUBINDEX, type);
}

// Perform manual position calibration
int perform_manual_calibration() {
  printf("Performing manual position calibration...\n");
  return write_sdo_uint8(CONFIG_OBJECT_INDEX, POS_CALIBRATION_SUBINDEX, 1);
}

// Set auto calibration
int set_auto_calibration(switch_state_t state) {
  printf("Setting auto calibration to: %s\n",
         state == SWITCH_ON ? "ON" : "OFF");
  return write_sdo_uint8(CONFIG_OBJECT_INDEX, AUTO_CALIBRATION_SUBINDEX, state);
}

// Set Turbo mode
int set_turbo_mode(switch_state_t state, uint16_t interval, uint16_t duration) {
  int ret = 0;
  uint32_t turbo_param = make_turbo_param(interval, duration);

  printf("Setting turbo mode to: %s, interval: %u, duration: %u (param: "
         "0x%08X)\n",
         state == SWITCH_ON ? "ON" : "OFF", interval, duration, turbo_param);

  ret |= write_sdo_uint8(CONFIG_OBJECT_INDEX, TURBO_MODE_SUBINDEX, state);

  if (state == SWITCH_ON) {
    // Write Turbo parameter (32-bit, big-endian)
    ret |= write_sdo_uint32(CONFIG_OBJECT_INDEX, TURBO_PARAM_SUBINDEX,
                            turbo_param);
  }

  return ret;
}

/****************************************************************************/
// Signal handler
/****************************************************************************/

void handler(int sig) {
  void *array[10];
  size_t size;

  size = backtrace(array, 10);
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

/****************************************************************************/
// Main function
/****************************************************************************/

int main(int argc, char **argv) {
  signal(SIGSEGV, handler);
  signal(SIGABRT, handler);

  printf("=== REVO2 SDO Configuration Tool ===\n");

  // 1. Initialize EtherCAT master
  printf("Requesting master...\n");
  master = ecrt_request_master(0);
  if (!master) {
    fprintf(stderr, "Failed to request master.\n");
    return -1;
  }

  // 2. Create slave configuration
  printf("Requesting slave configuration...\n");
  slave_config = ecrt_master_slave_config(master, 0, 0, STARK_VENDOR_ID,
                                          STARK_PRODUCT_CODE);
  if (!slave_config) {
    fprintf(stderr, "Failed to get slave configuration.\n");
    ecrt_release_master(master);
    return -1;
  }

  // 3. Demonstrate various SDO operations
  printf("\n=== SDO Configuration Demo ===\n");

  // Read firmware information
  firmware_info_t fw_info;
  if (read_firmware_info(&fw_info) == 0) {
    printf("\nFirmware Information:\n");
    printf("  Control Board FW: %s\n", fw_info.ctrl_wrist_fw_version);
    printf("  Control Board SN: %s\n", fw_info.ctrl_sn);
    printf("  Wrist Board FW:   %s\n", fw_info.wrist_fw_version);
    printf("  Wrist Board SN:   %s\n", fw_info.wrist_sn);
  }

  // Read current general configuration
  general_config_t gen_config;
  if (read_general_config(&gen_config) == 0) {
    uint16_t interval, duration;
    parse_turbo_param(gen_config.turbo_param, &interval, &duration);

    printf("\nCurrent General Configuration:\n");
    printf("  Hand Type: %s\n",
           gen_config.hand_type == LEFT_HAND ? "LEFT_HAND" : "RIGHT_HAND");
    printf("  LED: %s\n", gen_config.led_switch ? "ON" : "OFF");
    printf("  Buzzer: %s\n", gen_config.buzzer_switch ? "ON" : "OFF");
    printf("  Vibrator: %s\n", gen_config.vibrator_switch ? "ON" : "OFF");
    printf("  Unit Mode: %s\n", gen_config.unit_mode ? "PHYSICAL" : "NORMAL");
    printf("  Auto Calibration: %s\n",
           gen_config.auto_calibration ? "ON" : "OFF");
    printf("  Turbo Mode: %s\n", gen_config.turbo_mode ? "ON" : "OFF");
    printf("  Turbo Param: 0x%08X (interval: %u, duration: %u)\n",
           gen_config.turbo_param, interval, duration);
  }

  // Read tactile configuration
  touch_config_t touch_config;
  if (read_touch_config(&touch_config) == 0) {
    printf("\nTouch Configuration:\n");
    printf("  Vendor ID: %d\n", touch_config.vendor_id);
    printf("  Data Type: %s\n", touch_config.data_type == RAW ? "RAW" : touch_config.data_type == CALIBRATED ? "CALIBRATED" : "FORCE");
    printf("  Thumb Touch FW: %s\n", touch_config.thumb_touch_fw_version);
    printf("  Index Touch FW: %s\n", touch_config.index_touch_fw_version);
    printf("  Middle Touch FW: %s\n", touch_config.mid_touch_fw_version);
    printf("  Ring Touch FW: %s\n", touch_config.ring_touch_fw_version);
    printf("  Pinky Touch FW: %s\n", touch_config.pinky_touch_fw_version);
    printf("  Palm Touch FW: %s\n", touch_config.palm_touch_fw_version);
    printf("  Thumb Touch SN: %s\n", touch_config.thumb_touch_sn);
    printf("  Index Touch SN: %s\n", touch_config.index_touch_sn);
    printf("  Middle Touch SN: %s\n", touch_config.mid_touch_sn);
    printf("  Ring Touch SN: %s\n", touch_config.ring_touch_sn);
    printf("  Pinky Touch SN: %s\n", touch_config.pinky_touch_sn);
    printf("  Palm Touch SN: %s\n", touch_config.palm_touch_sn);
  }

  // Read protection current configuration
  protection_current_config_t prot_config;
  if (read_protection_current_config(&prot_config) == 0) {
    printf("\nProtection Current Configuration:\n");
    printf("  Thumb Flex: %u\n", prot_config.thumb_flex_pro_cur);
    printf("  Thumb Aux:  %u\n", prot_config.thumb_aux_pro_cur);
    printf("  Index:      %u\n", prot_config.index_pro_cur);
    printf("  Middle:     %u\n", prot_config.mid_pro_cur);
    printf("  Ring:       %u\n", prot_config.ring_pro_cur);
    printf("  Pinky:      %u\n", prot_config.pink_pro_cur);
    printf("  Thumb Lock: %u\n", prot_config.thumb_aux_lock_cur);
  }

  // Demonstrate configuration changes
  printf("\n=== Configuration Demo ===\n");

  // Control LED blinking
  printf("LED blink demo...\n");
  control_led(SWITCH_ON);
  sleep(0.5);
  control_led(SWITCH_OFF);
  sleep(0.5);
  control_led(SWITCH_ON);

  // Set hand type
  // set_hand_type(RIGHT_HAND);
  // sleep(1);
  // set_hand_type(LEFT_HAND);

  // Set unit mode
  set_unit_mode(PHYSICAL);
  sleep(0.5);
  set_unit_mode(NORMAL);

  // Set pressure-sensitive data type
  set_touch_data_type(RAW);
  sleep(0.5);
  set_touch_data_type(CALIBRATED);
  sleep(0.5);
  set_touch_data_type(FORCE);

  // Set Turbo mode
  set_turbo_mode(SWITCH_ON, 1000, 2000); // interval=1000, duration=2000
  sleep(0.5);
  set_turbo_mode(SWITCH_OFF, 0, 0);

  printf("\n=== SDO Demo Completed ===\n");

  // 4. Clean up resources
  ecrt_release_master(master);
  return 0;
}
