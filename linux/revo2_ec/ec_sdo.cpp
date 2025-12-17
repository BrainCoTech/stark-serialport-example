/**
 * @file ec_sdo.cpp
 * @brief EtherCAT SDO operations implementation
 */

#include "ec_sdo.h"
#include <arpa/inet.h> // For htonl/ntohl
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h> // For usleep

// Global master pointer (needs to be set by the application)
static ec_master_t *g_master = NULL;

void ec_sdo_set_master(ec_master_t *master) { g_master = master; }

/****************************************************************************/
// Basic SDO read/write functions
/****************************************************************************/

int read_sdo_uint8(ec_slave_config_t *slave_config, uint16_t slave_pos,
                   uint16_t index, uint8_t subindex, uint8_t *value) {
  if (!g_master) {
    fprintf(stderr, "Master not set. Call ec_sdo_set_master() first.\n");
    return -1;
  }

  size_t result_size;
  uint32_t abort_code;

  int ret =
      ecrt_master_sdo_upload(g_master, slave_pos, index, subindex, (uint8_t *)value,
                             sizeof(uint8_t), &result_size, &abort_code);
  if (ret) {
    fprintf(stderr,
            "Failed to read SDO 0x%04X:0x%02X: %s (abort code: 0x%08X)\n",
            index, subindex, strerror(-ret), abort_code);
    return -1;
  }

  if (result_size != sizeof(uint8_t)) {
    fprintf(stderr,
            "Unexpected data size for SDO 0x%04X:0x%02X: %zu (expected %zu)\n",
            index, subindex, result_size, sizeof(uint8_t));
    return -1;
  }

  return 0;
}

int read_sdo_uint16(ec_slave_config_t *slave_config, uint16_t slave_pos,
                    uint16_t index, uint8_t subindex, uint16_t *value) {
  if (!g_master) {
    fprintf(stderr, "Master not set. Call ec_sdo_set_master() first.\n");
    return -1;
  }

  size_t result_size;
  uint32_t abort_code;

  int ret =
      ecrt_master_sdo_upload(g_master, slave_pos, index, subindex, (uint8_t *)value,
                             sizeof(uint16_t), &result_size, &abort_code);
  if (ret) {
    fprintf(stderr,
            "Failed to read SDO 0x%04X:0x%02X: %s (abort code: 0x%08X)\n",
            index, subindex, strerror(-ret), abort_code);
    return -1;
  }

  if (result_size != sizeof(uint16_t)) {
    fprintf(stderr,
            "Unexpected data size for SDO 0x%04X:0x%02X: %zu (expected %zu)\n",
            index, subindex, result_size, sizeof(uint16_t));
    return -1;
  }

  return 0;
}

int read_sdo_uint32(ec_slave_config_t *slave_config, uint16_t slave_pos,
                    uint16_t index, uint8_t subindex, uint32_t *value) {
  if (!g_master) {
    fprintf(stderr, "Master not set. Call ec_sdo_set_master() first.\n");
    return -1;
  }

  size_t result_size;
  uint32_t abort_code;

  int ret =
      ecrt_master_sdo_upload(g_master, slave_pos, index, subindex, (uint8_t *)value,
                             sizeof(uint32_t), &result_size, &abort_code);
  if (ret) {
    fprintf(stderr,
            "Failed to read SDO 0x%04X:0x%02X: %s (abort code: 0x%08X)\n",
            index, subindex, strerror(-ret), abort_code);
    return -1;
  }

  if (result_size != sizeof(uint32_t)) {
    fprintf(stderr,
            "Unexpected data size for SDO 0x%04X:0x%02X: %zu (expected %zu)\n",
            index, subindex, result_size, sizeof(uint32_t));
    return -1;
  }

  return 0;
}

int read_sdo_string(ec_slave_config_t *slave_config, uint16_t slave_pos,
                    uint16_t index, uint8_t subindex, char *buffer, size_t buffer_size) {
  if (!g_master) {
    fprintf(stderr, "Master not set. Call ec_sdo_set_master() first.\n");
    return -1;
  }

  size_t result_size;
  uint32_t abort_code;

  int ret =
      ecrt_master_sdo_upload(g_master, slave_pos, index, subindex, (uint8_t *)buffer,
                             buffer_size - 1, &result_size, &abort_code);
  if (ret) {
    fprintf(stderr,
            "Failed to read SDO 0x%04X:0x%02X: %s (abort code: 0x%08X)\n",
            index, subindex, strerror(-ret), abort_code);
    return -1;
  }

  // Ensure null termination
  buffer[result_size] = '\0';
  return 0;
}

int write_sdo_uint8(ec_slave_config_t *slave_config, uint16_t slave_pos,
                    uint16_t index, uint8_t subindex, uint8_t value) {
  if (!g_master) {
    fprintf(stderr, "Master not set. Call ec_sdo_set_master() first.\n");
    return -1;
  }

  uint32_t abort_code;

  int ret =
      ecrt_master_sdo_download(g_master, slave_pos, index, subindex, (uint8_t *)&value,
                               sizeof(uint8_t), &abort_code);
  if (ret) {
    fprintf(stderr,
            "Failed to write SDO 0x%04X:0x%02X: %s (abort code: 0x%08X)\n",
            index, subindex, strerror(-ret), abort_code);
    return -1;
  }

  return 0;
}

int write_sdo_uint16(ec_slave_config_t *slave_config, uint16_t slave_pos,
                     uint16_t index, uint8_t subindex, uint16_t value) {
  if (!g_master) {
    fprintf(stderr, "Master not set. Call ec_sdo_set_master() first.\n");
    return -1;
  }

  uint32_t abort_code;

  int ret =
      ecrt_master_sdo_download(g_master, slave_pos, index, subindex, (uint8_t *)&value,
                               sizeof(uint16_t), &abort_code);
  if (ret) {
    fprintf(stderr,
            "Failed to write SDO 0x%04X:0x%02X: %s (abort code: 0x%08X)\n",
            index, subindex, strerror(-ret), abort_code);
    return -1;
  }

  return 0;
}

int write_sdo_uint32(ec_slave_config_t *slave_config, uint16_t slave_pos,
                     uint16_t index, uint8_t subindex, uint32_t value) {
  if (!g_master) {
    fprintf(stderr, "Master not set. Call ec_sdo_set_master() first.\n");
    return -1;
  }

  uint32_t abort_code;
  uint32_t network_value =
      htonl(value); // Host to network byte order (big-endian)

  int ret = ecrt_master_sdo_download(g_master, slave_pos, index, subindex,
                                     (uint8_t *)&network_value,
                                     sizeof(uint32_t), &abort_code);
  if (ret) {
    fprintf(stderr,
            "Failed to write SDO 0x%04X:0x%02X: %s (abort code: 0x%08X)\n",
            index, subindex, strerror(-ret), abort_code);
    return -1;
  }

  return 0;
}

/****************************************************************************/
// Control functions implementation
/****************************************************************************/

int set_led_control(ec_slave_config_t *slave_config, uint16_t slave_pos, uint8_t state) {
  printf("Setting LED control to: %s\n", state == SWITCH_ON ? "ON" : "OFF");
  return write_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX, LED_SWITCH_SUBINDEX,
                         state);
}

int set_buzzer_control(ec_slave_config_t *slave_config, uint16_t slave_pos, uint8_t state) {
  printf("Setting buzzer control to: %s\n", state == SWITCH_ON ? "ON" : "OFF");
  return write_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                         BUZZER_SWITCH_SUBINDEX, state);
}

int set_vibrator_control(ec_slave_config_t *slave_config, uint16_t slave_pos, uint8_t state) {
  printf("Setting vibrator control to: %s\n",
         state == SWITCH_ON ? "ON" : "OFF");
  return write_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                         VIBRATOR_SWITCH_SUBINDEX, state);
}

int set_unit_mode(ec_slave_config_t *slave_config, uint16_t slave_pos, uint8_t mode) {
  printf("Setting unit mode to: %s\n",
         mode == PHYSICAL ? "PHYSICAL" : "NORMAL");
  return write_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX, UNIT_MODE_SUBINDEX,
                         mode);
}

int set_turbo_mode(ec_slave_config_t *slave_config, uint16_t slave_pos, switch_state_t state,
                   uint16_t interval, uint16_t duration) {
  int ret = 0;
  uint32_t turbo_param = MAKE_TURBO_PARAM(interval, duration);

  printf(
      "Setting turbo mode to: %s, interval: %u, duration: %u (param: 0x%08X)\n",
      state == SWITCH_ON ? "ON" : "OFF", interval, duration, turbo_param);

  ret |= write_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX, TURBO_MODE_SUBINDEX,
                         state);

  if (state == SWITCH_ON) {
    // Write Turbo parameter (32-bit)
    ret |= write_sdo_uint32(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                            TURBO_PARAM_SUBINDEX, turbo_param);
  }

  return ret;
}

int set_pressure_touch_data_type(ec_slave_config_t *slave_config, uint16_t slave_pos,
                                 pressure_data_type_t data_type) {
  printf("Setting pressure touch data type to: %s\n", data_type == RAW ? "RAW"
                                                      : data_type == CALIBRATED
                                                          ? "CALIBRATED"
                                                          : "FORCE");
  return write_sdo_uint8(slave_config, slave_pos, TOUCH_OBJECT_INDEX,
                         PRESSURE_DATA_TYPE_SUBINDEX, (uint8_t)data_type);
}

/****************************************************************************/
// High-level configuration functions implementation
/****************************************************************************/

int read_general_config(ec_slave_config_t *slave_config, uint16_t slave_pos,
                        general_config_t *config) {
  int ret = 0;

  ret |= read_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX, HAND_TYPE_SUBINDEX,
                        &config->hand_type);
  ret |= read_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX, LED_SWITCH_SUBINDEX,
                        &config->led_switch);
  ret |= read_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                        BUZZER_SWITCH_SUBINDEX, &config->buzzer_switch);
  ret |= read_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX, UNIT_MODE_SUBINDEX,
                        &config->unit_mode);
  ret |= read_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                        AUTO_CALIBRATION_SUBINDEX, &config->auto_calibration);
  ret |= read_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX, TURBO_MODE_SUBINDEX,
                        &config->turbo_mode);
  ret |= read_sdo_uint32(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                         TURBO_PARAM_SUBINDEX, &config->turbo_param);

  return ret;
}

int write_general_config(ec_slave_config_t *slave_config, uint16_t slave_pos,
                         const general_config_t *config) {
  int ret = 0;

  ret |= write_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX, HAND_TYPE_SUBINDEX,
                         config->hand_type);
  ret |= write_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX, LED_SWITCH_SUBINDEX,
                         config->led_switch);
  ret |= write_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                         BUZZER_SWITCH_SUBINDEX, config->buzzer_switch);
  ret |= write_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX, UNIT_MODE_SUBINDEX,
                         config->unit_mode);
  ret |= write_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                         AUTO_CALIBRATION_SUBINDEX, config->auto_calibration);
  ret |= write_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX, TURBO_MODE_SUBINDEX,
                         config->turbo_mode);
  ret |= write_sdo_uint32(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                          TURBO_PARAM_SUBINDEX, config->turbo_param);

  return ret;
}

int read_protection_current_config(ec_slave_config_t *slave_config, uint16_t slave_pos,
                                   protection_current_config_t *config) {
  int ret = 0;

  ret |=
      read_sdo_uint16(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                      THUMB_FLEX_PRO_CUR_SUBINDEX, &config->thumb_flex_pro_cur);
  ret |=
      read_sdo_uint16(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                      THUMB_AUX_PRO_CUR_SUBINDEX, &config->thumb_aux_pro_cur);
  ret |= read_sdo_uint16(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                         INDEX_PRO_CUR_SUBINDEX, &config->index_pro_cur);
  ret |= read_sdo_uint16(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                         MIDDLE_PRO_CUR_SUBINDEX, &config->middle_pro_cur);
  ret |= read_sdo_uint16(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                         RING_PRO_CUR_SUBINDEX, &config->ring_pro_cur);
  ret |= read_sdo_uint16(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                         PINKY_PRO_CUR_SUBINDEX, &config->pinky_pro_cur);

  return ret;
}

int write_protection_current_config(ec_slave_config_t *slave_config, uint16_t slave_pos,
                                    const protection_current_config_t *config) {
  int ret = 0;

  ret |=
      write_sdo_uint16(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                       THUMB_FLEX_PRO_CUR_SUBINDEX, config->thumb_flex_pro_cur);
  ret |=
      write_sdo_uint16(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                       THUMB_AUX_PRO_CUR_SUBINDEX, config->thumb_aux_pro_cur);
  ret |= write_sdo_uint16(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                          INDEX_PRO_CUR_SUBINDEX, config->index_pro_cur);
  ret |= write_sdo_uint16(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                          MIDDLE_PRO_CUR_SUBINDEX, config->middle_pro_cur);
  ret |= write_sdo_uint16(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                          RING_PRO_CUR_SUBINDEX, config->ring_pro_cur);
  ret |= write_sdo_uint16(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                          PINKY_PRO_CUR_SUBINDEX, config->pinky_pro_cur);

  return ret;
}

int read_firmware_info(ec_slave_config_t *slave_config, uint16_t slave_pos, firmware_info_t *info) {
  int ret = 0;

  ret |= read_sdo_string(
      slave_config, slave_pos, CONFIG_OBJECT_INDEX, CTRL_WRIST_FW_VERSION_SUBINDEX,
      info->ctrl_wrist_fw_version, sizeof(info->ctrl_wrist_fw_version));
  ret |= read_sdo_string(slave_config, slave_pos, CONFIG_OBJECT_INDEX, CTRL_SN_SUBINDEX,
                         info->ctrl_sn, sizeof(info->ctrl_sn));
  ret |= read_sdo_string(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                         WRIST_FW_VERSION_SUBINDEX, info->wrist_fw_version,
                         sizeof(info->wrist_fw_version));
  ret |= read_sdo_string(slave_config, slave_pos, CONFIG_OBJECT_INDEX, WRIST_SN_SUBINDEX,
                         info->wrist_sn, sizeof(info->wrist_sn));

  return ret;
}

int read_touch_config(ec_slave_config_t *slave_config, uint16_t slave_pos, touch_config_t *config) {
  int ret = 0;

  ret |= read_sdo_uint8(slave_config, slave_pos, TOUCH_OBJECT_INDEX, TOUCH_VENDOR_SUBINDEX,
                        &config->vendor_id);
  ret |= read_sdo_string(
      slave_config, slave_pos, TOUCH_OBJECT_INDEX, THUMB_TOUCH_FW_VERSION_SUBINDEX,
      config->thumb_touch_fw_version, sizeof(config->thumb_touch_fw_version));
  ret |= read_sdo_string(
      slave_config, slave_pos, TOUCH_OBJECT_INDEX, INDEX_TOUCH_FW_VERSION_SUBINDEX,
      config->index_touch_fw_version, sizeof(config->index_touch_fw_version));
  ret |= read_sdo_string(
      slave_config, slave_pos, TOUCH_OBJECT_INDEX, MID_TOUCH_FW_VERSION_SUBINDEX,
      config->middle_touch_fw_version, sizeof(config->middle_touch_fw_version));
  ret |= read_sdo_string(
      slave_config, slave_pos, TOUCH_OBJECT_INDEX, RING_TOUCH_FW_VERSION_SUBINDEX,
      config->ring_touch_fw_version, sizeof(config->ring_touch_fw_version));
  ret |= read_sdo_string(
      slave_config, slave_pos, TOUCH_OBJECT_INDEX, PINK_TOUCH_FW_VERSION_SUBINDEX,
      config->pinky_touch_fw_version, sizeof(config->pinky_touch_fw_version));
  // Note: Basic touch configuration does not include palm firmware version

  return ret;
}

int read_pressure_touch_config(ec_slave_config_t *slave_config, uint16_t slave_pos,
                               pressure_touch_config_t *config) {
  int ret = 0;

  ret |= read_sdo_uint8(slave_config, slave_pos, TOUCH_OBJECT_INDEX,
                        PRESSURE_VENDOR_SUBINDEX, &config->vendor_id);
  ret |= read_sdo_uint8(slave_config, slave_pos, TOUCH_OBJECT_INDEX,
                        PRESSURE_DATA_TYPE_SUBINDEX, &config->data_type);
  ret |= read_sdo_string(slave_config, slave_pos, TOUCH_OBJECT_INDEX,
                         PRESSURE_FW_VERSION_THUMB_SUBINDEX,
                         config->thumb_pressure_fw_version,
                         sizeof(config->thumb_pressure_fw_version));
  ret |= read_sdo_string(slave_config, slave_pos, TOUCH_OBJECT_INDEX,
                         PRESSURE_FW_VERSION_INDEX_SUBINDEX,
                         config->index_pressure_fw_version,
                         sizeof(config->index_pressure_fw_version));
  ret |= read_sdo_string(slave_config, slave_pos, TOUCH_OBJECT_INDEX,
                         PRESSURE_FW_VERSION_MIDDLE_SUBINDEX,
                         config->middle_pressure_fw_version,
                         sizeof(config->middle_pressure_fw_version));
  ret |= read_sdo_string(slave_config, slave_pos, TOUCH_OBJECT_INDEX,
                         PRESSURE_FW_VERSION_RING_SUBINDEX,
                         config->ring_pressure_fw_version,
                         sizeof(config->ring_pressure_fw_version));
  ret |= read_sdo_string(slave_config, slave_pos, TOUCH_OBJECT_INDEX,
                         PRESSURE_FW_VERSION_PINKY_SUBINDEX,
                         config->pinky_pressure_fw_version,
                         sizeof(config->pinky_pressure_fw_version));
  ret |= read_sdo_string(slave_config, slave_pos, TOUCH_OBJECT_INDEX,
                         PRESSURE_FW_VERSION_PALM_SUBINDEX,
                         config->palm_pressure_fw_version,
                         sizeof(config->palm_pressure_fw_version));

  return ret;
}

int set_auto_calibration(ec_slave_config_t *slave_config, uint16_t slave_pos,
                         switch_state_t state) {
  printf("Setting auto calibration to: %s\n",
         state == SWITCH_ON ? "ON" : "OFF");
  return write_sdo_uint8(slave_config, slave_pos, CONFIG_OBJECT_INDEX,
                         AUTO_CALIBRATION_SUBINDEX, state);
}

/****************************************************************************/
// Demo helper functions implementation
/****************************************************************************/

void print_firmware_info(ec_slave_config_t *slave_config, uint16_t slave_pos) {
  firmware_info_t fw_info;
  if (read_firmware_info(slave_config, slave_pos, &fw_info) == 0) {
    printf("\nFirmware Information:\n");
    printf("  Control Board FW: %s\n", fw_info.ctrl_wrist_fw_version);
    printf("  Control Board SN: %s\n", fw_info.ctrl_sn);
    printf("  Wrist Board FW:   %s\n", fw_info.wrist_fw_version);
    printf("  Wrist Board SN:   %s\n", fw_info.wrist_sn);
  }
}

void print_general_config(ec_slave_config_t *slave_config, uint16_t slave_pos) {
  general_config_t gen_config;
  if (read_general_config(slave_config, slave_pos, &gen_config) == 0) {
    uint16_t interval = GET_TURBO_INTERVAL(gen_config.turbo_param);
    uint16_t duration = GET_TURBO_DURATION(gen_config.turbo_param);

    printf("\nCurrent General Configuration:\n");
    printf("  Hand Type: %s\n",
           gen_config.hand_type == LEFT_HAND ? "LEFT_HAND" : "RIGHT_HAND");
    printf("  LED: %s\n", gen_config.led_switch ? "ON" : "OFF");
    printf("  Buzzer: %s\n", gen_config.buzzer_switch ? "ON" : "OFF");
    printf("  Unit Mode: %s\n", gen_config.unit_mode ? "PHYSICAL" : "NORMAL");
    printf("  Auto Calibration: %s\n",
           gen_config.auto_calibration ? "ON" : "OFF");
    printf("  Turbo Mode: %s\n", gen_config.turbo_mode ? "ON" : "OFF");
    printf("  Turbo Param: 0x%08X (interval: %u, duration: %u)\n",
           gen_config.turbo_param, interval, duration);
  }
}

void print_protection_current_config(ec_slave_config_t *slave_config, uint16_t slave_pos) {
  protection_current_config_t prot_config;
  if (read_protection_current_config(slave_config, slave_pos, &prot_config) == 0) {
    printf("\nProtection Current Configuration:\n");
    printf("  Thumb Flex: %u\n", prot_config.thumb_flex_pro_cur);
    printf("  Thumb Aux:  %u\n", prot_config.thumb_aux_pro_cur);
    printf("  Index:      %u\n", prot_config.index_pro_cur);
    printf("  Middle:     %u\n", prot_config.middle_pro_cur);
    printf("  Ring:       %u\n", prot_config.ring_pro_cur);
    printf("  Pinky:      %u\n", prot_config.pinky_pro_cur);
    printf("  Thumb Lock: %u\n", prot_config.thumb_aux_pro_cur);
  }
}

void print_touch_config(ec_slave_config_t *slave_config, uint16_t slave_pos) {
  touch_config_t touch_config;
  if (read_touch_config(slave_config, slave_pos, &touch_config) == 0) {
    printf("\nTouch Configuration:\n");
    printf("  Vendor ID: %u\n", touch_config.vendor_id);
    printf("  Thumb Touch FW: %s\n", touch_config.thumb_touch_fw_version);
    printf("  Index Touch FW: %s\n", touch_config.index_touch_fw_version);
    printf("  Middle Touch FW: %s\n", touch_config.middle_touch_fw_version);
    printf("  Ring Touch FW: %s\n", touch_config.ring_touch_fw_version);
    printf("  Pinky Touch FW: %s\n", touch_config.pinky_touch_fw_version);
    printf("  Palm Touch FW: %s\n", touch_config.palm_touch_fw_version);
  }
}

void print_pressure_touch_config(ec_slave_config_t *slave_config, uint16_t slave_pos) {
  pressure_touch_config_t pressure_config;
  if (read_pressure_touch_config(slave_config, slave_pos, &pressure_config) == 0) {
    printf("\nPressure Touch Configuration:\n");
    printf("  Vendor ID: %u\n", pressure_config.vendor_id);
    printf("  Thumb Pressure FW: %s\n",
           pressure_config.thumb_pressure_fw_version);
    printf("  Index Pressure FW: %s\n",
           pressure_config.index_pressure_fw_version);
    printf("  Middle Pressure FW: %s\n",
           pressure_config.middle_pressure_fw_version);
    printf("  Ring Pressure FW: %s\n",
           pressure_config.ring_pressure_fw_version);
    printf("  Pinky Pressure FW: %s\n",
           pressure_config.pinky_pressure_fw_version);
    printf("  Palm Pressure FW: %s\n",
           pressure_config.palm_pressure_fw_version);
  }
}

void demo_basic_controls(ec_slave_config_t *slave_config, uint16_t slave_pos) {
  printf("\n=== Demonstrating Configuration Changes ===\n");

  // Set LED control
  set_led_control(slave_config, slave_pos, SWITCH_ON);
  usleep(500000); // 500ms
  set_led_control(slave_config, slave_pos, SWITCH_OFF);

  // Set buzzer control
  set_buzzer_control(slave_config, slave_pos, SWITCH_ON);
  usleep(500000); // 500ms
  set_buzzer_control(slave_config, slave_pos, SWITCH_OFF);

  // Set unit mode
  set_unit_mode(slave_config, slave_pos, PHYSICAL);
  usleep(500000); // 500ms
  set_unit_mode(slave_config, slave_pos, NORMAL);

  // Set Turbo mode
  set_turbo_mode(slave_config, slave_pos, SWITCH_ON, 1000, 2000);
  usleep(500000); // 500ms
  set_turbo_mode(slave_config, slave_pos, SWITCH_OFF, 0, 0);
}
