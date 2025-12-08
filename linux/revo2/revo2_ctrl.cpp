#include "stark-sdk.h"
#include "stark_common.h"
#include <stdio.h>
#include <unistd.h>

// Function declarations
void get_device_info(DeviceHandler *handle, uint8_t slave_id);
void get_info(DeviceHandler *handle, uint8_t slave_id);

int main(int argc, char const *argv[]) {
  // Setup signal handlers for crash debugging
  setup_signal_handlers();

  init_cfg(STARK_PROTOCOL_TYPE_MODBUS,
           LOG_LEVEL_INFO); // Initialize configuration
  auto cfg = auto_detect_modbus_revo2(
      "/dev/ttyUSB0", true); // Replace with actual serial port name; passing
                             // NULL will try auto-detection
  if (cfg == NULL) {
    fprintf(stderr, "Failed to auto-detect Modbus device configuration.\n");
    return -1;
  }
  uint8_t slave_id = cfg->slave_id;
  auto handle = modbus_open(cfg->port_name, cfg->baudrate);
  free_device_config(cfg);

  get_device_info(handle, slave_id);

  // Set the unit mode for finger control parameters
  stark_set_finger_unit_mode(handle, slave_id, FINGER_UNIT_MODE_NORMALIZED);
  // stark_set_finger_unit_mode(handle, slave_id, FINGER_UNIT_MODE_PHYSICAL);

  auto mode = stark_get_finger_unit_mode(handle, slave_id);
  if (mode == FINGER_UNIT_MODE_NORMALIZED) {
    printf("Finger unit mode: Normalized\n");
  } else if (mode == FINGER_UNIT_MODE_PHYSICAL) {
    printf("Finger unit mode: Physical\n");
  } else {
    printf("Finger unit mode: Unknown\n");
  }

  // Configure finger parameters: max angle, min angle, max speed, max current,
  // protection current; see documentation for valid ranges
  auto finger_id = STARK_FINGER_ID_MIDDLE;
  // stark_set_finger_min_position(handle, slave_id, finger_id, 0);
  // auto min_position = stark_get_finger_min_position(handle, slave_id,
  // finger_id); printf("Finger[%hhu] min position: %hu\n", finger_id,
  // min_position);

  // stark_set_finger_max_position(handle, slave_id, finger_id, 80);
  // auto max_position = stark_get_finger_max_position(handle, slave_id,
  // finger_id); printf("Finger[%hhu] max position: %hu\n", finger_id,
  // max_position);

  // stark_set_finger_max_speed(handle, slave_id, finger_id, 130);
  // auto max_speed = stark_get_finger_max_speed(handle, slave_id, finger_id);
  // printf("Finger[%hhu] max speed: %hu\n", finger_id, max_speed);

  // stark_set_finger_max_current(handle, slave_id, finger_id, 1000);
  // auto max_current = stark_get_finger_max_current(handle, slave_id,
  // finger_id); printf("Finger[%hhu] max current: %hu\n", finger_id,
  // max_current);

  // stark_set_finger_protected_current(handle, slave_id, finger_id, 500);
  // auto protected_current = stark_get_finger_protected_current(handle,
  // slave_id, finger_id); printf("Finger[%hhu] protect current: %hu\n",
  // finger_id, protected_current);

  useconds_t delay = 1000 * 1000; // 1000ms

  // Single finger control by speed/current/PWM.
  // The sign represents direction: positive for closing (grip), negative for
  // opening (release).
  stark_set_finger_speed(handle, slave_id, finger_id, 500); // -1000 ~ 1000
  usleep(delay); // Wait for finger to reach target position
  stark_set_finger_current(handle, slave_id, finger_id, -300); // -1000 ~ 1000
  usleep(delay); // Wait for finger to reach target position
  stark_set_finger_pwm(handle, slave_id, finger_id, 700); // -1000 ~ 1000
  usleep(delay); // Wait for finger to reach target position

  // Multiple fingers control by speed/current/PWM.
  // The sign represents direction: positive for closing (grip), negative for
  // opening (release).
  int16_t speeds[6] = {100, 100, 500, 500, 500, 500};
  stark_set_finger_speeds(handle, slave_id, speeds, 6);
  usleep(delay); // Wait for fingers to reach target position
  int16_t currents[6] = {-300, -300, -300, -300, -300, -300};
  stark_set_finger_currents(handle, slave_id, currents, 6);
  usleep(delay); // Wait for fingers to reach target position
  int16_t pwms[6] = {100, 100, 700, 700, 700, 700};
  stark_set_finger_pwms(handle, slave_id, pwms, 6);
  usleep(delay); // Wait for fingers to reach target position

  // Single finger control by position + speed/expected time (unsigned)
  stark_set_finger_position_with_millis(handle, slave_id, finger_id, 1000,
                                        1000);
  usleep(delay); // Wait for finger to reach target position
  stark_set_finger_position_with_speed(handle, slave_id, finger_id, 1, 50);
  usleep(delay); // Wait for finger to reach target position

  // Multiple fingers control by position + speed/expected time (unsigned)
  uint16_t positions[6] = {300, 300, 500, 500, 500, 500};
  uint16_t durations[6] = {300, 300, 300, 300, 300, 300};
  stark_set_finger_positions_and_durations(handle, slave_id, positions,
                                           durations, 6);
  usleep(delay); // Wait for fingers to reach target position

  uint16_t positions2[6] = {30, 30, 100, 100, 100, 100};
  uint16_t speeds2[6] = {500, 500, 500, 500, 500, 500};
  stark_set_finger_positions_and_speeds(handle, slave_id, positions2, speeds2,
                                        6);
  usleep(delay); // Wait for fingers to reach target position

  auto finger_status = stark_get_motor_status(handle, slave_id);
  if (finger_status != NULL) {
    printf("Positions: %hu, %hu, %hu, %hu, %hu, %hu\n",
           finger_status->positions[0], finger_status->positions[1],
           finger_status->positions[2], finger_status->positions[3],
           finger_status->positions[4], finger_status->positions[5]);
    printf("Speeds: %hd, %hd, %hd, %hd, %hd, %hd\n", finger_status->speeds[0],
           finger_status->speeds[1], finger_status->speeds[2],
           finger_status->speeds[3], finger_status->speeds[4],
           finger_status->speeds[5]);
    printf("Currents: %hd, %hd, %hd, %hd, %hd, %hd\n",
           finger_status->currents[0], finger_status->currents[1],
           finger_status->currents[2], finger_status->currents[3],
           finger_status->currents[4], finger_status->currents[5]);
    printf("States: %hhu, %hhu, %hhu, %hhu, %hhu, %hhu\n",
           finger_status->states[0], finger_status->states[1],
           finger_status->states[2], finger_status->states[3],
           finger_status->states[4], finger_status->states[5]);
    free_motor_status_data(finger_status);
  }

  return 0;
}

// Get device serial number, firmware version and other information
void get_device_info(DeviceHandler *handle, uint8_t slave_id) {
  auto info = stark_get_device_info(handle, slave_id);
  if (info != NULL) {
    printf(
        "Slave[%hhu] SKU Type: %hhu, Serial Number: %s, Firmware Version: %s\n",
        slave_id, (uint8_t)info->sku_type, info->serial_number,
        info->firmware_version);
    if (info->hardware_type == STARK_HARDWARE_TYPE_REVO1_TOUCH ||
        info->hardware_type == STARK_HARDWARE_TYPE_REVO2_TOUCH) {
      // Enable all tactile sensors
      stark_enable_touch_sensor(handle, slave_id, 0x1F);
      usleep(1000 * 1000); // wait for tactile sensor to be ready
    }
    free_device_info(info);
  }
}

// Get device information: baudrate, LED info, button events
void get_info(DeviceHandler *handle, uint8_t slave_id) {
  // RS485 serial baudrate
  auto baudrate = stark_get_rs485_baudrate(handle, slave_id);
  printf("Slave[%hhu] Baudrate: %d\n", slave_id, baudrate);

  // CANFD baudrate
  auto canfd_baudrate = stark_get_canfd_baudrate(handle, slave_id);
  printf("Slave[%hhu] CANFD Baudrate: %d\n", slave_id, canfd_baudrate);

  auto led_info = stark_get_led_info(handle, slave_id);
  if (led_info != NULL) {
    printf("Slave[%hhu] LED Info: %hhu, %hhu\n", slave_id, led_info->mode,
           led_info->color);
    free_led_info(led_info);
  }

  auto button_event = stark_get_button_event(handle, slave_id);
  if (button_event != NULL) {
    printf("Slave[%hhu] Button Event: %d, %d, %hhu\n", slave_id,
           button_event->timestamp, button_event->button_id,
           button_event->press_state);
    free_button_event(button_event);
  }
}
