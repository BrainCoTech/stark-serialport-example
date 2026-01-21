// Revo2 CANFD touch example (ported from revo2_touch.cpp).
#include "can_common.h"
#include "stark-sdk.h"
#include "stark_common.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// Function declarations
void get_device_info(DeviceHandler *handle, uint8_t slave_id);
void get_info(DeviceHandler *handle, uint8_t slave_id);
bool enable_touch_raw_data(void);
void print_touch_status(DeviceHandler *handle, uint8_t slave_id);
void print_touch_raw_data(DeviceHandler *handle, uint8_t slave_id);

int main(int argc, char const *argv[]) {
  // Setup signal handlers for crash debugging
  setup_signal_handlers();

  // Setup CANFD (device + channel + callbacks)
  if (!setup_canfd()) {
    return -1;
  }

  // Initialize STARK SDK
  init_cfg(STARK_PROTOCOL_TYPE_CAN_FD, LOG_LEVEL_INFO);
  const uint8_t MASTER_ID = 1; // Master device ID
  auto handle = canfd_init(MASTER_ID);
  uint8_t slave_id = 0x7f; // Default right-hand ID for Revo2 is 0x7f

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

  auto finger_id = STARK_FINGER_ID_MIDDLE;
  useconds_t delay = 1000 * 1000; // 1000ms

  // Single finger control by speed/current/PWM.
  stark_set_finger_speed(handle, slave_id, finger_id, 500); // -1000 ~ 1000
  print_touch_status(handle, slave_id);
  usleep(delay);
  stark_set_finger_current(handle, slave_id, finger_id, -300); // -1000 ~ 1000
  print_touch_status(handle, slave_id);
  usleep(delay);
  stark_set_finger_pwm(handle, slave_id, finger_id, 700); // -1000 ~ 1000
  print_touch_status(handle, slave_id);
  usleep(delay);

  // Multiple fingers control by speed/current/PWM.
  int16_t speeds[6] = {100, 100, 500, 500, 500, 500};
  stark_set_finger_speeds(handle, slave_id, speeds, 6);
  print_touch_status(handle, slave_id);
  usleep(delay);
  int16_t currents[6] = {-300, -300, -300, -300, -300, -300};
  stark_set_finger_currents(handle, slave_id, currents, 6);
  print_touch_status(handle, slave_id);
  usleep(delay);
  int16_t pwms[6] = {100, 100, 700, 700, 700, 700};
  stark_set_finger_pwms(handle, slave_id, pwms, 6);
  print_touch_status(handle, slave_id);
  usleep(delay);

  // Single finger control by position + speed/expected time (unsigned)
  stark_set_finger_position_with_millis(handle, slave_id, finger_id, 1000, 1000);
  print_touch_status(handle, slave_id);
  usleep(delay);
  stark_set_finger_position_with_speed(handle, slave_id, finger_id, 1, 50);
  print_touch_status(handle, slave_id);
  usleep(delay);

  // Multiple fingers control by position + speed/expected time (unsigned)
  uint16_t positions[6] = {300, 300, 500, 500, 500, 500};
  uint16_t durations[6] = {300, 300, 300, 300, 300, 300};
  stark_set_finger_positions_and_durations(handle, slave_id, positions, durations, 6);
  print_touch_status(handle, slave_id);
  usleep(delay);

  uint16_t positions2[6] = {30, 30, 100, 100, 100, 100};
  uint16_t speeds2[6] = {500, 500, 500, 500, 500, 500};
  stark_set_finger_positions_and_speeds(handle, slave_id, positions2, speeds2, 6);
  print_touch_status(handle, slave_id);
  usleep(delay);

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

  // Keep running and show touch status continuously after motion sequence.
  while (1) {
    print_touch_status(handle, slave_id);
    usleep(200 * 1000);
  }

  cleanup_can_resources();
  return 0;
}

bool enable_touch_raw_data(void) {
  const char *env = getenv("STARK_ENABLE_TOUCH_RAW");
  return env != NULL && strcmp(env, "1") == 0;
}

// Get device serial number, firmware version and other information
void get_device_info(DeviceHandler *handle, uint8_t slave_id) {
  auto info = stark_get_device_info(handle, slave_id);
  if (info != NULL) {
    printf("Slave[%hhu] SKU Type: %hhu, Serial Number: %s, Firmware Version: %s\n",
           slave_id, (uint8_t)info->sku_type, info->serial_number, info->firmware_version);
    if (info->hardware_type == STARK_HARDWARE_TYPE_REVO2_TOUCH) {
      // Enable all tactile sensors
      stark_enable_touch_sensor(handle, slave_id, 0x1F);
      usleep(1000 * 1000); // wait for tactile sensor to be ready
    } else {
      printf("Not Revo2 Touch, hardware type: %hhu\n", info->hardware_type);
      exit(1);
    }
    free_device_info(info);
  } else {
    printf("Error: Failed to get device info\n");
    exit(1);
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
    printf("Slave[%hhu] LED Info: %hhu, %hhu\n", slave_id, led_info->mode, led_info->color);
    free_led_info(led_info);
  }

  auto button_event = stark_get_button_event(handle, slave_id);
  if (button_event != NULL) {
    printf("Slave[%hhu] Button Event: %d, %d, %hhu\n", slave_id,
           button_event->timestamp, button_event->button_id, button_event->press_state);
    free_button_event(button_event);
  }
}

void print_touch_status(DeviceHandler *handle, uint8_t slave_id) {
  auto status = stark_get_touch_status(handle, slave_id);
  if (status == NULL) {
    printf("Error: Failed to get tactile sensor status\n");
    return;
  }

  const char *names[5] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
  for (int i = 0; i < 5; ++i) {
    auto data = status->items[i];
    printf("Slave[%hhu] %s: N=%hu T=%hu Dir=%hu P=%u Status=%hu\n", slave_id,
           names[i], data.normal_force1, data.tangential_force1,
           data.tangential_direction1, data.self_proximity1, data.status);
  }
  free_touch_finger_data(status);

  if (enable_touch_raw_data()) {
    print_touch_raw_data(handle, slave_id);
  }
}

void print_touch_raw_data(DeviceHandler *handle, uint8_t slave_id) {
  auto raw_data = stark_get_touch_raw_data(handle, slave_id);
  if (raw_data != NULL) {
    printf("Slave[%hhu] Touch Raw Data:\n", slave_id);
    printf("Thumb: %u, %u, %u, %u, %u, %u, %u\n", raw_data->thumb[0],
           raw_data->thumb[1], raw_data->thumb[2], raw_data->thumb[3],
           raw_data->thumb[4], raw_data->thumb[5], raw_data->thumb[6]);
    printf("Index: %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u\n",
           raw_data->index[0], raw_data->index[1], raw_data->index[2],
           raw_data->index[3], raw_data->index[4], raw_data->index[5],
           raw_data->index[6], raw_data->index[7], raw_data->index[8],
           raw_data->index[9], raw_data->index[10]);
    printf("Middle: %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u\n",
           raw_data->middle[0], raw_data->middle[1], raw_data->middle[2],
           raw_data->middle[3], raw_data->middle[4], raw_data->middle[5],
           raw_data->middle[6], raw_data->middle[7], raw_data->middle[8],
           raw_data->middle[9], raw_data->middle[10]);
    printf("Ring: %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u\n",
           raw_data->ring[0], raw_data->ring[1], raw_data->ring[2],
           raw_data->ring[3], raw_data->ring[4], raw_data->ring[5],
           raw_data->ring[6], raw_data->ring[7], raw_data->ring[8],
           raw_data->ring[9], raw_data->ring[10]);
    printf("Pinky: %u, %u, %u, %u, %u, %u, %u\n", raw_data->pinky[0],
           raw_data->pinky[1], raw_data->pinky[2], raw_data->pinky[3],
           raw_data->pinky[4], raw_data->pinky[5], raw_data->pinky[6]);
    free_touch_raw_data(raw_data);
  }
}
