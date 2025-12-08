#include "stark-sdk.h"
#include <execinfo.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

// Function declarations
void get_touch_status(DeviceHandler *handle, uint8_t slave_id);
void *motor_pdo_thread_func(void *arg);
void *touch_pdo_thread_func(void *arg);

DeviceHandler *device_handle;
uint16_t slave_pos = 0; // Device position
bool is_touch_hand = false;

void start_pdo_thread() {
  // Create thread
  pthread_t thread_id;
  int result = pthread_create(&thread_id, NULL, motor_pdo_thread_func, NULL);
  if (result != 0) {
    fprintf(stderr, "Failed to create thread: %d\n", result);
    return;
  }
  // Detach thread so it runs in the background
  pthread_detach(thread_id);

  printf("Motor thread started.\n");
}

void start_pdo_thread_touch() {
  // Create thread
  pthread_t thread_id;
  int result = pthread_create(&thread_id, NULL, touch_pdo_thread_func, NULL);
  if (result != 0) {
    fprintf(stderr, "Failed to create thread: %d\n", result);
    return;
  }
  // Detach thread so it runs in the background
  pthread_detach(thread_id);

  printf("Touch thread started.\n");
}

int main(int argc, char const *argv[]) {
  printf("main\n");
  setup_signal_handlers();

  init_cfg(STARK_PROTOCOL_TYPE_ETHER_CAT, LOG_LEVEL_INFO);
  printf("ethercat_open_master...\n");
  device_handle = ethercat_open_master(0);
  ethercat_setup_sdo(device_handle, slave_pos);
  printf("setup_sdo done\n");

  uint8_t slave_pos_i = (uint8_t)slave_pos;
  auto info = stark_get_device_info(device_handle, slave_pos_i);
  if (info != NULL) {
    is_touch_hand = is_touch_hand_by_sn(info->serial_number);
    printf("is_touch_hand: %d\n", is_touch_hand);
    printf(
        "Slave[%hu] SKU Type: %hhu, Serial Number: %s, Firmware Version: %s\n",
        slave_pos, (uint8_t)info->sku_type, info->serial_number,
        info->firmware_version);
    free_device_info(info);
  } else {
    printf("stark_get_device_info empty\n");
  }

  // Set the unit mode for finger control parameters
  stark_set_finger_unit_mode(device_handle, slave_pos,
                             FINGER_UNIT_MODE_NORMALIZED);
  // stark_set_finger_unit_mode(device_handle, slave_pos,
  // FINGER_UNIT_MODE_PHYSICAL);

  // ethercat_start_dfu(device_handle, slave_pos, ETHER_CAT_FOE_TYPE_CONTROL,
  // "firmware.bin");

  // Configure finger protection current
  auto finger_id = STARK_FINGER_ID_MIDDLE;
  // stark_set_finger_Ced_current(device_handle, slave_pos, finger_id, 500);
  auto protected_current =
      stark_get_finger_protected_current(device_handle, slave_pos, finger_id);
  printf("Finger[%hhu] protect current: %hu\n", finger_id, protected_current);

  // usleep(1 * 1000 * 1000); // wait test
  const uint16_t slave_positions[] = {slave_pos}; // Device position array
  printf("start_loop...\n");
  ethercat_reserve_master(device_handle);

  ethercat_start_loop(device_handle, slave_positions, 1, 0, 500000, 0, 0,
                      0); // Start PDO loop thread, DC synchronization disabled
  printf("start_loop done\n");

  start_pdo_thread();
  if (is_touch_hand) {
    start_pdo_thread_touch();
  }
  usleep(500 * 1000 * 1000); // Wait for test
  printf("test done\n");
  ethercat_stop_loop(device_handle);
  printf("stop_loop done\n");
  ethercat_close(device_handle);
  return 0;
}

void *touch_pdo_thread_func(void *arg) {
  printf("Touch thread started.\n");

  while (1) {
    get_touch_status(device_handle, slave_pos);
    usleep(500); // 0.5 ms
  }

  pthread_exit(NULL);
  return NULL;
}

// Thread for controlling and retrieving motor status
void *motor_pdo_thread_func(void *arg) {
  printf("Motor thread started.\n");
  useconds_t delay = 1000 * 1000; // 1000 ms

  // Single finger control by speed/current/PWM.
  // The sign represents direction: positive for closing (grip), negative for
  // opening (release).
  auto finger_id = STARK_FINGER_ID_RING;
  stark_set_finger_speed(device_handle, slave_pos, finger_id,
                         500); // -1000 ~ 1000
  usleep(delay);               // Wait for finger to reach target position
  stark_set_finger_current(device_handle, slave_pos, finger_id,
                           -300); // -1000 ~ 1000
  usleep(delay);                  // Wait for finger to reach target position
  stark_set_finger_pwm(device_handle, slave_pos, finger_id,
                       700); // -1000 ~ 1000
  usleep(delay);             // Wait for finger to reach target position

  // Multiple fingers control by speed/current/PWM.
  // The sign represents direction: positive for closing (grip), negative for
  // opening (release).
  int16_t speeds[6] = {500, 500, 500, 500, 500, 500};
  stark_set_finger_speeds(device_handle, slave_pos, speeds, 6);
  usleep(delay); // Wait for fingers to reach target position
  int16_t currents[6] = {-300, -300, -300, -300, -300, -300};
  stark_set_finger_currents(device_handle, slave_pos, currents, 6);
  usleep(delay); // Wait for fingers to reach target position
  int16_t pwms[6] = {700, 700, 700, 700, 700, 700};
  stark_set_finger_pwms(device_handle, slave_pos, pwms, 6);
  usleep(delay); // Wait for fingers to reach target position

  // Single finger control by position + speed/expected time (unsigned)
  stark_set_finger_position_with_millis(device_handle, slave_pos, finger_id,
                                        1000, 1000);
  usleep(delay); // Wait for finger to reach target position
  stark_set_finger_position_with_speed(device_handle, slave_pos, finger_id, 1,
                                       50);
  usleep(delay); // Wait for finger to reach target position

  // Multiple fingers control by position + speed/expected time (unsigned)
  uint16_t positions[6] = {500, 500, 500, 500, 500, 500};
  uint16_t durations[6] = {300, 300, 300, 300, 300, 300};
  stark_set_finger_positions_and_durations(device_handle, slave_pos, positions,
                                           durations, 6);
  usleep(delay); // Wait for fingers to reach target position

  uint16_t positions2[6] = {100, 100, 100, 100, 100, 100};
  uint16_t speeds2[6] = {500, 500, 500, 500, 500, 500};
  stark_set_finger_positions_and_speeds(device_handle, slave_pos, positions2,
                                        speeds2, 6);
  usleep(delay); // Wait for fingers to reach target position

  while (1) {
    auto finger_status = stark_get_motor_status(device_handle, slave_pos);
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
      usleep(500); // 0.5 ms
    }
  }

  pthread_exit(NULL);
  return NULL;
}

// Get tactile sensor status, 3D force, proximity values, and sensor status
void get_touch_status(DeviceHandler *handle, uint8_t slave_id) {
  auto status = stark_get_touch_status(handle, slave_id);
  if (status != NULL) {
    auto data = status->items[3];
    printf("Slave[%hhu] Touch Sensor Status At Pink Finger:\n", slave_id);
    printf("Normal Force: %hu\n", data.normal_force1);
    printf("Tangential Force: %hu\n", data.tangential_force1);
    printf("Tangential Direction: %hu\n", data.tangential_direction1);
    printf("Proximity: %u\n", data.self_proximity1);
    printf("Status: %hu\n", data.status);

    // Free the allocated memory
    free_touch_finger_data(status);
  } else {
    printf("Error: Failed to get tactile sensor status\n");
  }
}
