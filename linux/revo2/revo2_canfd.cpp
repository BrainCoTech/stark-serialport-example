// This example demonstrates basic control of Revo2 with a ZLG USB-CAN FD
// device. USBCANFD-200U USBCANFD-100U USBCANFD-100U-mini You need to download
// the vendor-provided .so https://manual.zlg.cn/web/#/146
#include "can_common.h"
#include "stark-sdk.h"
#include "stark_common.h"
#include <cstring>
#include <stdio.h>
#include <unistd.h>

// Function declarations
void cleanup_resources();
void get_device_info(DeviceHandler *handle, uint8_t slave_id);

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
  // uint8_t slave_id = 0x7e; // Default left-hand ID for Revo2 is 0x7e
  uint8_t slave_id = 0x7f; // Default right-hand ID for Revo2 is 0x7f
  // uint8_t slave_id_right = 0x7f; // Default right-hand ID for Revo2 is 0x7f
  // get_device_info(handle, slave_id); return 0;
  get_device_info(handle, slave_id);
  // return 0;

  // Interface to modify hand baudrate; can also be modified via upper-computer
  // tools stark_set_canfd_baudrate(handle, slave_id_right, 2000); // Set to 2
  // Mbps stark_set_canfd_baudrate(handle, slave_id_right, 5000); // Set to 5
  // Mbps return 0;

  while (1) {
    uint16_t durations[6] = {300, 300, 300, 300, 300, 300};
    // uint16_t speeds[6] = {500, 500, 500, 500, 500, 500};
    uint16_t positions_fist_1000[] = {300, 300, 1000, 1000, 1000, 1000}; // Fist
    uint16_t positions_open[] = {0, 0, 0, 0, 0, 0}; // Open hand
    useconds_t delay = 1000 * 1000;                 // 1000 ms
    stark_set_finger_positions_and_durations(handle, slave_id,
                                             positions_fist_1000, durations, 6);
    // stark_set_finger_positions_and_durations(handle, slave_id_right,
    // positions_fist_1000, durations, 6);
    usleep(delay);
    stark_set_finger_positions_and_durations(handle, slave_id, positions_open,
                                             durations, 6);
    // stark_set_finger_positions_and_durations(handle, slave_id_right,
    // positions_open, durations, 6);
    usleep(delay);

    // Position + target speed mode
    // uint16_t speeds[6] = {500, 500, 500, 500, 500, 500};
    // stark_set_finger_positions_and_speeds(handle, slave_id, positions_fist,
    // speeds, 6); stark_set_finger_positions_and_speeds(handle, slave_id_right,
    // positions_fist, speeds, 6); usleep(delay);

    // Position mode - percentage
    // uint16_t positions_fist[] = {30, 30, 100, 100, 100, 100}; // Fist
    // stark_set_finger_positions(handle, slave_id, positions_fist, 6);
    // stark_set_finger_positions(handle, slave_id_right, positions_fist, 6);
    // usleep(delay);
    // stark_set_finger_positions(handle, slave_id, positions_open, 6);
    // stark_set_finger_positions(handle, slave_id_right, positions_open, 6);
    // usleep(delay);
  }

  // Clean up resources
  cleanup_resources();
  return 0;
}

void cleanup_resources() {
  cleanup_can_resources(); // Use common cleanup function
  printf("Resources cleaned up.\n");
}

void get_device_info(DeviceHandler *handle, uint8_t slave_id) {
  // Use common function to get and print device info
  if (!get_and_print_device_info(handle, slave_id)) {
    printf("Error: Failed to get device info\n");
    exit(1);
  }

  // Verify device is Revo2
  if (!verify_device_is_revo2(handle, slave_id)) {
    printf("Error: Device is not Revo2\n");
    exit(1);
  }
}
