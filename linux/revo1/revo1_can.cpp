// This example demonstrates a simple control of Revo1 combined with a ZLG
// USB-CAN device USBCANFD-200U USBCANFD-100U USBCANFD-100U-mini You need to
// download the vendor-provided .so https://manual.zlg.cn/web/#/146
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

  // Setup CAN (device + channel + callbacks)
  if (!setup_can()) {
    return -1;
  }

  // Initialize STARK SDK
  init_logging(LOG_LEVEL_INFO);
  auto handle = init_device_handler(STARK_PROTOCOL_TYPE_CAN, 0);
  uint8_t slave_id = 1; // Default slave ID for Revo1 is 1
  get_device_info(handle, slave_id);

  uint16_t positions_fist[] = {500, 500, 1000, 1000, 1000, 1000}; // Fist
  uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};                 // Open hand
  useconds_t delay = 1000 * 1000;                                 // 1000ms
  stark_set_finger_positions(handle, slave_id, positions_fist, 6);
  usleep(delay);
  stark_set_finger_positions(handle, slave_id, positions_open, 6);
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

  // Clean up resources
  cleanup_resources();
  return 0;
}

void get_device_info(DeviceHandler *handle, uint8_t slave_id) {
  // Get device info and verify it's Revo1 in one call
  if (!verify_device_is_revo1(handle, slave_id)) {
    exit(1);
  }
}

void cleanup_resources() {
  cleanup_can_resources(); // Use common cleanup function
  printf("Resources cleaned up.\n");
}
