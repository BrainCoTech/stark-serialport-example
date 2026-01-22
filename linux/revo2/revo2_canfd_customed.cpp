#include "stark-sdk.h"
#include "stark_common.h"
#include <stdio.h>
#include <unistd.h>

// Function declarations
void setup_canfd_callbacks();
void get_device_info(DeviceHandler *handle, uint8_t slave_id);

int main(int argc, char const *argv[]) {
  // Setup signal handlers for crash debugging
  setup_signal_handlers();

  setup_canfd_callbacks(); // Set CAN FD read/write callbacks

  init_cfg(STARK_PROTOCOL_TYPE_CAN_FD, LOG_LEVEL_INFO);
  const int MASTER_ID = 1;
  auto handle = canfd_init(MASTER_ID); // Only initialize a handle; the actual
                                       // CAN FD device is managed externally
  get_device_info(handle, 0x7f);
  return 0;
}

void setup_canfd_callbacks() {
  // The integrator should implement CAN FD device startup and the following
  // read/write callbacks
  set_can_tx_callback([](uint8_t slave_id, uint32_t can_id, const uint8_t *data,
                         uintptr_t data_len) -> int {
    printf("CAN FD Send: Slave ID: %d, CAN ID: %d, Data Length: %zu\n",
           slave_id, can_id, data_len);
    // TODO: sending data
    return 0; // Return 0 to indicate success
  });
  set_can_rx_callback([](uint8_t slave_id, uint32_t *can_id_out,
                         uint8_t *data_out, uintptr_t *data_len_out) -> int {
    printf("CAN FD Read: Slave ID: %d\n", slave_id);
    // TODO: Read data

    // Simulate read data returning
    *can_id_out = 0x123; // Example CAN ID
    *data_len_out = 64;  // Example data length
    for (uintptr_t i = 0; i < *data_len_out; ++i) {
      data_out[i] = i;
    }
    return 0; // Return 0 to indicate success
  });
}

void get_device_info(DeviceHandler *handle, uint8_t slave_id) {
  // Get device info and verify it's Revo2 in one call
  if (!verify_device_is_revo2(handle, slave_id)) {
    exit(1);
  }
}
