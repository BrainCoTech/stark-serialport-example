#include "stark-sdk.h"
#include "stark_common.h"
#include <stdio.h>
#include <unistd.h>

// function declarations
void setup_can_callbacks();
void get_device_info(DeviceHandler *handleint, uint8_t slave_id);

int main(int argc, char const *argv[]) {
  // Setup signal handlers for crash debugging
  setup_signal_handlers();

  setup_can_callbacks(); // set read/write callbacks

  init_cfg(STARK_PROTOCOL_TYPE_CAN, LOG_LEVEL_INFO);
  auto handle = create_device_handler();
  uint8_t slave_id = 1;
  get_device_info(handle, slave_id);
  return 0;
}

void get_device_info(DeviceHandler *handle, uint8_t slave_id) {
  // Get device info and verify it's Revo1 in one call
  if (!verify_device_is_revo1(handle, slave_id)) {
    exit(1);
  }
}

void setup_can_callbacks() {
  set_can_tx_callback([](uint8_t slave_id, uint32_t can_id, const uint8_t *data,
                         uintptr_t data_len) -> int {
    printf("CAN Send: Slave ID: %d, CAN ID: %d, Data Length: %zu\n", slave_id,
           can_id, data_len);
    // TODO: sending data
    return 0; // Return 0 to indicate success
  });
  set_can_rx_callback([](uint8_t slave_id, uint32_t *can_id_out,
                         uint8_t *data_out, uintptr_t *data_len_out) -> int {
    printf("CAN Read: Slave ID: %d\n", slave_id);
    // TODO: Read data

    // Simulate read data returning
    *can_id_out = 0x123; // Example CAN ID
    *data_len_out = 8;   // Example data length
    for (uintptr_t i = 0; i < *data_len_out; ++i) {
      data_out[i] = i;
    }
    // TODO: when multi-frame response, need to拼接
    return 0; // Return 0 to indicate success
  });
}
