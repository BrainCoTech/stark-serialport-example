#include "stark-sdk.h"
#include "stark_common.h"
#include <stdio.h>
#include <unistd.h>


// Declare functions
void setup_modbus_callbacks();
void get_device_info(DeviceHandler *handle, uint8_t slave_id);

int main(int argc, char const *argv[]) {
  // Setup signal handlers for crash debugging
  setup_signal_handlers();

  setup_modbus_callbacks(); // Set read and write callbacks

  init_cfg(STARK_PROTOCOL_TYPE_MODBUS, LOG_LEVEL_INFO);
  auto handle = create_device_handler();
  uint8_t slave_id = 1;
  get_device_info(handle, slave_id);
  return 0;
}

void get_device_info(DeviceHandler *handle, uint8_t slave_id) {
  uint32_t baudrate = stark_get_rs485_baudrate(handle, slave_id);
  printf("Slave[%hhu] Baudrate: %d\n", slave_id, baudrate);
  DeviceInfo *info = stark_get_device_info(handle, slave_id);
  if (info != NULL) {
    printf("Slave[%hhu] Serial Number: %s, FW: %s\n", slave_id,
           info->serial_number, info->firmware_version);
    if (info->hardware_type != STARK_HARDWARE_TYPE_REVO1_BASIC &&
        info->hardware_type != STARK_HARDWARE_TYPE_REVO1_TOUCH) {
      printf("Not Revo1, hardware type: %hhu\n", info->hardware_type);
      exit(1);
    }
    free_device_info(info);
  }
}

void setup_modbus_callbacks() {

  set_modbus_read_holding_callback(
      [](uint8_t slave_id, uint16_t register_address, uint16_t *data_out,
         uint16_t count) -> int {
        printf("Read holding registers: Slave ID: %d, Address: %d\n", slave_id,
               register_address);
        // FIXME: Simulate reading data
        for (uint16_t i = 0; i < count; ++i) {
          data_out[i] = i;
        }
        return 0; // Return 0 to indicate success
      });
  set_modbus_read_input_callback([](uint8_t slave_id, uint16_t register_address,
                                    uint16_t *data_out, uint16_t count) -> int {
    printf("Read input registers: Slave ID: %d, Address: %d\n", slave_id,
           register_address);
    // FIXME: Simulate reading data
    for (uint16_t i = 0; i < count; ++i) {
      data_out[i] = i;
    }
    return 0; // Return 0 to indicate success
  });
  set_modbus_write_callback([](uint8_t slave_id, uint16_t register_address,
                               const uint16_t *data_in, uint16_t count) -> int {
    printf("Write holding registers: Slave ID: %d, Address: %d\n", slave_id,
           register_address);
    // FIXME: Simulate writing data
    for (uint16_t i = 0; i < count; ++i) {
      printf("Data[%d]: %d\n", i, data_in[i]);
    }
    return 0; // Return 0 to indicate success
  });
}
