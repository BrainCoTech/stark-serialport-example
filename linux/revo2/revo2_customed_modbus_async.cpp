#include "stark-sdk.h"
#include "stark_common.h"
#include <stdio.h>
#include <unistd.h>

// Function declarations
// print_hex is now provided by stark_common.h
int modbus_operation_async(const uint8_t *values, int len,
                           ModbusOperationResultCallback callback,
                           void *user_data);
void get_device_info(DeviceHandler *handleint, uint8_t slave_id);

struct {
  DeviceHandler *handle;
  uint8_t slave_id;
} set_modbus_operation_params;

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

  while (true) {
    printf("Performing periodic operations...\n");
    // Simulate some operations
    usleep(1000 * 1000); // Execute once per second
  }
  return 0;
}

// Thread execution function
void *get_device_info_thread(void *arg) {
  auto params = static_cast<decltype(set_modbus_operation_params) *>(arg);
  DeviceHandler *modbus_handle = params->handle;
  uint8_t slave_id = params->slave_id;

  uint32_t baudrate = stark_get_rs485_baudrate(modbus_handle, slave_id);
  printf("Slave[%hhu] Baudrate: %d\n", slave_id, baudrate);

  free(arg); // Free parameter memory
  pthread_exit(NULL);
  return NULL;
}

void get_device_info(DeviceHandler *handler, uint8_t slave_id) {
  if (!handler) {
    fprintf(stderr, "Invalid Modbus handle.\n");
    return;
  }
  printf("Getting device info for slave ID: %hhu\n", slave_id);

  // Create thread
  pthread_t thread_id;
  auto params = new decltype(set_modbus_operation_params);
  if (!params) {
    fprintf(stderr, "Failed to allocate memory for thread parameters.\n");
    return;
  }
  params->handle = handler;
  params->slave_id = slave_id;
  int result = pthread_create(&thread_id, NULL, get_device_info_thread, params);
  if (result != 0) {
    fprintf(stderr, "Failed to create thread: %d\n", result);
    return;
  }
  // Detach thread so it runs in the background
  pthread_detach(thread_id);

  printf("Async get_device_info thread started.\n");
}

int modbus_operation_async(const uint8_t *values, int len,
                           ModbusOperationResultCallback callback,
                           void *user_data) {
  if (len <= 0 || !values || !callback) {
    fprintf(stderr, "Invalid parameters for modbus_operation_async.\n");
    return -1;
  }
  printf("modbus_operation_async called, len=%d, data: ", len);
  print_hex((unsigned char *)values, len);

  // TODO: Implement the actual Modbus operation here
  // ...

  return 0;
}