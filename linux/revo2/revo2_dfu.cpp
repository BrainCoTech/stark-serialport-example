#include "dfu_common.h"
#include "stark-sdk.h"
#include "stark_common.h"
#include <stdio.h>

int main(int argc, char const *argv[]) {
  // Setup signal handlers for crash debugging
  setup_signal_handlers();

  init_cfg(STARK_PROTOCOL_TYPE_MODBUS, LOG_LEVEL_INFO); // Initialize configuration
  auto cfg = auto_detect_modbus_revo2(NULL, true); // Replace with actual serial port name; passing NULL will try auto-detection
  if (cfg == NULL) {
    fprintf(stderr, "Failed to auto-detect Modbus device configuration.\n");
    return -1;
  }

  auto handle = modbus_open(cfg->port_name, cfg->baudrate);
  uint8_t slave_id = cfg->slave_id;
  if (cfg != NULL)
    free_device_config(cfg);

  // Setup standard DFU callbacks
  setup_dfu_callbacks_simple();

  start_dfu(handle, slave_id,
            "ota_bin/stark2/Revo2_V1.0.20.U_2601091030.bin", 5);

  printf("Revo2 Modbus DFU, Waiting for DFU to complete...\n");
  wait_for_dfu_completion(60);
}
