#include "dfu_common.h"
#include "stark-sdk.h"
#include "stark_common.h"
#include <stdio.h>

int main(int argc, char const *argv[]) {
  // Setup signal handlers for crash debugging
  setup_signal_handlers();

  auto cfg = auto_detect_modbus_revo1(NULL, true);
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

  start_dfu(
      handle, slave_id,
      "ota_bin/modbus/FW_MotorController_Release_SecureOTA_modbus_0.1.7.ota",
      5);

  printf("Revo1 Modbus DFU, Waiting for DFU to complete...\n");
  wait_for_dfu_completion(60);
}
