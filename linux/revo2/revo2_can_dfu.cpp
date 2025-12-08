// This example demonstrates firmware upgrade of Revo2 with a ZLG USB-CAN FD
// device. USBCANFD-200U USBCANFD-100U USBCANFD-100U-mini You need to download
// the vendor-provided .so https://manual.zlg.cn/web/#/146
#include "can_common.h"
#include "dfu_common.h"
#include "stark-sdk.h"
#include "stark_common.h"
#include <cstring>
#include <execinfo.h>
#include <signal.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

// ================== Global variables ==================
static int g_start_time = 0;

// ================== Function declarations ==================
void cleanup_resources();
void get_device_info(DeviceHandler *handle, uint8_t slave_id);

// ================== Main function ==================

int main(int argc, char const *argv[]) {
  // Setup signal handlers for crash debugging
  setup_signal_handlers();

  // Setup CAN (device + channel + callbacks)
  if (!setup_can()) {
    return -1;
  }

  // Initialize STARK SDK
  init_cfg(STARK_PROTOCOL_TYPE_CAN, LOG_LEVEL_INFO);
  auto handle = create_device_handler();
  uint8_t slave_id = 1; // Default left-hand ID for Revo2 is 1
  // uint8_t slave_id = 2; // Default right-hand ID for Revo2 is 2
  get_device_info(handle, slave_id);

  // Set up DFU callbacks with timing and cleanup
  g_start_time = get_current_time_ms();
  setup_dfu_callbacks_with_timing(g_start_time, cleanup_resources);

  // Start DFU upgrade
  start_dfu(handle, slave_id, "ota_bin/stark2/Revo2_V1.0.4_2508291545.bin", 5);

  printf("Revo2 CAN DFU, Waiting for DFU to complete...\n");
  useconds_t delay = 300 * 1000 * 1000; // 300 s, wait for DFU to complete
  usleep(delay);

  // Clean up resources
  cleanup_resources();
  return 0;
}

void cleanup_resources() {
  cleanup_can_resources(); // Use common cleanup function
  printf("Resources cleaned up.\n");
}

void get_device_info(DeviceHandler *handle, uint8_t slave_id) {
  DeviceInfo *info = stark_get_device_info(handle, slave_id);
  if (info != NULL) {
    printf("Slave[%hhu] Serial Number: %s, FW: %s\n", slave_id,
           info->serial_number, info->firmware_version);
    if (info->hardware_type != STARK_HARDWARE_TYPE_REVO2_BASIC &&
        info->hardware_type != STARK_HARDWARE_TYPE_REVO2_TOUCH) {
      printf("Not Revo2, hardware type: %hhu\n", info->hardware_type);
      exit(1);
    }
    free_device_info(info);
  } else {
    printf("Error: Failed to get device info\n");
    exit(1);
  }
}