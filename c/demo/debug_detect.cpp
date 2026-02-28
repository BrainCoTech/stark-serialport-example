/**
 * @file debug_detect.cpp
 * @brief Simplified Auto-Detect for Debugging
 *
 * Scans a specific port and baudrate for IDs 126 and 127.
 *
 * Build: make debug_detect.exe
 * Run:   ./debug_detect.exe /dev/ttyUSB1
 */

#include "stark-sdk.h"
#include "stark_common.h"
#include <stdio.h>
#include <string.h>

int main(int argc, char const *argv[]) {
  printf("=== Stark Debug Detect Example ===\n\n");

  init_logging(LOG_LEVEL_INFO);

  // Default configuration from user request
//   const char *port = "/dev/ttyUSB1";
  const char *port = "/dev/tty.usbserial-D30JXZTY";
  uint32_t baudrate = 460800;
  uint8_t scan_ids[] = {126, 127};
  int id_count = 2;

  // Use command line port if provided
  if (argc > 1) {
    port = argv[1];
  }

  printf("[INFO] Scanning port: %s\n", port);
  printf("[INFO] Fixed Baudrate: %u\n", baudrate);
  printf("[INFO] Target IDs: 126, 127\n\n");

  // Open Modbus port once for all scans
  DeviceHandler *handle = modbus_open(port, baudrate);
  if (handle == NULL) {
    printf("[ERROR] Failed to open port %s at %u\n", port, baudrate);
    printf("[TIP] Check if the port exists and you have permissions.\n");
    return -1;
  }

  bool found_any = false;
  for (int i = 0; i < id_count; i++) {
    uint8_t slave_id = scan_ids[i];
    printf("[SCAN] Trying slave ID: %d (0x%02X)...\n", slave_id, slave_id);

    // Fast ping using input register 3000 (GetFwVersion)
    // Fast ping using holding register 901 (GetHandType)
    uint16_t dummy_val = 0;
    if (stark_read_input_registers(handle, slave_id, 3000, 1, &dummy_val) ==
            0 ||
        stark_read_holding_registers(handle, slave_id, 901, 1, &dummy_val) ==
            0) {
      printf("\n  ✅ Ping SUCCESS: Device responded at ID %d!\n", slave_id);

      // Now get full info
      CDeviceInfo *info = stark_get_device_info(handle, slave_id);
      if (info != NULL) {
        printf("  ----------------------------------\n");
        printf("  Serial Number: %s\n",
               info->serial_number ? info->serial_number : "N/A");
        printf("  Firmware:      %s\n",
               info->firmware_version ? info->firmware_version : "N/A");
        printf("  Hardware Type: %d (%s)\n", info->hardware_type,
               get_hardware_type_name_str(info->hardware_type));
        printf("  ----------------------------------\n\n");

        free_device_info(info);
        found_any = true;
      } else {
        printf("  [WARN] Device responded to ping but failed to return full "
               "info.\n");
      }
    } else {
      printf("  ❌ No response (timeout) from ID %d\n", slave_id);
    }
  }

  if (!found_any) {
    printf("\n[RESULT] No devices detected for IDs 126 or 127.\n");
  } else {
    printf("\n[RESULT] Scan completed.\n");
  }

  // Cleanup
  modbus_close(handle);
  //   close_device_handler(handle, STARK_PROTOCOL_TYPE_MODBUS);

  printf("\n=== Debug scan finished ===\n");
  return 0;
}
