/**
 * @file stark_common.c
 * @brief Implementation of common utility functions for Stark SDK C examples
 */

#include "stark_common.h"
#include <execinfo.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

// Suppress clang-tidy warnings for this file
// #pragma clang diagnostic push
// #pragma clang diagnostic ignored "-Wunknown-pragmas"
// #pragma clang diagnostic ignored "-Wunused-parameter"

/****************************************************************************/
// Signal handling
/****************************************************************************/

static void signal_handler(int sig) {
  void *array[10];
  size_t size;

  // Get stack frames
  size = backtrace(array, 10);

  // Print all stack frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

void setup_signal_handlers(void) {
  signal(SIGSEGV,
         signal_handler); // Install handler for SIGSEGV (segmentation fault)
  signal(SIGABRT, signal_handler); // Install handler for SIGABRT (abort signal)
}

/****************************************************************************/
// Utility functions
/****************************************************************************/

void print_hex(const unsigned char *data, int len) {
  for (int i = 0; i < len; i++) {
    printf("%02X", data[i]);
    if (i < len - 1) {
      printf(" ");
    }
  }
  printf("\n");
}

/****************************************************************************/
// Device information functions
/****************************************************************************/

bool get_and_print_device_info(DeviceHandler *handle, uint8_t slave_id) {
  CDeviceInfo *info = stark_get_device_info(handle, slave_id);
  if (info != NULL) {
    printf("Slave[%hhu] Serial Number: %s, FW: %s\n", slave_id,
           info->serial_number, info->firmware_version);

    // Print hardware type
    const char *hw_type = "Unknown";
    switch (info->hardware_type) {
    case STARK_HARDWARE_TYPE_REVO1_BASIC:
      hw_type = "Revo1 Basic";
      break;
    case STARK_HARDWARE_TYPE_REVO1_TOUCH:
      hw_type = "Revo1 Touch";
      break;
    case STARK_HARDWARE_TYPE_REVO2_BASIC:
      hw_type = "Revo2 Basic";
      break;
    case STARK_HARDWARE_TYPE_REVO2_TOUCH:
      hw_type = "Revo2 Touch";
      break;
    case STARK_HARDWARE_TYPE_REVO1_ADVANCED:
      hw_type = "Revo1 Advanced";
      break;
    default:
      break;
    }
    printf("Hardware Type: %s (%hhu)\n", hw_type, info->hardware_type);

    free_device_info(info);
    return true;
  } else {
    printf("Error: Failed to get device info\n");
    return false;
  }
}

void get_and_print_extended_info(DeviceHandler *handle, uint8_t slave_id) {
  // Get baudrate
  auto baudrate = stark_get_rs485_baudrate(handle, slave_id);
  printf("Slave[%hhu] Baudrate: %d\n", slave_id, baudrate);

  // Get voltage
  auto voltage = stark_get_voltage(handle, slave_id);
  printf("Slave[%hhu] Voltage: %.2fV\n", slave_id, voltage / 1000.0);

  // Get LED info
  auto led_info = stark_get_led_info(handle, slave_id);
  if (led_info != NULL) {
    printf("Slave[%hhu] LED: color=%hhu, mode=%hhu\n", slave_id,
           led_info->color, led_info->mode);
    free_led_info(led_info);
  }
}

bool verify_device_is_revo1(DeviceHandler *handle, uint8_t slave_id) {
  CDeviceInfo *info = stark_get_device_info(handle, slave_id);
  if (info == NULL) {
    printf("Error: Failed to get device info\n");
    return false;
  }

  // Print device info
  printf("Slave[%hhu] Serial Number: %s, FW: %s\n", slave_id,
         info->serial_number, info->firmware_version);

  // Print hardware type and check if it's Revo1
  const char *hw_type = "Unknown";
  bool is_revo1 = false;
  switch (info->hardware_type) {
  case STARK_HARDWARE_TYPE_REVO1_BASIC:
    hw_type = "Revo1 Basic";
    is_revo1 = true;
    break;
  case STARK_HARDWARE_TYPE_REVO1_TOUCH:
    hw_type = "Revo1 Touch";
    is_revo1 = true;
    break;
  case STARK_HARDWARE_TYPE_REVO1_ADVANCED:
    hw_type = "Revo1 Advanced";
    is_revo1 = true;
    break;
  case STARK_HARDWARE_TYPE_REVO1_ADVANCED_TOUCH:
    hw_type = "Revo1 Advanced Touch";
    is_revo1 = true;
    break;
  case STARK_HARDWARE_TYPE_REVO2_BASIC:
    hw_type = "Revo2 Basic";
    break;
  case STARK_HARDWARE_TYPE_REVO2_TOUCH:
    hw_type = "Revo2 Touch";
    break;
  default:
    break;
  }
  printf("Hardware Type: %s (%hhu)\n", hw_type, info->hardware_type);

  if (!is_revo1) {
    printf("Error: Device is not Revo1\n");
  }

  free_device_info(info);
  return is_revo1;
}

bool verify_device_is_revo2(DeviceHandler *handle, uint8_t slave_id) {
  CDeviceInfo *info = stark_get_device_info(handle, slave_id);
  if (info == NULL) {
    printf("Error: Failed to get device info\n");
    return false;
  }

  // Print device info
  printf("Slave[%hhu] Serial Number: %s, FW: %s\n", slave_id,
         info->serial_number, info->firmware_version);

  // Print hardware type and check if it's Revo2
  const char *hw_type = "Unknown";
  bool is_revo2 = false;
  switch (info->hardware_type) {
  case STARK_HARDWARE_TYPE_REVO1_BASIC:
    hw_type = "Revo1 Basic";
    break;
  case STARK_HARDWARE_TYPE_REVO1_TOUCH:
    hw_type = "Revo1 Touch";
    break;
  case STARK_HARDWARE_TYPE_REVO2_BASIC:
    hw_type = "Revo2 Basic";
    is_revo2 = true;
    break;
  case STARK_HARDWARE_TYPE_REVO2_TOUCH:
    hw_type = "Revo2 Touch";
    is_revo2 = true;
    break;
  case STARK_HARDWARE_TYPE_REVO1_ADVANCED:
    hw_type = "Revo1 Advanced";
    is_revo2 = true;
    break;
  case STARK_HARDWARE_TYPE_REVO1_ADVANCED_TOUCH:
    hw_type = "Revo1 Advanced Touch";
    is_revo2 = true;
    break;
  default:
    printf("Warning: Unknown hardware type\n");
    break;
  }
  printf("Hardware Type: %s (%hhu)\n", hw_type, info->hardware_type);

  if (!is_revo2) {
    printf("Error: Device is not Revo2\n");
  }

  free_device_info(info);
  return is_revo2;
}

/****************************************************************************/
// Time utilities
/****************************************************************************/

int get_current_time_ms(void) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (int)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}
