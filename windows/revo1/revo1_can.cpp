// This example demonstrates simple control of Revo1 using a ZQWL USB-CAN device
// USBCANFD-200U
// USBCANFD-100U
// USBCANFD-100U-mini
// You need to download the vendor-provided .dll
// http://39.108.220.80/download/user/ZQWL/UCANFD/
#include "stark-sdk.h"
#include "zqwl-can/zlgcan.h"
#include <stdio.h>
#include <windows.h>

// Global variable declarations
static long device_handle_ = INVALID_DEVICE_HANDLE;
static Handle_chl channel_handle_ = {INVALID_CHANNEL_HANDLE};

// Function declarations
bool init_can_device(int device_type, int channel_index);
bool setup_can_baudrate(int channel_index, int baudrate);
bool start_can_channel(int channel_index);
void setup_can_callbacks();
void get_device_info(DeviceHandler *handle, uint8_t slave_id);
void cleanup_resources();

int main(int argc, char const *argv[]) {
  const int device_type = ZCAN_USBCANFD_100U;
  const int channel_index = 0;
  const int baudrate = 1000000; // 1 Mbps

  // Initialize CAN device
  if (!init_can_device(device_type, channel_index)) {
    return -1;
  }

  // Start CAN channel
  if (!start_can_channel(channel_index)) {
    cleanup_resources();
    return -1;
  }

  // Configure baudrate and timing parameters
  if (!setup_can_baudrate(channel_index, baudrate)) {
    cleanup_resources();
    return -1;
  }

  setup_can_callbacks(); // Set read/write callbacks

  init_logging(LOG_LEVEL_INFO);
  // Revo1 CAN slave ID: 1 = left hand, 2 = right hand
  uint8_t slave_id = 1;
  auto handle = init_device_handler(STARK_PROTOCOL_TYPE_CAN, 0);
  get_device_info(handle, slave_id);

  uint16_t positions_fist[] = {500, 500, 1000, 1000, 1000, 1000}; // Fist
  uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};                 // Open hand
  int delay = 1000;                                               // 1000 ms
  // stark_set_finger_position(handle, slave_id, STARK_FINGER_ID_PINKY, 100);
  // Sleep(delay);
  stark_set_finger_positions(handle, slave_id, positions_fist, 6);
  Sleep(delay);
  stark_set_finger_positions(handle, slave_id, positions_open, 6);
  Sleep(delay);

  // Clean up resources
  cleanup_resources();
  return 0;
}

bool init_can_device(int device_type, int channel_index) {
  device_handle_ = ZCAN_OpenDevice(device_type, channel_index, 0);
  if (device_handle_ == INVALID_DEVICE_HANDLE) {
    printf("Failed to open device\n");
    return false;
  }
  return true;
}

bool setup_can_baudrate(int channel_index, int baudrate) {
  // Set baudrate
  char path[50] = {0};
  char value[10] = {0};
  sprintf_s(path, "%d/baud_rate", channel_index);
  sprintf_s(value, "%d", baudrate);
  ZCAN_SetValue(device_handle_, path, value);

  // Set CAN timing parameters (timing0, timing1)
  // For 1 Mbps, recommended timing0=0x00, timing1=0x14
  char timing_path0[50] = {0};
  char timing_value0[10] = {0};
  char timing_path1[50] = {0};
  char timing_value1[10] = {0};
  sprintf_s(timing_path0, "%d/abit_timing0", channel_index);
  sprintf_s(timing_value0, "0x00");
  sprintf_s(timing_path1, "%d/abit_timing1", channel_index);
  sprintf_s(timing_value1, "0x14");
  ZCAN_SetValue(device_handle_, timing_path0, timing_value0);
  ZCAN_SetValue(device_handle_, timing_path1, timing_value1);

  return true;
}

bool start_can_channel(int channel_index) {
  ZCAN_CHANNEL_INIT_CONFIG config;
  memset(&config, 0, sizeof(config));
  config.can_type = TYPE_CAN;
  config.can.mode = 0; // 0: normal mode, 1: listen-only mode

  channel_handle_ = ZCAN_InitCAN(device_handle_, channel_index, config)[0];
  if (INVALID_CHANNEL_HANDLE == channel_handle_.handle) {
    printf("Failed to initialize CAN channel\n");
    return false;
  }

  if (ZCAN_StartCAN(channel_handle_) != STATUS_OK) {
    printf("Failed to start CAN channel\n");
    return false;
  }

  return true;
}

void setup_can_callbacks() {
  // CAN transmit callback
  set_can_tx_callback([](uint8_t slave_id, uint32_t can_id, const uint8_t *data,
                         uintptr_t data_len) -> int {
    printf("CAN Send: Slave ID: %d, CAN ID: 0x%X, Data Length: %zu\n", slave_id,
           can_id, data_len);
    printf("Data: ");
    for (uintptr_t i = 0; i < data_len; ++i) {
      printf("%02x ", data[i]);
    }
    printf("\n");

    // Build CAN transmit data structure
    ZCAN_Transmit_Data can_data;
    memset(&can_data, 0, sizeof(can_data));

    // Configure CAN ID, standard frame eff=0, extended frame eff=1
    // rtr=0: data frame, rtr=1: remote frame; err=0: normal frame, err=1: error
    // frame
    can_data.frame.can_id = MAKE_CAN_ID(can_id, 0, 0, 0); // Standard frame
    can_data.frame.can_dlc = data_len;
    can_data.transmit_type = 0; // Normal transmit

    // Fill data
    for (uintptr_t i = 0; i < data_len && i < 8; ++i) {
      can_data.frame.data[i] = data[i];
    }

    // Transmit CAN frame
    int result = ZCAN_Transmit(channel_handle_, &can_data, 1);
    return result == 1 ? 0 : -1; // 0 indicates success
  });

  // CAN receive callback
  set_can_rx_callback([](uint8_t slave_id, uint32_t *can_id_out,
                         uint8_t *data_out, uintptr_t *data_len_out) -> int {
    printf("CAN Read: Slave ID: %d\n", slave_id);

    // Read data
    ZCAN_Receive_Data can_data[1000];
    int len = ZCAN_GetReceiveNum(channel_handle_, TYPE_CAN);
    len = ZCAN_Receive(channel_handle_, can_data, 1000, 50);
    if (len < 1) {
      return -1;
    }

    // Concatenate multiple CAN frames; all frames should share the same CAN ID
    *can_id_out = can_data[0].frame.can_id;

    int idx = 0;
    int total_dlc = 0;

    for (int i = 0; i < len; i++) {
      ZCAN_Receive_Data recv_data = can_data[i];
      int can_dlc = recv_data.frame.can_dlc;
      for (int j = 0; j < can_dlc; j++) {
        data_out[idx++] = recv_data.frame.data[j];
      }
      total_dlc += can_dlc;
    }

    *data_len_out = total_dlc;
    return 0; // Return 0 on success
  });
}

void get_device_info(DeviceHandler *handle, uint8_t slave_id) {
  DeviceInfo *info = stark_get_device_info(handle, slave_id);
  if (info != NULL) {
    printf("Slave[%hhu] Serial Number: %s\n", slave_id, info->serial_number);
    free_device_info(info);
  }
}

void cleanup_resources() {
  if (channel_handle_.handle != INVALID_CHANNEL_HANDLE) {
    ZCAN_ResetCAN(channel_handle_);
  }
  if (device_handle_ != INVALID_DEVICE_HANDLE) {
    ZCAN_CloseDevice(device_handle_);
  }
}
