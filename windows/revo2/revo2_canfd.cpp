#include <stdio.h>
#include <windows.h>
#include "stark-sdk.h"
#include "zlgcan/zlgcan.h"

// Global variable declarations
static long device_handle_ = INVALID_DEVICE_HANDLE;
static Handle_chl channel_handle_ = {INVALID_CHANNEL_HANDLE};

// Function declarations
bool init_canfd_device(int device_type, int channel_index);
bool setup_canfd_baudrate(int channel_index, int abit_baud, int dbit_baud);
bool start_canfd_channel(int channel_index);
void setup_canfd_callbacks();
void get_device_info(DeviceHandler *handle, uint8_t slave_id);
void cleanup_resources();

int main(int argc, char const *argv[])
{
  const int device_type = ZCAN_USBCANFD_100U;
  const int channel_index = 0;
  const int abit_baudrate = 1000000; // Arbitration domain baudrate 1 Mbps
  const int dbit_baudrate = 5000000; // Data domain baudrate 5 Mbps

  // Initialize CANFD device
  if (!init_canfd_device(device_type, channel_index))
  {
    return -1;
  }

  // Start CANFD channel
  if (!start_canfd_channel(channel_index))
  {
    cleanup_resources();
    return -1;
  }

  // Configure CANFD baudrate parameters
  if (!setup_canfd_baudrate(channel_index, abit_baudrate, dbit_baudrate))
  {
    cleanup_resources();
    return -1;
  }

  setup_canfd_callbacks(); // Set read/write callbacks

  // Open device and get information
  const uint8_t MASTER_ID = 1;
  init_cfg(STARK_PROTOCOL_TYPE_CAN_FD, LOG_LEVEL_DEBUG);
  auto handle = canfd_init(MASTER_ID);

  // uint8_t slave_id = 0x7e; // 0x7e for left hand
  uint8_t slave_id = 0x7f;   // 0x7f for right hand
  get_device_info(handle, slave_id);

  int delay = 1000; // 1000 ms
  uint16_t positions[6] = {500, 500, 500, 500, 500, 500};
  uint16_t durations[6] = {300, 300, 300, 300, 300, 300};
  stark_set_finger_positions_and_durations(handle, slave_id, positions, durations, 6);
  Sleep(delay); // Wait for fingers to reach target positions

  // Clean up resources
  cleanup_resources();
  return 0;
}

bool init_canfd_device(int device_type, int channel_index)
{
  device_handle_ = ZCAN_OpenDevice(device_type, channel_index, 0);
  if (device_handle_ == INVALID_DEVICE_HANDLE)
  {
    printf("Failed to open CANFD device\n");
    return false;
  }
  return true;
}

bool setup_canfd_baudrate(int channel_index, int abit_baud, int dbit_baud)
{
  char abit_path[50] = {0};
  char abit_value[20] = {0};
  char dbit_path[50] = {0};
  char dbit_value[20] = {0};
  char filter_path[50] = {0};

  // Set arbitration domain baudrate
  sprintf_s(abit_path, "%d/canfd_abit_baud_rate", channel_index);
  sprintf_s(abit_value, "%d", abit_baud);
  ZCAN_SetValue(device_handle_, abit_path, abit_value);

  // Set data domain baudrate
  sprintf_s(dbit_path, "%d/canfd_dbit_baud_rate", channel_index);
  sprintf_s(dbit_value, "%d", dbit_baud);
  ZCAN_SetValue(device_handle_, dbit_path, dbit_value);

  // Clear filter configuration
  sprintf_s(filter_path, "%d/filter_clear", channel_index);
  char filter_value[] = "0";
  ZCAN_SetValue(device_handle_, filter_path, filter_value);

  return true;
}

bool start_canfd_channel(int channel_index)
{
  ZCAN_CHANNEL_INIT_CONFIG config;
  memset(&config, 0, sizeof(config));
  config.can_type = TYPE_CANFD; // Use CANFD type
  config.canfd.mode = 0;        // 0: normal mode, 1: listen-only mode

  channel_handle_ = ZCAN_InitCAN(device_handle_, channel_index, config)[0];
  if (INVALID_CHANNEL_HANDLE == channel_handle_.handle)
  {
    printf("Failed to initialize CANFD channel\n");
    return false;
  }

  if (ZCAN_StartCAN(channel_handle_) != STATUS_OK)
  {
    printf("Failed to start CANFD channel\n");
    return false;
  }

  return true;
}

void setup_canfd_callbacks()
{
  // CANFD transmit callback
  set_can_tx_callback([](uint8_t slave_id,
                         uint32_t can_id,
                         const uint8_t *data,
                         uintptr_t data_len) -> int
                      {
                        printf("CANFD Send: Slave ID: %d, CAN ID: 0x%X, Data Length: %zu\n", slave_id, can_id, data_len);
                        printf("Data: ");
                        for (uintptr_t i = 0; i < data_len; ++i)
                        {
                          printf("%02x ", data[i]);
                        }
                        printf("\n");

                        // Build CANFD transmit data structure
                        ZCAN_TransmitFD_Data canfd_data;
                        memset(&canfd_data, 0, sizeof(canfd_data));

                        // Configure CAN ID: extended frame eff=1, data frame rtr=0, normal frame err=0
                        canfd_data.frame.can_id = MAKE_CAN_ID(can_id, 1, 0, 0); // Extended frame
                        canfd_data.frame.len = data_len;
                        canfd_data.transmit_type = 0;        // Normal transmit
                        canfd_data.frame.flags |= CANFD_BRS; // Use BRS (bit rate switch)

                        // Fill data, maximum length 64
                        for (uintptr_t i = 0; i < data_len && i < 64; ++i)
                        {
                          canfd_data.frame.data[i] = data[i];
                        }

                        // Transmit CANFD frame
                        int result = ZCAN_TransmitFD(channel_handle_, &canfd_data, 1);
                        return result == 1 ? 0 : -1; // 0 indicates success
                      });

  // CANFD receive callback
  set_can_rx_callback([](uint8_t slave_id,
                         uint32_t *can_id_out,
                         uint8_t *data_out,
                         uintptr_t *data_len_out) -> int
                      {
                        printf("CANFD Read: Slave ID: %d\n", slave_id);
                        // Sleep(1000); // Wait for data ready

                        // Read CANFD data
                        ZCAN_ReceiveFD_Data canfd_data[1000];
                        int len = ZCAN_GetReceiveNum(channel_handle_, TYPE_CANFD);
                        printf("ZCAN_GetReceiveNum, len: %d\n", len);
                        len = ZCAN_ReceiveFD(channel_handle_, canfd_data, 1000, 50);
                        printf("ZCAN_ReceiveFD, len: %d\n", len);
                        if (len < 1)
                        {
                          return -1;
                        }

                        // Process the first frame
                        ZCAN_ReceiveFD_Data recv_data = canfd_data[0];
                        canfd_frame frame = recv_data.frame;

                        *can_id_out = frame.can_id;
                        *data_len_out = frame.len; // CANFD uses `len` instead of `can_dlc`

                        // Fill data, maximum length 64
                        for (int i = 0; i < frame.len && i < 64; i++)
                        {
                          data_out[i] = frame.data[i];
                        }

                        return 0; // Return 0 on success
                      });
}

void get_device_info(DeviceHandler *handle, uint8_t slave_id)
{
  DeviceInfo *info = stark_get_device_info(handle, slave_id);
  if (info != NULL)
  {
    printf("Slave[%hhu] Serial Number: %s, FW: %s\n", slave_id, info->serial_number, info->firmware_version);
    if (info->hardware_type != STARK_HARDWARE_TYPE_REVO2_BASIC && info->hardware_type != STARK_HARDWARE_TYPE_REVO2_TOUCH)
    {
      printf("Not Revo2, hardware type: %hhu\n", info->hardware_type);
      exit(1);
    }
    free_device_info(info);
  } else {
    printf("Error: Failed to get device info\n");
    exit(1);
  }
}

void cleanup_resources()
{
  if (channel_handle_.handle != INVALID_CHANNEL_HANDLE)
  {
    ZCAN_ResetCAN(channel_handle_);
  }
  if (device_handle_ != INVALID_DEVICE_HANDLE)
  {
    ZCAN_CloseDevice(device_handle_);
  }
}
