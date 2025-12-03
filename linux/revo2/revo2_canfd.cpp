// This example demonstrates basic control of Revo2 with a ZLG USB-CAN FD device.
// USBCANFD-200U
// USBCANFD-100U
// USBCANFD-100U-mini
// You need to download the vendor-provided .so
// https://manual.zlg.cn/web/#/146
#include <stdio.h>
#include <cstring>
#include <unistd.h>
#include "stark-sdk.h"
#include "zlgcan/zcan.h"

// Constant definitions
#define ZCANFD_TYPE_USBCANFD 33 // Device type
#define ZCANFD_CARD_INDEX 0     // Card index
#define ZCANFD_CHANNEL_INDEX 0  // Channel index
#define MAX_CHANNELS 2          // Maximum number of channels
#define RX_WAIT_TIME 100        // Receive wait time
#define RX_BUFF_SIZE 1000       // Receive buffer size

// Function declarations
bool init_canfd_device();
bool start_canfd_channel();
void cleanup_resources();
void setup_canfd_callbacks();
void get_device_info(DeviceHandler *handle, uint8_t slave_id);

int main(int argc, char const *argv[])
{
  // Initialize CANFD device
  if (!init_canfd_device())
  {
    return -1;
  }

  // Start CANFD channel
  if (!start_canfd_channel())
  {
    cleanup_resources();
    return -1;
  }

  setup_canfd_callbacks(); // Set read/write callbacks

  init_cfg(STARK_PROTOCOL_TYPE_CAN_FD, LOG_LEVEL_DEBUG);
  const uint8_t MASTER_ID = 1; // Master device ID
  auto handle = canfd_init(MASTER_ID);
  // uint8_t slave_id = 0x7e;       // Default left-hand ID for Revo2 is 0x7e
  uint8_t slave_id = 0x7f; // Default right-hand ID for Revo2 is 0x7f
  // uint8_t slave_id_right = 0x7f; // Default right-hand ID for Revo2 is 0x7f
  // get_device_info(handle, slave_id); return 0;
  get_device_info(handle, slave_id); return 0;

  // Interface to modify hand baudrate; can also be modified via upper-computer tools
  // stark_set_canfd_baudrate(handle, slave_id_right, 2000); // Set to 2 Mbps
  // stark_set_canfd_baudrate(handle, slave_id_right, 5000); // Set to 5 Mbps
  // return 0;

  while (1)
  {
    uint16_t durations[6] = {300, 300, 300, 300, 300, 300};
    // uint16_t speeds[6] = {500, 500, 500, 500, 500, 500};
    uint16_t positions_fist_1000[] = {300, 300, 1000, 1000, 1000, 1000}; // Fist
    uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};                      // Open hand
    useconds_t delay = 1000 * 1000;                                      // 1000 ms
    stark_set_finger_positions_and_durations(handle, slave_id, positions_fist_1000, durations, 6);
    // stark_set_finger_positions_and_durations(handle, slave_id_right, positions_fist_1000, durations, 6);
    usleep(delay);
    stark_set_finger_positions_and_durations(handle, slave_id, positions_open, durations, 6);
    // stark_set_finger_positions_and_durations(handle, slave_id_right, positions_open, durations, 6);
    usleep(delay);

    // Position + target speed mode
    // uint16_t speeds[6] = {500, 500, 500, 500, 500, 500};
    // stark_set_finger_positions_and_speeds(handle, slave_id, positions_fist, speeds, 6);
    // stark_set_finger_positions_and_speeds(handle, slave_id_right, positions_fist, speeds, 6);
    // usleep(delay);

    // Position mode - percentage
    // uint16_t positions_fist[] = {30, 30, 100, 100, 100, 100}; // Fist
    // stark_set_finger_positions(handle, slave_id, positions_fist, 6);
    // stark_set_finger_positions(handle, slave_id_right, positions_fist, 6);
    // usleep(delay);
    // stark_set_finger_positions(handle, slave_id, positions_open, 6);
    // stark_set_finger_positions(handle, slave_id_right, positions_open, 6);
    // usleep(delay);
  }

  // Clean up resources
  cleanup_resources();
  return 0;
}

bool init_canfd_device()
{
  // Open device
  if (!VCI_OpenDevice(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX))
  {
    printf("Failed to open device\n");
    return false;
  }
  return true;
}

bool start_canfd_channel()
{
  ZCAN_INIT init;      // Baud rate configuration, values are based on zcanpro baud-rate calculator
  init.mode = 0;       // 0 - normal mode
  init.clk = 60000000; // clock: 60M (V1.01) or 80M (V1.03 and above)

  // init.aset.brp = 5;
  // init.aset.tseg1 = 6;
  // init.aset.tseg2 = 1;

  // Arbitration domain 1 Mbps, 75% sampling
  init.aset.sjw = 1;
  init.aset.brp = 4;
  init.aset.tseg1 = 7;
  init.aset.tseg2 = 2;
  init.aset.smp = 0;

  // Data domain 5 Mbps, 75% sampling (default)
  init.dset.sjw = 1;
  init.dset.brp = 0;
  init.dset.tseg1 = 7;
  init.dset.tseg2 = 2;
  init.dset.smp = 0;
  printf("Data domain 5 Mbps\n");

  // Data domain 2 Mbps, 73.33% sampling
  // init.dset.sjw = 1;
  // init.dset.brp = 1;
  // init.dset.tseg1 = 9;
  // init.dset.tseg2 = 3;
  // init.dset.smp = 0;
  // printf("Data domain 2 Mbps\n");

  // Data domain 1 Mbps, 75% sampling
  // init.dset.sjw = 1;
  // init.dset.brp = 4;
  // init.dset.tseg1 = 7;
  // init.dset.tseg2 = 2;
  // init.dset.smp = 0;

  // Initialize CANFD channel
  if (!VCI_InitCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, &init))
  {
    printf("Failed to initialize CANFD channel\n");
    return false;
  }

  // Enable/disable termination resistor
  // U32 on = 0; // on=0 disable, on=1 enable
  // if (!VCI_SetReference(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, CMD_CAN_TRES, &on)) // termination resistor
  // {
  //   printf("CMD_CAN_TRES fail\n");
  // } else {
  //   printf("CMD_CAN_TRES success\n");
  // }

  // Start CANFD channel
  if (!VCI_StartCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX))
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
                         uint32_t canfd_id,
                         const uint8_t *data,
                         uintptr_t data_len) -> int
                      {
                        printf("CANFD Send: Slave ID: %d, CANFD ID: 0x%X, Data Length: %zu\n", slave_id, canfd_id, data_len);
                        printf("Data: ");
                        for (uintptr_t i = 0; i < data_len; ++i)
                        {
                          printf("%02x ", data[i]);
                        }
                        printf("\n");

                        // Construct CANFD transmit message
                        ZCAN_FD_MSG canfd_msg;
                        memset(&canfd_msg, 0, sizeof(ZCAN_FD_MSG));

                        canfd_msg.hdr.inf.txm = 0; // 0 - normal transmit
                        canfd_msg.hdr.inf.fmt = 1; // 0 - CAN, 1 - CANFD
                        canfd_msg.hdr.inf.sdf = 0; // 0 - data frame, CANFD only supports data frames
                        canfd_msg.hdr.inf.sef = 1; // 0 - standard frame, 1 - extended frame
                        canfd_msg.hdr.inf.brs = 1; // CANFD only, 1 - enable data phase bit-rate switching
                        // canfd_msg.hdr.inf.echo = 1;    // transmit echo

                        canfd_msg.hdr.id = canfd_id;              // ID
                        canfd_msg.hdr.chn = ZCANFD_CHANNEL_INDEX; // Channel
                        canfd_msg.hdr.len = data_len;             // Data length
                        // canfd_msg.hdr.len = 64;                // Maximum data length 64

                        // Fill data, maximum length 64
                        for (uintptr_t i = 0; i < data_len && i < 64; ++i)
                        {
                          canfd_msg.dat[i] = data[i];
                        }

                        // Transmit CANFD frame
                        printf("Transmitting CANFD frame...\n");
                        int result = VCI_TransmitFD(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, &canfd_msg, 1);
                        printf("Transmit result: %d\n", result);
                        return result == 1 ? 0 : -1; // 0 indicates success
                      });

  // CANFD receive callback
  set_can_rx_callback([](uint8_t slave_id,
                         uint32_t *canfd_id_out,
                         uint8_t *data_out,
                         uintptr_t *data_len_out) -> int
                      {
                        printf("CANFD Read: Slave ID: %d\n", slave_id);

                        // Read data
                        ZCAN_FD_MSG canfd_data[RX_BUFF_SIZE];
                        int len = VCI_ReceiveFD(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, canfd_data, RX_BUFF_SIZE, RX_WAIT_TIME); // CANFD
                        if (len < 1)
                        {
                          printf("VCI_ReceiveFD, len: %d\n", len);
                          return -1;
                        }

                        // Handle the first frame
                        ZCAN_FD_MSG recv_data = canfd_data[0];
                        int canfd_dlc = recv_data.hdr.len;
                        // printf("canfd_dlc: %d\n", canfd_dlc);
                        *canfd_id_out = recv_data.hdr.id;
                        *data_len_out = canfd_dlc;

                        for (int j = 0; j < canfd_dlc && j < 64; j++)
                        {
                          data_out[j] = recv_data.dat[j];
                        }
                        return 0; // Return 0 on success
                      });
}

void cleanup_resources()
{
  VCI_ResetCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX); // Reset CANFD channel
  VCI_CloseDevice(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX);
  printf("Resources cleaned up.\n");
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
