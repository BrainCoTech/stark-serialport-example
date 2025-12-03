// This example demonstrates basic control of Revo2 with a ZLG USB-CAN FD device.
// USBCANFD-200U
// USBCANFD-100U
// USBCANFD-100U-mini
// You need to download the vendor-provided .so
// https://manual.zlg.cn/web/#/146
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <execinfo.h>
#include <cstring>
#include <sys/time.h>
#include "zlgcan/zcan.h"
#include "stark-sdk.h"

// ================== Constant definitions ==================
#define ZCANFD_TYPE_USBCANFD 33 // Device type
#define ZCANFD_CARD_INDEX 0     // Card index
#define ZCANFD_CHANNEL_INDEX 0  // Channel index
#define MAX_CHANNELS 2          // Maximum number of channels
#define RX_WAIT_TIME 100        // Receive wait time
#define RX_BUFF_SIZE 1000       // Receive buffer size

// ================== Function declarations ==================
void handler(int sig);
bool init_can_device();
bool start_can_channel();
void cleanup_resources();
void setup_can_callbacks();
void get_device_info(DeviceHandler *handle, uint8_t slave_id);

// ================== Main function ==================

int main(int argc, char const *argv[])
{
  // Set up signal handlers
  signal(SIGSEGV, handler);
  signal(SIGABRT, handler);

  // Initialize CAN device
  if (!init_can_device())
  {
    return -1;
  }

  // Start CAN channel
  if (!start_can_channel())
  {
    cleanup_resources();
    return -1;
  }

  setup_can_callbacks();

  // Initialize STARK SDK
  init_cfg(STARK_PROTOCOL_TYPE_CAN, LOG_LEVEL_DEBUG);
  auto handle = create_device_handler();
  // uint8_t slave_id = 1; // Default left-hand ID for Revo2 is 1
  uint8_t slave_id = 2; // Default right-hand ID for Revo2 is 2
  get_device_info(handle, slave_id);

  auto finger_id = STARK_FINGER_ID_MIDDLE;
  bool running = true;
  while (running)
  {
    uint16_t durations[6] = {300, 300, 300, 300, 300, 300};
    uint16_t positions_fist_1000[] = {500, 500, 1000, 1000, 1000, 1000}; // Fist
    uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};                      // Open hand
    useconds_t delay = 1000 * 1000;                                      // 1000 ms
    stark_set_finger_positions_and_durations(handle, slave_id, positions_fist_1000, durations, 6);
    usleep(delay);
    stark_set_finger_positions_and_durations(handle, slave_id, positions_open, durations, 6);
    usleep(delay);
    stark_set_finger_position_with_millis(handle, slave_id, finger_id, 1000, 300);
    usleep(delay);
    stark_set_finger_position_with_millis(handle, slave_id, finger_id, 0, 300);
    usleep(delay);
    // running = false;

    auto finger_status = stark_get_motor_status(handle, slave_id);
    if (finger_status != NULL)
    {
      printf("Positions: %hu, %hu, %hu, %hu, %hu, %hu\n", finger_status->positions[0], finger_status->positions[1], finger_status->positions[2], finger_status->positions[3], finger_status->positions[4], finger_status->positions[5]);
      printf("Speeds: %hd, %hd, %hd, %hd, %hd, %hd\n", finger_status->speeds[0], finger_status->speeds[1], finger_status->speeds[2], finger_status->speeds[3], finger_status->speeds[4], finger_status->speeds[5]);
      printf("Currents: %hd, %hd, %hd, %hd, %hd, %hd\n", finger_status->currents[0], finger_status->currents[1], finger_status->currents[2], finger_status->currents[3], finger_status->currents[4], finger_status->currents[5]);
      printf("States: %hhu, %hhu, %hhu, %hhu, %hhu, %hhu\n", finger_status->states[0], finger_status->states[1], finger_status->states[2], finger_status->states[3], finger_status->states[4], finger_status->states[5]);
      free_motor_status_data(finger_status);
    }
  }

  // Clean up resources
  cleanup_resources();
  return 0;
}

// ================== CANFD device management ==================

bool init_can_device()
{
  // Open device
  if (!VCI_OpenDevice(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX))
  {
    printf("Failed to open device\n");
    return false;
  }
  return true;
}

bool start_can_channel()
{
  ZCAN_INIT init;      // Baud rate configuration, values are based on zcanpro baud-rate calculator
  init.mode = 0;       // 0 - normal mode
  init.clk = 60000000; // clock: 60M (V1.01) or 80M (V1.03 and above)

  // Arbitration domain 1 Mbps
  init.aset.sjw = 1;
  init.aset.brp = 5;
  init.aset.tseg1 = 6;
  init.aset.tseg2 = 1;
  init.aset.smp = 0;

  // Data domain 5 Mbps
  init.dset.sjw = 1;
  init.dset.brp = 0;
  init.dset.tseg1 = 7;
  init.dset.tseg2 = 2;
  init.dset.smp = 0;

  // Initialize CAN channel
  if (!VCI_InitCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, &init))
  {
    printf("Failed to initialize CANFD channel\n");
    return false;
  }

  // Start CAN channel
  if (!VCI_StartCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX))
  {
    printf("Failed to start CANFD channel\n");
    return false;
  }

  return true;
}

// ================== CAN callback setup ==================

void setup_can_callbacks()
{
  // CAN transmit callback
  set_can_tx_callback([](uint8_t slave_id,
                         uint32_t can_id,
                         const uint8_t *data,
                         uintptr_t data_len) -> int
                      {
                        // printf("CAN Send: Slave ID: %d, CAN ID: 0x%X, Data Length: %zu\n", slave_id, can_id, data_len);
                        // printf("Data: ");
                        // for (uintptr_t i = 0; i < data_len; ++i)
                        // {
                        //   printf("%02x ", data[i]);
                        // }
                        // printf("\n");

                        // Construct CAN transmit message
                        ZCAN_20_MSG can_msg;
                        memset(&can_msg, 0, sizeof(ZCAN_20_MSG));

                        can_msg.hdr.inf.txm = 0; // 0 - normal transmit
                        can_msg.hdr.inf.fmt = 0; // 0 - CAN
                        can_msg.hdr.inf.sdf = 0; // 0 - data frame, 1 - remote frame
                        can_msg.hdr.inf.sef = 0; // 0 - standard frame, 1 - extended frame

                        can_msg.hdr.id = can_id;                // ID
                        can_msg.hdr.chn = ZCANFD_CHANNEL_INDEX; // Channel
                        can_msg.hdr.len = data_len;             // Data length

                        // Fill data
                        for (uintptr_t i = 0; i < data_len && i < 8; ++i)
                        {
                          can_msg.dat[i] = data[i];
                        }

                        // Transmit CAN frame
                        int result = VCI_Transmit(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, &can_msg, 1);
                        if (result != 1)
                          printf("Transmit result: %d\n", result);
                        return result == 1 ? 0 : -1; // 0 indicates success
                      });

  // CANFD receive callback
  set_can_rx_callback([](uint8_t slave_id,
                         uint32_t *can_id_out,
                         uint8_t *data_out,
                         uintptr_t *data_len_out) -> int
                      {
                        // printf("CAN Read: Slave ID: %d\n", slave_id);

                        // Read data
                        ZCAN_20_MSG can_data[RX_BUFF_SIZE];
                        int len = VCI_Receive(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, can_data, RX_BUFF_SIZE, RX_WAIT_TIME); // CAN
                        if (len < 1)
                        {
                          printf("ZCAN Receive, len: %d\n", len);
                          // Retry once; the last response may take a long time while firmware writes to flash, wait 2 seconds then continue
                          usleep(2000 * 1000);
                          printf("Retrying ZCAN Receive...\n");
                          len = VCI_Receive(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, can_data, RX_BUFF_SIZE, RX_WAIT_TIME); // CANFD
                          printf("ZCAN Receive, len: %d\n", len);
                        }
                        if (len < 1)
                        {
                          return -1;
                        }

                        // Concatenate multiple CAN frames; all frames should have the same CAN ID
                        *can_id_out = can_data[0].hdr.id & 0x1FFFFFFF;

                        int idx = 0;
                        int total_dlc = 0;

                        for (int i = 0; i < len; i++)
                        {
                          ZCAN_20_MSG recv_data = can_data[i];
                          int can_dlc = recv_data.hdr.len;
                          for (int j = 0; j < can_dlc; j++)
                          {
                            data_out[idx++] = recv_data.dat[j];
                          }
                          total_dlc += can_dlc;
                        }

                        *data_len_out = total_dlc;
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

// ================== Signal handler and utility functions ==================
void handler(int sig)
{
  void *array[10];
  size_t size;

  // Get stack frames
  size = backtrace(array, 10);

  // Print all stack frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

int get_current_time_ms()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (int)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}
