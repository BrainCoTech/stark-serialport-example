#include "stark-sdk.h"
#include "zqwl-can/zlgcan.h"
#include <stdio.h>
#include <windows.h>

// This file implements the Windows CANFD example using the ZQWL CAN API
// (`zqwlcan.dll`), aligned in structure with the ZLG-based example
// `windows/revo2/revo2_canfd.cpp`.

// Global ZQWL handles
static long      g_device_handle  = INVALID_DEVICE_HANDLE;
static Handle_chl g_channel_handle = {INVALID_CHANNEL_HANDLE};

// Function declarations
bool setup_canfd();
void cleanup_can_resources();
void get_device_info(DeviceHandler *handle, uint8_t slave_id);

// Internal helpers (ZQWL CANFD)
static bool init_canfd_device(int device_type, int device_index);
static bool setup_canfd_baudrate(int channel_index, int abit_baud, int dbit_baud);
static bool start_canfd_channel(int channel_index);
static void setup_canfd_callbacks();

int main(int argc, char const *argv[]) {
  // Setup CANFD (device + channel + callbacks)
  if (!setup_canfd()) {
    return -1;
  }

  // Initialize STARK SDK
  init_logging(LOG_LEVEL_INFO);
  const uint8_t MASTER_ID = 1;
  auto handle = init_device_handler(STARK_PROTOCOL_TYPE_CAN_FD, MASTER_ID);
  printf("init_device_handler\n");

  // Get device information
  uint8_t slave_id = 0x7e; // 0x7e for left hand
  // uint8_t slave_id = 0x7f; // 0x7f for right hand
  get_device_info(handle, slave_id);
  printf("get_device_info\n");

  // Simple motion demo (same style as ZLG example)
  int delay = 1000; // 1000 ms
  uint16_t positions[6] = {500, 500, 1000, 1000, 1000, 1000};
  uint16_t durations[6] = {300, 300, 300, 300, 300, 300};
  stark_set_finger_positions_and_durations(handle, slave_id, positions, durations, 6);
  Sleep(delay); // Wait for fingers to reach target positions

  uint16_t open_positions[6] = {500, 500, 0, 0, 0, 0};
  stark_set_finger_positions_and_durations(handle, slave_id, open_positions, durations, 6);

  // Clean up resources
  cleanup_can_resources();
  return 0;
}

void get_device_info(DeviceHandler *handle, uint8_t slave_id) {
  CDeviceInfo *info = stark_get_device_info(handle, slave_id);
  if (info != NULL) {
    printf("Slave[%hhu] Serial Number: %s, FW: %s\n", slave_id,
           info->serial_number, info->firmware_version);
    if (stark_uses_revo1_motor_api(info->hardware_type)) {
      printf("Device uses Revo1 motor API, hardware type: %hhu\n", info->hardware_type);
      free_device_info(info);
      exit(1);
    }
    free_device_info(info);
  } else {
    printf("Error: Failed to get device info\n");
    exit(1);
  }
}

/****************************************************************************/
// CANFD Functions (ZQWL Windows API, aligned with ZLG example)
/****************************************************************************/

// ZQWL device constants (use the same USBCANFD type as ZLG example)
#define ZCANFD_DEVICE_TYPE   ZCAN_USBCANFD_100U // USBCANFD device type
#define ZCANFD_DEVICE_INDEX  0                  // Card index
#define ZCANFD_CHANNEL_INDEX 0                  // Channel index
#define RX_WAIT_TIME         100                // Receive wait time (ms)
#define RX_BUFF_SIZE         1000               // Receive buffer size

// Initialize CANFD device (open ZQWL device)
static bool init_canfd_device(int device_type, int device_index) {
  g_device_handle = ZCAN_OpenDevice(device_type, device_index, 0);
  if (g_device_handle == INVALID_DEVICE_HANDLE) {
    printf("ZQWL: Failed to open CANFD device\n");
    return false;
  }
  printf("ZQWL: ZCAN_OpenDevice, device_handle: %ld\n", g_device_handle);
  return true;
}

// Configure CANFD baud rates and clear filters (per-channel)
static bool setup_canfd_baudrate(int channel_index, int abit_baud, int dbit_baud) {
  char path[64] = {0};
  char value[32] = {0};

  // Arbitration domain baudrate
  sprintf_s(path, "%d/canfd_abit_baud_rate", channel_index);
  sprintf_s(value, "%d", abit_baud);
  if (ZCAN_SetValue(g_device_handle, path, value) != STATUS_OK) {
    printf("ZQWL: Failed to set %s\n", path);
    return false;
  }

  // Data domain baudrate
  sprintf_s(path, "%d/canfd_dbit_baud_rate", channel_index);
  sprintf_s(value, "%d", dbit_baud);
  if (ZCAN_SetValue(g_device_handle, path, value) != STATUS_OK) {
    printf("ZQWL: Failed to set %s\n", path);
    return false;
  }

  // Clear filter configuration
  sprintf_s(path, "%d/filter_clear", channel_index);
  strcpy_s(value, "0");
  if (ZCAN_SetValue(g_device_handle, path, value) != STATUS_OK) {
    printf("ZQWL: Failed to clear filter: %s\n", path);
    return false;
  }

  return true;
}

// Initialize and start CANFD channel
static bool start_canfd_channel(int channel_index) {
  ZCAN_CHANNEL_INIT_CONFIG cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.can_type    = TYPE_CANFD; // CANFD device
  cfg.canfd.mode  = 0;          // 0 - normal mode, 1 - listen-only
  cfg.canfd.brp   = 0;          // Use default timing; detailed timing is set via ZCAN_SetValue

  printf("ZQWL: ZCAN_InitCAN\n");
  g_channel_handle = ZCAN_InitCAN(g_device_handle, channel_index, cfg)[0];
  if (g_channel_handle.handle == INVALID_CHANNEL_HANDLE) {
    printf("ZQWL: Failed to initialize CANFD channel\n");
    return false;
  }

  printf("ZQWL: ZCAN_StartCAN\n");
  if (ZCAN_StartCAN(g_channel_handle) != STATUS_OK) {
    printf("ZQWL: Failed to start CANFD channel\n");
    return false;
  }

  return true;
}

// Setup CANFD TX/RX callbacks used by STARK SDK
static void setup_canfd_callbacks(void) {
  // CANFD transmit callback
  set_can_tx_callback([](uint8_t slave_id,
                         uint32_t can_id,
                         const uint8_t *data,
                         uintptr_t data_len) -> int {
    printf("[ZQWL CANFD TX] slave_id=%u can_id=0x%08X len=%zu\n",
           slave_id, can_id, (size_t)data_len);

    ZCAN_TransmitFD_Data tx{};

    // Configure CAN ID: use MAKE_CAN_ID helper (EFF=1, RTR=0, ERR=0)
    tx.frame.can_id = MAKE_CAN_ID(can_id, 1, 0, 0);
    tx.frame.len    = (data_len > 64) ? 64 : (BYTE)data_len;

    // flags: bit0 = BRS (bit-rate switch)
    tx.frame.flags = 0;
    tx.frame.flags |= CANFD_BRS; // Use BRS like original ZQWL example

    // Copy data
    for (uintptr_t i = 0; i < data_len && i < 64; ++i) {
      tx.frame.data[i] = data[i];
    }

    tx.transmit_type = 0; // Normal send

    int sent = ZCAN_TransmitFD(g_channel_handle, &tx, 1);
    if (sent != 1) {
      printf("[ZQWL CANFD TX][ERROR] ZCAN_TransmitFD failed, sent=%d\n", sent);
      return -1;
    }

    printf("[ZQWL CANFD TX] success, sent=%d\n", sent);
    return 0;
  });

  // CANFD receive callback
  set_can_rx_callback([](uint8_t slave_id,
                         uint32_t *can_id_out,
                         uint8_t *data_out,
                         uintptr_t *data_len_out) -> int {
    ZCAN_ReceiveFD_Data rcv[RX_BUFF_SIZE];

    UINT pending = ZCAN_GetReceiveNum(g_channel_handle, TYPE_CANFD);
    if (pending == 0) {
      printf("[ZQWL CANFD RX] slave_id=%u, pending=0 (no frames)\n", slave_id);
      return -1;
    }
    if (pending > RX_BUFF_SIZE) {
      pending = RX_BUFF_SIZE;
    }

    printf("[ZQWL CANFD RX] slave_id=%u, pending=%u\n", slave_id, pending);

    UINT received = ZCAN_ReceiveFD(g_channel_handle, rcv, pending, RX_WAIT_TIME);
    if (received == 0) {
      printf("[ZQWL CANFD RX][ERROR] ReceiveFD returned 0 frames\n");
      return -1;
    }

    printf("[ZQWL CANFD RX] received=%u\n", received);

    // Prefer the last non-TX-echo frame (same pattern as ZLG example)
    const ZCAN_ReceiveFD_Data *last_rx = nullptr;
    for (int i = (int)received - 1; i >= 0; --i) {
      const canfd_frame &frame = rcv[i].frame;
      // TX echo uses flag bit5 = 1 (TX_ECHO_FLAG); skip those
      if (frame.flags & TX_ECHO_FLAG) {
        printf("[ZQWL CANFD RX] frame[%d] is TX echo, id=0x%08X len=%u\n",
               i, frame.can_id, frame.len);
        continue;
      }
      last_rx = &rcv[i];
      break;
    }
    if (last_rx == nullptr) {
      // All frames are TX echoes
      printf("[ZQWL CANFD RX][WARN] all %u frames are TX echo, no RX data\n", received);
      return -1;
    }

    const canfd_frame &frame = last_rx->frame;
    *can_id_out    = frame.can_id;
    *data_len_out  = frame.len;

    printf("[ZQWL CANFD RX] picked frame: can_id=0x%08X len=%u\n",
           frame.can_id, frame.len);

    for (int i = 0; i < frame.len && i < 64; ++i) {
      data_out[i] = frame.data[i];
    }

    return 0;
  });
}

void cleanup_can_resources(void) {
  if (g_channel_handle.handle != INVALID_CHANNEL_HANDLE) {
    ZCAN_ResetCAN(g_channel_handle);
    g_channel_handle.handle = INVALID_CHANNEL_HANDLE;
  }
  if (g_device_handle != INVALID_DEVICE_HANDLE) {
    ZCAN_CloseDevice(g_device_handle);
    g_device_handle = INVALID_DEVICE_HANDLE;
  }
  printf("ZQWL CAN/CANFD resources cleaned up.\n");
}

/****************************************************************************/
// High-level CANFD setup (device + channel + callbacks)
/****************************************************************************/

bool setup_canfd(void) {
  printf("Setting up CANFD (ZQWL)...\n");

  const int device_type    = ZCANFD_DEVICE_TYPE;
  const int device_index   = ZCANFD_DEVICE_INDEX;
  const int channel_index  = ZCANFD_CHANNEL_INDEX;
  const int abit_baudrate  = 1000000; // 1 Mbps arbitration domain
  const int dbit_baudrate  = 5000000; // 5 Mbps data domain

  // Step 1: Open device
  if (!init_canfd_device(device_type, device_index)) {
    return false;
  }

  // Step 2: Configure baud rates & filter
  if (!setup_canfd_baudrate(channel_index, abit_baudrate, dbit_baudrate)) {
    cleanup_can_resources();
    return false;
  }

  // Step 3: Initialize and start channel
  if (!start_canfd_channel(channel_index)) {
    cleanup_can_resources();
    return false;
  }

  // Step 4: Setup TX/RX callbacks used by STARK SDK
  setup_canfd_callbacks();

  printf("CANFD setup (ZQWL) completed successfully\n");
  return true;
}
