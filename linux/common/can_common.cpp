/**
 * @file can_common.cpp
 * @brief Implementation of common utility functions for CAN/CANFD examples
 */

#include "can_common.h"
#include "stark-sdk.h"
#include "zlgcan/zcan.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/****************************************************************************/
// Device initialization
/****************************************************************************/

bool init_can_device(void) {
  // Open device
  if (!VCI_OpenDevice(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX)) {
    printf("Failed to open CAN device\n");
    return false;
  }
  return true;
}

bool init_canfd_device(void) {
  // Same as CAN device initialization
  return init_can_device();
}

/****************************************************************************/
// Channel configuration and startup
/****************************************************************************/

bool start_can_channel(void) {
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
  if (!VCI_InitCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, &init)) {
    printf("Failed to initialize CAN channel\n");
    return false;
  }

  // Start CAN channel
  if (!VCI_StartCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX)) {
    printf("Failed to start CAN channel\n");
    return false;
  }

  return true;
}

bool start_canfd_channel(void) {
  // Same configuration as CAN channel
  return start_can_channel();
}

/****************************************************************************/
// CAN callbacks
/****************************************************************************/

void setup_can_callbacks(void) {
  // CAN transmit callback
  set_can_tx_callback([](uint8_t slave_id,
                         uint32_t can_id,
                         const uint8_t *data,
                         uintptr_t data_len) -> int {
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
    for (uintptr_t i = 0; i < data_len && i < 8; ++i) {
      can_msg.dat[i] = data[i];
    }

    // Transmit CAN frame
    int result = VCI_Transmit(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, &can_msg, 1);
    if (result != 1)
      printf("CAN Transmit result: %d\n", result);
    return result == 1 ? 0 : -1; // 0 indicates success
  });

  // CAN receive callback
  set_can_rx_callback([](uint8_t slave_id,
                         uint32_t *can_id_out,
                         uint8_t *data_out,
                         uintptr_t *data_len_out) -> int {
    // Read data
    ZCAN_20_MSG can_data[RX_BUFF_SIZE];
    int len = VCI_Receive(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, can_data, RX_BUFF_SIZE, RX_WAIT_TIME);
    if (len < 1) {
      printf("CAN Receive, len: %d\n", len);
      // Retry once; the last response may take a long time while firmware writes to flash, wait 2 seconds then continue
      usleep(2000 * 1000);
      printf("Retrying CAN Receive...\n");
      len = VCI_Receive(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, can_data, RX_BUFF_SIZE, RX_WAIT_TIME);
      printf("CAN Receive, len: %d\n", len);
    }
    if (len < 1) {
      return -1;
    }

    // Concatenate multiple CAN frames; all frames should have the same CAN ID
    *can_id_out = can_data[0].hdr.id & 0x1FFFFFFF;

    int idx = 0;
    int total_dlc = 0;

    for (int i = 0; i < len; i++) {
      ZCAN_20_MSG recv_data = can_data[i];
      int can_dlc = recv_data.hdr.len;
      for (int j = 0; j < can_dlc; j++) {
        data_out[idx++] = recv_data.dat[j];
      }
      total_dlc += can_dlc;
    }

    *data_len_out = total_dlc;
    return 0; // Return 0 on success
  });
}

/****************************************************************************/
// CANFD callbacks
/****************************************************************************/

void setup_canfd_callbacks(void) {
  // CANFD transmit callback
  set_can_tx_callback([](uint8_t slave_id,
                         uint32_t canfd_id,
                         const uint8_t *data,
                         uintptr_t data_len) -> int {
    // Construct CANFD transmit message
    ZCAN_FD_MSG canfd_msg;
    memset(&canfd_msg, 0, sizeof(ZCAN_FD_MSG));

    canfd_msg.hdr.inf.txm = 0; // 0 - normal transmit
    canfd_msg.hdr.inf.fmt = 1; // 0 - CAN, 1 - CANFD
    canfd_msg.hdr.inf.sdf = 0; // 0 - data frame, CANFD only supports data frames
    canfd_msg.hdr.inf.sef = 1; // 0 - standard frame, 1 - extended frame

    canfd_msg.hdr.id = canfd_id;              // ID
    canfd_msg.hdr.chn = ZCANFD_CHANNEL_INDEX; // Channel
    canfd_msg.hdr.len = data_len;             // Data length

    // Fill data, maximum length 64
    for (uintptr_t i = 0; i < data_len && i < 64; ++i) {
      canfd_msg.dat[i] = data[i];
    }

    // Transmit CANFD frame
    int result = VCI_TransmitFD(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, &canfd_msg, 1);
    if (result != 1)
      printf("CANFD Transmit result: %d\n", result);
    return result == 1 ? 0 : -1; // 0 indicates success
  });

  // CANFD receive callback
  set_can_rx_callback([](uint8_t slave_id,
                         uint32_t *canfd_id_out,
                         uint8_t *data_out,
                         uintptr_t *data_len_out) -> int {
    // Read data
    ZCAN_FD_MSG canfd_data[RX_BUFF_SIZE];
    int len = VCI_ReceiveFD(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, canfd_data, RX_BUFF_SIZE, RX_WAIT_TIME);
    if (len < 1) {
      printf("CANFD Receive, len: %d\n", len);
      // Retry once; the last response may take a long time while firmware writes to flash, wait 2 seconds then continue
      usleep(2000 * 1000);
      printf("Retrying CANFD Receive...\n");
      len = VCI_ReceiveFD(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, canfd_data, RX_BUFF_SIZE, RX_WAIT_TIME);
      printf("CANFD Receive, len: %d\n", len);
    }
    if (len < 1) {
      return -1;
    }

    // Handle the first frame
    ZCAN_FD_MSG recv_data = canfd_data[0];
    int canfd_dlc = recv_data.hdr.len;
    *canfd_id_out = recv_data.hdr.id;
    *data_len_out = canfd_dlc;

    for (int j = 0; j < canfd_dlc && j < 64; j++) {
      data_out[j] = recv_data.dat[j];
    }
    return 0; // Return 0 on success
  });
}

/****************************************************************************/
// Resource cleanup
/****************************************************************************/

bool setup_can(void) {
  printf("Setting up CAN...\n");
  
  // Step 1: Initialize CAN device
  if (!init_can_device()) {
    printf("Failed to initialize CAN device\n");
    return false;
  }
  
  // Step 2: Start CAN channel
  if (!start_can_channel()) {
    printf("Failed to start CAN channel\n");
    cleanup_can_resources();
    return false;
  }
  
  // Step 3: Setup CAN callbacks
  setup_can_callbacks();
  
  printf("CAN setup completed successfully\n");
  return true;
}

bool setup_canfd(void) {
  printf("Setting up CANFD...\n");
  
  // Step 1: Initialize CANFD device
  if (!init_canfd_device()) {
    printf("Failed to initialize CANFD device\n");
    return false;
  }
  
  // Step 2: Start CANFD channel
  if (!start_canfd_channel()) {
    printf("Failed to start CANFD channel\n");
    cleanup_can_resources();
    return false;
  }
  
  // Step 3: Setup CANFD callbacks
  setup_canfd_callbacks();
  
  printf("CANFD setup completed successfully\n");
  return true;
}

void cleanup_can_resources(void) {
  VCI_ResetCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX);
  VCI_CloseDevice(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX);
  printf("CAN/CANFD resources cleaned up.\n");
}
