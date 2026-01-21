/**
 * @file can_common.cpp
 * @brief Implementation of common utility functions for CAN/CANFD examples
 */

#include "can_common.h"
#include "stark-sdk.h"
#include <errno.h>
#include <poll.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#ifndef STARK_USE_ZLG
#define STARK_USE_ZLG 1
#endif

#if STARK_USE_ZLG
#include "zlgcan/zcan.h"
#endif

#ifndef STARK_USE_SOCKETCAN
#define STARK_USE_SOCKETCAN 1
#endif

#if STARK_USE_SOCKETCAN
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#endif

/****************************************************************************/
// Get CAN backend
// Determines the CAN backend to use: ZLG or SocketCAN
// @return CanBackend enum value
/****************************************************************************/

enum CanBackend {
  kBackendZlg,
  kBackendSocketcan,
};

static CanBackend get_can_backend(void) {
  static CanBackend backend = kBackendZlg;
  static bool initialized = false;
  if (initialized) {
    return backend;
  }
  initialized = true;

  const char *env = getenv("STARK_CAN_BACKEND");
  if (env && (strcasecmp(env, "socketcan") == 0 || strcasecmp(env, "socket_can") == 0 ||
              strcasecmp(env, "socket") == 0)) {
#if STARK_USE_SOCKETCAN
    backend = kBackendSocketcan;
#else
    printf("SocketCAN backend not compiled. Rebuild with CAN_BACKEND=socketcan or both.\n");
#endif
  }
  return backend;
}

/****************************************************************************/
// SocketCAN implementation
/****************************************************************************/

#if STARK_USE_SOCKETCAN
static int g_socketcan_fd = -1;
static bool g_socketcan_is_canfd = false;

static const char *get_socketcan_iface(void) {
  const char *iface = getenv("STARK_SOCKETCAN_IFACE");
  if (iface && iface[0] != '\0') {
    return iface;
  }
  return "can0";
}

static bool set_socketcan_nonblocking(int fd) {
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    return false;
  }
  if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    return false;
  }
  return true;
}

static bool init_socketcan_device(bool enable_fd) {
  if (g_socketcan_fd >= 0) {
    if (!enable_fd || g_socketcan_is_canfd) {
      return true;
    }
    close(g_socketcan_fd);
    g_socketcan_fd = -1;
  }

  int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    printf("Failed to open SocketCAN socket: %s\n", strerror(errno));
    return false;
  }

  if (enable_fd) {
    int enable = 1;
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) != 0) {
      printf("Failed to enable CANFD on SocketCAN: %s\n", strerror(errno));
      close(fd);
      return false;
    }
  }

  struct ifreq ifr;
  memset(&ifr, 0, sizeof(ifr));
  const char *iface = get_socketcan_iface();
  strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    printf("Failed to get SocketCAN iface %s: %s\n", iface, strerror(errno));
    close(fd);
    return false;
  }

  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    printf("Failed to bind SocketCAN iface %s: %s\n", iface, strerror(errno));
    close(fd);
    return false;
  }

  if (!set_socketcan_nonblocking(fd)) {
    printf("Failed to set SocketCAN socket nonblocking: %s\n", strerror(errno));
    close(fd);
    return false;
  }

  g_socketcan_fd = fd;
  g_socketcan_is_canfd = enable_fd;
  return true;
}

static int socketcan_wait_for_frame(void) {
  struct pollfd pfd;
  pfd.fd = g_socketcan_fd;
  pfd.events = POLLIN;
  pfd.revents = 0;
  return poll(&pfd, 1, RX_WAIT_TIME);
}

static int socketcan_send_can(uint32_t can_id, const uint8_t *data, uintptr_t data_len) {
  if (g_socketcan_fd < 0) {
    return -1;
  }

  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));
  if (can_id > CAN_SFF_MASK) {
    frame.can_id = (can_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  } else {
    frame.can_id = can_id & CAN_SFF_MASK;
  }
  frame.can_dlc = (data_len > 8) ? 8 : static_cast<uint8_t>(data_len);
  memcpy(frame.data, data, frame.can_dlc);

  int nbytes = write(g_socketcan_fd, &frame, sizeof(frame));
  return (nbytes == static_cast<int>(sizeof(frame))) ? 0 : -1;
}

static int socketcan_recv_can(uint32_t *can_id_out, uint8_t *data_out, uintptr_t *data_len_out) {
  if (g_socketcan_fd < 0) {
    return -1;
  }

  int ret = socketcan_wait_for_frame();
  if (ret <= 0) {
    return -1;
  }

  struct can_frame frame;
  int nbytes = read(g_socketcan_fd, &frame, sizeof(frame));
  if (nbytes != static_cast<int>(sizeof(frame))) {
    return -1;
  }
  if (frame.can_id & CAN_RTR_FLAG) {
    return -1;
  }

  uint32_t can_id = (frame.can_id & CAN_EFF_FLAG) ? (frame.can_id & CAN_EFF_MASK)
                                                  : (frame.can_id & CAN_SFF_MASK);
  *can_id_out = can_id;
  int total_dlc = 0;

  int can_dlc = frame.can_dlc;
  for (int i = 0; i < can_dlc; ++i) {
    data_out[total_dlc++] = frame.data[i];
  }

  while (1) {
    nbytes = read(g_socketcan_fd, &frame, sizeof(frame));
    if (nbytes < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      return -1;
    }
    if (nbytes != static_cast<int>(sizeof(frame))) {
      break;
    }
    if (frame.can_id & CAN_RTR_FLAG) {
      continue;
    }
    can_dlc = frame.can_dlc;
    for (int i = 0; i < can_dlc; ++i) {
      data_out[total_dlc++] = frame.data[i];
    }
  }

  *data_len_out = total_dlc;
  return 0;
}

static int socketcan_send_canfd(uint32_t can_id, const uint8_t *data, uintptr_t data_len) {
  if (g_socketcan_fd < 0 || !g_socketcan_is_canfd) {
    return -1;
  }

  struct canfd_frame frame;
  memset(&frame, 0, sizeof(frame));
  frame.can_id = (can_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  frame.len = (data_len > 64) ? 64 : static_cast<uint8_t>(data_len);
  frame.flags = CANFD_BRS;
  memcpy(frame.data, data, frame.len);

  int nbytes = write(g_socketcan_fd, &frame, sizeof(frame));
  return (nbytes == static_cast<int>(sizeof(frame))) ? 0 : -1;
}

static int socketcan_recv_canfd(uint32_t *can_id_out, uint8_t *data_out, uintptr_t *data_len_out) {
  if (g_socketcan_fd < 0 || !g_socketcan_is_canfd) {
    return -1;
  }

  int ret = socketcan_wait_for_frame();
  if (ret <= 0) {
    return -1;
  }

  struct canfd_frame frame;
  int nbytes = read(g_socketcan_fd, &frame, sizeof(frame));
  if (nbytes != static_cast<int>(sizeof(frame))) {
    return -1;
  }

  uint32_t can_id = (frame.can_id & CAN_EFF_FLAG) ? (frame.can_id & CAN_EFF_MASK)
                                                  : (frame.can_id & CAN_SFF_MASK);
  *can_id_out = can_id;
  *data_len_out = frame.len;
  memcpy(data_out, frame.data, frame.len);
  return 0;
}
#endif

/****************************************************************************/
// Device initialization CAN/CANFD
// Initializes the CAN/CANFD device
// @return true if successful, false otherwise
/****************************************************************************/

bool init_can_device(void) {
  if (get_can_backend() == kBackendSocketcan) {
#if STARK_USE_SOCKETCAN
    return init_socketcan_device(false);
#else
    printf("SocketCAN backend not compiled. Rebuild with CAN_BACKEND=socketcan or both.\n");
    return false;
#endif
  }
#if STARK_USE_ZLG
  // Open device
  if (!VCI_OpenDevice(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX)) {
    printf("Failed to open CAN device\n");
    return false;
  }
  return true;
#else
  printf("ZLG backend disabled. Rebuild with CAN_BACKEND=zlg.\n");
  return false;
#endif
}

bool init_canfd_device(void) {
  if (get_can_backend() == kBackendSocketcan) {
#if STARK_USE_SOCKETCAN
    return init_socketcan_device(true);
#else
    printf("SocketCAN backend not compiled. Rebuild with CAN_BACKEND=socketcan or both.\n");
    return false;
#endif
  }
  // Same as CAN device initialization for ZLG
  return init_can_device();
}

/****************************************************************************/
// Channel configuration and startup CAN/CANFD
// Configures and starts the CAN/CANFD channel
// @return true if successful, false otherwise
/****************************************************************************/

bool start_can_channel(void) {
  if (get_can_backend() == kBackendSocketcan) {
    return true;
  }
#if STARK_USE_ZLG
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
#else
  printf("ZLG backend disabled. Rebuild with CAN_BACKEND=zlg.\n");
  return false;
#endif
}

bool start_canfd_channel(void) {
  // Same configuration as CAN channel for ZLG
  return start_can_channel();
}

/****************************************************************************/
// CAN callbacks
/****************************************************************************/

void setup_can_callbacks(void) {
  if (get_can_backend() == kBackendSocketcan) {
#if STARK_USE_SOCKETCAN
    set_can_tx_callback([](uint8_t slave_id,
                           uint32_t can_id,
                           const uint8_t *data,
                           uintptr_t data_len) -> int {
      (void)slave_id;
      return socketcan_send_can(can_id, data, data_len);
    });

    set_can_rx_callback([](uint8_t slave_id,
                           uint32_t *can_id_out,
                           uint8_t *data_out,
                           uintptr_t *data_len_out) -> int {
      (void)slave_id;
      return socketcan_recv_can(can_id_out, data_out, data_len_out);
    });
#else
    printf("SocketCAN backend not compiled. Rebuild with CAN_BACKEND=socketcan or both.\n");
#endif
    return;
  }
#if STARK_USE_ZLG
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
#else
  printf("ZLG backend disabled. Rebuild with CAN_BACKEND=zlg.\n");
#endif
}

/****************************************************************************/
// CANFD callbacks
/****************************************************************************/

void setup_canfd_callbacks(void) {
  if (get_can_backend() == kBackendSocketcan) {
#if STARK_USE_SOCKETCAN
    set_can_tx_callback([](uint8_t slave_id,
                           uint32_t can_id,
                           const uint8_t *data,
                           uintptr_t data_len) -> int {
      (void)slave_id;
      return socketcan_send_canfd(can_id, data, data_len);
    });

    set_can_rx_callback([](uint8_t slave_id,
                           uint32_t *can_id_out,
                           uint8_t *data_out,
                           uintptr_t *data_len_out) -> int {
      (void)slave_id;
      return socketcan_recv_canfd(can_id_out, data_out, data_len_out);
    });
#else
    printf("SocketCAN backend not compiled. Rebuild with CAN_BACKEND=socketcan or both.\n");
#endif
    return;
  }
#if STARK_USE_ZLG
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
#else
  printf("ZLG backend disabled. Rebuild with CAN_BACKEND=zlg.\n");
#endif
}

/****************************************************************************/
// setup CAN/CANFD
// Performs complete CAN/CANFD setup: device init, channel start, and callback setup
// @return true if successful, false otherwise
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

/****************************************************************************/
// Resource cleanup CAN/CANFD
// Cleans up CAN/CANFD resources: device close, channel reset, and callback cleanup
// @return void
/****************************************************************************/

void cleanup_can_resources(void) {
  if (get_can_backend() == kBackendSocketcan) {
#if STARK_USE_SOCKETCAN
    if (g_socketcan_fd >= 0) {
      close(g_socketcan_fd);
      g_socketcan_fd = -1;
      g_socketcan_is_canfd = false;
    }
    printf("SocketCAN resources cleaned up.\n");
#else
    printf("SocketCAN backend not compiled. Rebuild with CAN_BACKEND=socketcan or both.\n");
#endif
    return;
  }
#if STARK_USE_ZLG
  VCI_ResetCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX);
  VCI_CloseDevice(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX);
  printf("CAN/CANFD resources cleaned up.\n");
#else
  printf("ZLG backend disabled. Rebuild with CAN_BACKEND=zlg.\n");
#endif
}
