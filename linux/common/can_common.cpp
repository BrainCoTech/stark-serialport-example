/**
 * @file can_common.cpp
 * @brief Implementation of common utility functions for CAN/CANFD examples
 * 
 * Supports multiple CAN backends with runtime selection:
 * - SocketCAN (Linux): Native kernel CAN interface
 * - ZLG: USB-CANFD adapter (dynamic loading, no compile-time dependency)
 * 
 * Backend selection:
 * - Environment: STARK_CAN_BACKEND=socketcan|zlg
 * - Default: SocketCAN on Linux if available, otherwise ZLG
 */

#include "can_common.h"
#include "stark-sdk.h"
#include <dlfcn.h>
#include <errno.h>
#include <poll.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

// Backend compile flags (both can be enabled for runtime selection)
#ifndef STARK_USE_ZLG
  #define STARK_USE_ZLG 0
#endif

#ifndef STARK_USE_SOCKETCAN
  #define STARK_USE_SOCKETCAN 0
#endif

// ZLG types and function pointers (for dynamic loading)
#if STARK_USE_ZLG
#include "zlgcan/zcan.h"

// Dynamic library handle
static void* g_zlg_lib = nullptr;
static bool g_zlg_loaded = false;

// Function pointers
typedef int (*VCI_OpenDevice_t)(unsigned int, unsigned int, unsigned int);
typedef int (*VCI_CloseDevice_t)(unsigned int, unsigned int);
typedef int (*VCI_InitCAN_t)(unsigned int, unsigned int, unsigned int, void*);
typedef int (*VCI_StartCAN_t)(unsigned int, unsigned int, unsigned int);
typedef int (*VCI_ResetCAN_t)(unsigned int, unsigned int, unsigned int);
typedef int (*VCI_Transmit_t)(unsigned int, unsigned int, unsigned int, void*, unsigned int);
typedef int (*VCI_Receive_t)(unsigned int, unsigned int, unsigned int, void*, unsigned int, int);
typedef int (*VCI_TransmitFD_t)(unsigned int, unsigned int, unsigned int, void*, unsigned int);
typedef int (*VCI_ReceiveFD_t)(unsigned int, unsigned int, unsigned int, void*, unsigned int, int);

static VCI_OpenDevice_t pVCI_OpenDevice = nullptr;
static VCI_CloseDevice_t pVCI_CloseDevice = nullptr;
static VCI_InitCAN_t pVCI_InitCAN = nullptr;
static VCI_StartCAN_t pVCI_StartCAN = nullptr;
static VCI_ResetCAN_t pVCI_ResetCAN = nullptr;
static VCI_Transmit_t pVCI_Transmit = nullptr;
static VCI_Receive_t pVCI_Receive = nullptr;
static VCI_TransmitFD_t pVCI_TransmitFD = nullptr;
static VCI_ReceiveFD_t pVCI_ReceiveFD = nullptr;

static bool load_zlg_library(void) {
  if (g_zlg_loaded) return g_zlg_lib != nullptr;
  g_zlg_loaded = true;

  // Check for custom library path
  const char* lib_path = getenv("STARK_ZLG_LIB_PATH");

  if (lib_path && lib_path[0]) {
    g_zlg_lib = dlopen(lib_path, RTLD_NOW);
  }
  if (!g_zlg_lib) g_zlg_lib = dlopen("libusbcanfd.so", RTLD_NOW);
  if (!g_zlg_lib) g_zlg_lib = dlopen("./libusbcanfd.so", RTLD_NOW);

  if (!g_zlg_lib) {
    printf("[ZLG] Library not found. Set STARK_ZLG_LIB_PATH or install to system path.\n");
    return false;
  }

  #define LOAD_SYM(name) p##name = (name##_t)dlsym(g_zlg_lib, #name)

  LOAD_SYM(VCI_OpenDevice);
  LOAD_SYM(VCI_CloseDevice);
  LOAD_SYM(VCI_InitCAN);
  LOAD_SYM(VCI_StartCAN);
  LOAD_SYM(VCI_ResetCAN);
  LOAD_SYM(VCI_Transmit);
  LOAD_SYM(VCI_Receive);
  LOAD_SYM(VCI_TransmitFD);
  LOAD_SYM(VCI_ReceiveFD);

#undef LOAD_SYM

  if (!pVCI_OpenDevice || !pVCI_CloseDevice || !pVCI_InitCAN || 
      !pVCI_StartCAN || !pVCI_Transmit || !pVCI_Receive) {
    printf("[ZLG] Failed to load required functions\n");
    dlclose(g_zlg_lib);
    g_zlg_lib = nullptr;
    return false;
  }

  printf("[ZLG] Library loaded successfully\n");
  return true;
}
#endif // STARK_USE_ZLG

// SocketCAN backend (Linux only)
#if STARK_USE_SOCKETCAN
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#endif

/****************************************************************************/
// Get CAN backend (runtime selection)
/****************************************************************************/

enum CanBackend {
  kBackendNone,
  kBackendZlg,
  kBackendSocketcan,
};

static CanBackend g_selected_backend = kBackendNone;

static CanBackend get_can_backend(void) {
  if (g_selected_backend != kBackendNone) {
    return g_selected_backend;
  }

  // Check environment variable
  const char *env = getenv("STARK_CAN_BACKEND");
  if (env) {
    if (strcasecmp(env, "socketcan") == 0 || strcasecmp(env, "socket") == 0) {
#if STARK_USE_SOCKETCAN
      g_selected_backend = kBackendSocketcan;
      return g_selected_backend;
#else
      printf("[WARN] SocketCAN not compiled in, trying ZLG\n");
#endif
    } else if (strcasecmp(env, "zlg") == 0) {
#if STARK_USE_ZLG
      if (load_zlg_library()) {
        g_selected_backend = kBackendZlg;
        return g_selected_backend;
      }
#else
      printf("[WARN] ZLG not compiled in\n");
#endif
    }
  }

  // Default: prefer SocketCAN on Linux, then ZLG
#if STARK_USE_SOCKETCAN
  g_selected_backend = kBackendSocketcan;
#elif STARK_USE_ZLG
  if (load_zlg_library()) {
    g_selected_backend = kBackendZlg;
  }
#endif

  return g_selected_backend;
}

// Allow explicit backend selection (called from CLI parsing)
void set_can_backend_zlg(void) {
#if STARK_USE_ZLG
  if (load_zlg_library()) {
    g_selected_backend = kBackendZlg;
  }
#else
  printf("[ERROR] ZLG backend not compiled\n");
#endif
}

void set_can_backend_socketcan(void) {
#if STARK_USE_SOCKETCAN
  g_selected_backend = kBackendSocketcan;
#else
  printf("[ERROR] SocketCAN backend not compiled\n");
#endif
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

static int socketcan_recv_can(uint32_t expected_can_id, uint8_t expected_frames,
                              uint32_t *can_id_out, uint8_t *data_out, uintptr_t *data_len_out) {
  if (g_socketcan_fd < 0) {
    return -1;
  }

  int total_dlc = 0;
  int received_count = 0;
  uint8_t total_frames = 0;
  bool is_multi_frame = false;

  // Check if this is DFU mode (expected_can_id == 0)
  bool is_dfu_mode = (expected_can_id == 0);

  // Extract command from CAN ID to detect multi-frame commands
  uint8_t cmd = (expected_can_id >> 3) & 0x0F;
  bool is_multi_frame_cmd = (cmd == 0x0B || cmd == 0x0D);  // MultiRead or TouchSensorRead

  // Determine retry strategy (aligned with Rust ZQWL):
  // - DFU mode: 200 attempts (for CRC verification)
  // - Multi-frame commands: 5 attempts
  // - Single frame: 2 attempts
  int max_attempts = is_dfu_mode ? 200 : (expected_frames > 1 || is_multi_frame_cmd ? 5 : 2);

  for (int attempt = 0; attempt < max_attempts; attempt++) {
    int ret = socketcan_wait_for_frame();
    if (ret <= 0) {
      int wait_ms = (attempt < 5) ? 2 : 5;
      usleep(wait_ms * 1000);
      continue;
    }

    struct can_frame frame;
    int nbytes = read(g_socketcan_fd, &frame, sizeof(frame));
    if (nbytes != static_cast<int>(sizeof(frame))) {
      continue;
    }
    if (frame.can_id & CAN_RTR_FLAG) {
      continue;
    }

    uint32_t can_id = (frame.can_id & CAN_EFF_FLAG) ? (frame.can_id & CAN_EFF_MASK)
                                                    : (frame.can_id & CAN_SFF_MASK);
    int can_dlc = frame.can_dlc;

    // Skip non-matching frames (unless DFU mode)
    if (!is_dfu_mode && expected_can_id != 0 && can_id != expected_can_id) {
      continue;
    }

    // Check for multi-frame protocol format
    if (is_multi_frame_cmd && can_dlc > 0) {
      uint8_t frame_header = frame.data[0];

      if (cmd == 0x0B) {
        // MultiRead format: [addr, len|flag, data...]
        if (can_dlc >= 2) {
          uint8_t len_and_flag = frame.data[1];
          bool is_last = (len_and_flag & 0x80) != 0;

          // Append entire frame (8 bytes)
          for (int j = 0; j < can_dlc; j++) {
            data_out[total_dlc++] = frame.data[j];
          }
          received_count++;

          // Check if this is the last frame
          if (is_last) {
            *can_id_out = can_id;
            *data_len_out = total_dlc;
            return 0;
          }
          continue;
        }
      } else if (cmd == 0x0D) {
        // TouchSensorRead format: [total:4bit|seq:4bit, data...]
        uint8_t total = (frame_header >> 4) & 0x0F;
        uint8_t seq = frame_header & 0x0F;

        // Detect multi-frame data
        if (total > 0 && seq > 0) {
          if (!is_multi_frame) {
            is_multi_frame = true;
            total_frames = total;
          }

          // Append entire frame (8 bytes)
          for (int j = 0; j < can_dlc; j++) {
            data_out[total_dlc++] = frame.data[j];
          }
          received_count++;

          // Check if all frames received
          if (received_count >= total_frames) {
            *can_id_out = can_id;
            *data_len_out = total_dlc;
            return 0;
          }
          continue;
        }
      }
    }

    // Standard single-frame or non-protocol data
    for (int j = 0; j < can_dlc; j++) {
      data_out[total_dlc++] = frame.data[j];
    }
    received_count++;
    *can_id_out = can_id;

    // For single frame request, return immediately
    if (expected_frames <= 1 && !is_multi_frame_cmd) {
      *data_len_out = total_dlc;
      return 0;
    }

    // Check if we have enough frames (for non-protocol multi-frame)
    if (expected_frames > 1 && received_count >= expected_frames) {
      *data_len_out = total_dlc;
      return 0;
    }
  }

  // Timeout - return whatever we have
  if (total_dlc > 0) {
    *can_id_out = expected_can_id;
    *data_len_out = total_dlc;
    return 0;
  }

  return -1;
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

static int socketcan_recv_canfd(uint32_t expected_can_id, uint8_t expected_frames,
                                uint32_t *can_id_out, uint8_t *data_out, uintptr_t *data_len_out) {
  if (g_socketcan_fd < 0 || !g_socketcan_is_canfd) {
    return -1;
  }

  (void)expected_frames;  // Not used for CANFD single-frame

  // CANFD CAN ID format: (slave_id << 16) | (master_id << 8) | payload_len
  uint8_t expected_slave_id = (expected_can_id >> 16) & 0xFF;
  uint8_t expected_master_id = (expected_can_id >> 8) & 0xFF;

  // CANFD uses application-layer chunking, so most operations are single-frame
  // Retry strategy (aligned with Rust ZQWL): normal=2, DFU not applicable for CANFD
  int max_attempts = 2;

  for (int attempt = 0; attempt < max_attempts; attempt++) {
    int ret = socketcan_wait_for_frame();
    if (ret <= 0) {
      int wait_ms = (attempt < 5) ? 2 : 5;
      usleep(wait_ms * 1000);
      continue;
    }

    // Read into canfd_frame buffer (can hold both CAN and CANFD frames)
    struct canfd_frame frame;
    int nbytes = read(g_socketcan_fd, &frame, sizeof(frame));
    
    // Accept both CAN frames (16 bytes) and CANFD frames (72 bytes)
    if (nbytes != static_cast<int>(sizeof(struct can_frame)) && 
        nbytes != static_cast<int>(sizeof(struct canfd_frame))) {
      continue;
    }

    uint32_t can_id = (frame.can_id & CAN_EFF_FLAG) ? (frame.can_id & CAN_EFF_MASK)
                                                    : (frame.can_id & CAN_SFF_MASK);

    // Match slave_id and master_id from CAN ID
    uint8_t resp_slave_id = (can_id >> 16) & 0xFF;
    uint8_t resp_master_id = (can_id >> 8) & 0xFF;

    if (resp_slave_id == expected_slave_id && resp_master_id == expected_master_id) {
      *can_id_out = can_id;
      *data_len_out = frame.len;
      memcpy(data_out, frame.data, frame.len);
      return 0;
    }
  }

  return -1;
}
#endif

/****************************************************************************/
// Device initialization CAN/CANFD
// Initializes the CAN/CANFD device
// @return true if successful, false otherwise
/****************************************************************************/

bool init_can_device(void) {
  CanBackend backend = get_can_backend();
  
  if (backend == kBackendSocketcan) {
#if STARK_USE_SOCKETCAN
    return init_socketcan_device(false);
#else
    printf("[ERROR] SocketCAN not compiled\n");
    return false;
#endif
  }
  
  if (backend == kBackendZlg) {
#if STARK_USE_ZLG
    if (!pVCI_OpenDevice) {
      printf("[ERROR] ZLG library not loaded\n");
      return false;
    }
    if (!pVCI_OpenDevice(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX)) {
      printf("[ERROR] Failed to open ZLG CAN device\n");
      return false;
    }
    return true;
#else
    printf("[ERROR] ZLG not compiled\n");
    return false;
#endif
  }
  
  printf("[ERROR] No CAN backend available\n");
  return false;
}

bool init_canfd_device(void) {
  CanBackend backend = get_can_backend();
  
  if (backend == kBackendSocketcan) {
#if STARK_USE_SOCKETCAN
    return init_socketcan_device(true);
#else
    printf("[ERROR] SocketCAN not compiled\n");
    return false;
#endif
  }
  
  // ZLG: same as CAN device initialization
  return init_can_device();
}

/****************************************************************************/
// Channel configuration and startup CAN/CANFD
// Configures and starts the CAN/CANFD channel
// @return true if successful, false otherwise
/****************************************************************************/

bool start_can_channel(void) {
  if (get_can_backend() == kBackendSocketcan) {
    return true;  // SocketCAN doesn't need channel setup
  }
#if STARK_USE_ZLG
  if (!pVCI_InitCAN || !pVCI_StartCAN) {
    printf("[ERROR] ZLG library not loaded\n");
    return false;
  }
  
  ZCAN_INIT init;      // Baud rate configuration
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
  if (!pVCI_InitCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, &init)) {
    printf("[ERROR] Failed to initialize ZLG CAN channel\n");
    return false;
  }

  // Start CAN channel
  if (!pVCI_StartCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX)) {
    printf("[ERROR] Failed to start ZLG CAN channel\n");
    return false;
  }

  return true;
#else
  printf("[ERROR] ZLG not compiled\n");
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
                           uint32_t expected_can_id,
                           uint8_t expected_frames,
                           uint32_t *can_id_out,
                           uint8_t *data_out,
                           uintptr_t *data_len_out) -> int {
      (void)slave_id;
      return socketcan_recv_can(expected_can_id, expected_frames, can_id_out, data_out, data_len_out);
    });
#else
    printf("[ERROR] SocketCAN not compiled\n");
#endif
    return;
  }
#if STARK_USE_ZLG
  // CAN transmit callback
  set_can_tx_callback([](uint8_t slave_id,
                         uint32_t can_id,
                         const uint8_t *data,
                         uintptr_t data_len) -> int {
    (void)slave_id;
    if (!pVCI_Transmit) return -1;
    
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
    int result = pVCI_Transmit(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, &can_msg, 1);
    if (result != 1)
      printf("CAN Transmit result: %d\n", result);
    return result == 1 ? 0 : -1; // 0 indicates success
  });

  // CAN receive callback with multi-frame support
  set_can_rx_callback([](uint8_t slave_id,
                         uint32_t expected_can_id,
                         uint8_t expected_frames,
                         uint32_t *can_id_out,
                         uint8_t *data_out,
                         uintptr_t *data_len_out) -> int {
    (void)slave_id;
    if (!pVCI_Receive) return -1;
    
    int total_dlc = 0;
    int received_count = 0;
    uint8_t total_frames = 0;  // For multi-frame detection
    bool is_multi_frame = false;
    
    // Check if this is DFU mode (expected_can_id == 0)
    bool is_dfu_mode = (expected_can_id == 0);
    
    // Extract command from CAN ID to detect multi-frame commands
    uint8_t cmd = (expected_can_id >> 3) & 0x0F;
    bool is_multi_frame_cmd = (cmd == 0x0B || cmd == 0x0D);  // MultiRead or TouchSensorRead
    
    // Determine retry strategy:
    // - DFU mode: 200 attempts (for CRC verification)
    // - Multi-frame commands: 5 attempts
    // - Single frame: 2 attempts
    int max_attempts = is_dfu_mode ? 200 : (expected_frames > 1 || is_multi_frame_cmd ? 5 : 2);
    
    for (int attempt = 0; attempt < max_attempts; attempt++) {
      ZCAN_20_MSG can_data[RX_BUFF_SIZE];
      int len = pVCI_Receive(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, can_data, RX_BUFF_SIZE, RX_WAIT_TIME);
      
      if (len < 1) {
        int wait_ms = (attempt < 5) ? 2 : 5;
        usleep(wait_ms * 1000);
        continue;
      }
      
      // Process all frames in this batch
      for (int i = 0; i < len; i++) {
        ZCAN_20_MSG& frame = can_data[i];
        uint32_t can_id = frame.hdr.id & 0x1FFFFFFF;
        int can_dlc = frame.hdr.len;
        
        // Skip non-matching frames (unless DFU mode)
        if (!is_dfu_mode && expected_can_id != 0 && can_id != expected_can_id) {
          continue;
        }
        
        // Check for multi-frame protocol format
        if (is_multi_frame_cmd && can_dlc > 0) {
          uint8_t frame_header = frame.dat[0];
          
          if (cmd == 0x0B) {
            // MultiRead format: [addr, len|flag, data...]
            if (can_dlc >= 2) {
              uint8_t len_and_flag = frame.dat[1];
              bool is_last = (len_and_flag & 0x80) != 0;
              
              // Append entire frame (8 bytes)
              for (int j = 0; j < can_dlc; j++) {
                data_out[total_dlc++] = frame.dat[j];
              }
              received_count++;
              
              // Check if this is the last frame
              if (is_last) {
                *can_id_out = can_id;
                *data_len_out = total_dlc;
                return 0;
              }
              continue;
            }
          } else if (cmd == 0x0D) {
            // TouchSensorRead format: [total:4bit|seq:4bit, data...]
            uint8_t total = (frame_header >> 4) & 0x0F;
            uint8_t seq = frame_header & 0x0F;
            
            // Detect multi-frame data
            if (total > 0 && seq > 0) {
              if (!is_multi_frame) {
                is_multi_frame = true;
                total_frames = total;
              }
              
              // Append entire frame (8 bytes)
              for (int j = 0; j < can_dlc; j++) {
                data_out[total_dlc++] = frame.dat[j];
              }
              received_count++;
              
              // Check if all frames received
              if (received_count >= total_frames) {
                *can_id_out = can_id;
                *data_len_out = total_dlc;
                return 0;
              }
              continue;
            }
          }
        }
        
        // Standard single-frame data - return immediately
        for (int j = 0; j < can_dlc; j++) {
          data_out[total_dlc++] = frame.dat[j];
        }
        *can_id_out = can_id;
        *data_len_out = total_dlc;
        return 0;
      }
    }
    
    // Timeout - return whatever we have
    if (total_dlc > 0) {
      *can_id_out = expected_can_id;
      *data_len_out = total_dlc;
      return 0;
    }
    
    return -1;
  });
#else
  printf("[ERROR] ZLG not compiled\n");
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
                           uint32_t expected_can_id,
                           uint8_t expected_frames,
                           uint32_t *can_id_out,
                           uint8_t *data_out,
                           uintptr_t *data_len_out) -> int {
      (void)slave_id;
      return socketcan_recv_canfd(expected_can_id, expected_frames, can_id_out, data_out, data_len_out);
    });
#else
    printf("[ERROR] SocketCAN not compiled\n");
#endif
    return;
  }
#if STARK_USE_ZLG
  // CANFD transmit callback
  set_can_tx_callback([](uint8_t slave_id,
                         uint32_t canfd_id,
                         const uint8_t *data,
                         uintptr_t data_len) -> int {
    (void)slave_id;
    if (!pVCI_TransmitFD) return -1;
    
    // Construct CANFD transmit message
    ZCAN_FD_MSG canfd_msg;
    memset(&canfd_msg, 0, sizeof(ZCAN_FD_MSG));

    canfd_msg.hdr.inf.txm = 0; // 0 - normal transmit
    canfd_msg.hdr.inf.fmt = 1; // 0 - CAN, 1 - CANFD
    canfd_msg.hdr.inf.sdf = 0; // 0 - data frame
    canfd_msg.hdr.inf.sef = 1; // 0 - standard frame, 1 - extended frame

    canfd_msg.hdr.id = canfd_id;              // ID
    canfd_msg.hdr.chn = ZCANFD_CHANNEL_INDEX; // Channel
    canfd_msg.hdr.len = data_len;             // Data length

    // Fill data, maximum length 64
    for (uintptr_t i = 0; i < data_len && i < 64; ++i) {
      canfd_msg.dat[i] = data[i];
    }

    // Transmit CANFD frame
    int result = pVCI_TransmitFD(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, &canfd_msg, 1);
    if (result != 1)
      printf("CANFD Transmit result: %d\n", result);
    return result == 1 ? 0 : -1;
  });

  // CANFD receive callback
  set_can_rx_callback([](uint8_t slave_id,
                         uint32_t expected_can_id,
                         uint8_t expected_frames,
                         uint32_t *canfd_id_out,
                         uint8_t *data_out,
                         uintptr_t *data_len_out) -> int {
    (void)slave_id;
    (void)expected_frames;
    if (!pVCI_ReceiveFD) return -1;
    
    // CANFD CAN ID format: (slave_id << 16) | (master_id << 8) | payload_len
    uint8_t expected_slave_id = (expected_can_id >> 16) & 0xFF;
    uint8_t expected_master_id = (expected_can_id >> 8) & 0xFF;
    
    // Retry strategy (aligned with Rust ZQWL): normal=2
    int max_attempts = 2;
    
    for (int attempt = 0; attempt < max_attempts; attempt++) {
      ZCAN_FD_MSG canfd_data[RX_BUFF_SIZE];
      int len = pVCI_ReceiveFD(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, 
                               canfd_data, RX_BUFF_SIZE, RX_WAIT_TIME);
      
      if (len < 1) {
        int wait_ms = (attempt < 5) ? 2 : 5;
        usleep(wait_ms * 1000);
        continue;
      }
      
      // Find matching frame by slave_id and master_id
      for (int i = 0; i < len; i++) {
        ZCAN_FD_MSG& frame = canfd_data[i];
        uint32_t can_id = frame.hdr.id & 0x1FFFFFFF;
        
        // Match slave_id and master_id from CAN ID
        uint8_t resp_slave_id = (can_id >> 16) & 0xFF;
        uint8_t resp_master_id = (can_id >> 8) & 0xFF;
        
        if (resp_slave_id == expected_slave_id && resp_master_id == expected_master_id) {
          int canfd_dlc = (frame.hdr.len < 64) ? frame.hdr.len : 64;
          *canfd_id_out = can_id;
          *data_len_out = canfd_dlc;
          
          for (int j = 0; j < canfd_dlc; j++) {
            data_out[j] = frame.dat[j];
          }
          return 0;
        }
      }
    }
    
    // No matching frame found
    return -1;
  });
#else
  printf("[ERROR] ZLG not compiled\n");
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
  CanBackend backend = get_can_backend();
  
  if (backend == kBackendSocketcan) {
#if STARK_USE_SOCKETCAN
    if (g_socketcan_fd >= 0) {
      close(g_socketcan_fd);
      g_socketcan_fd = -1;
      g_socketcan_is_canfd = false;
    }
    printf("[SocketCAN] Resources cleaned up\n");
#endif
    return;
  }
  
  if (backend == kBackendZlg) {
#if STARK_USE_ZLG
    if (pVCI_ResetCAN) {
      pVCI_ResetCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX);
    }
    if (pVCI_CloseDevice) {
      pVCI_CloseDevice(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX);
    }
    printf("[ZLG] Resources cleaned up\n");
#endif
  }
}
