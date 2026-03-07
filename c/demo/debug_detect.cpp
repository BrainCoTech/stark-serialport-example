/**
 * @file debug_detect.cpp
 * @brief Auto-Detect Stark devices by scanning Modbus + Protobuf protocols
 *
 * Scans a port with multiple baudrate and slave ID combinations,
 * following the same priority order as the Rust detect_modbus function:
 *   1. 460800 + 0x7E/0x7F  (Revo2 / Revo1 Advanced)
 *   2. 115200 + 1/2         (Revo1 Basic/Touch)
 *   3. 460800 + 1/2         (fallback)
 *   4. 1000000 + 0x7E/0x7F  (high-speed)
 *   5. 2000000 + 0x7E/0x7F  (high-speed)
 *   6. 115200 + 0x7E/0x7F   (legacy)
 *   7. Protobuf 115200 + ID 10 (legacy RS485 Revo1 Basic)
 *
 * Protobuf detection sends raw bytes (no protobuf parsing needed),
 * and checks for the "BnCP" magic header in the response.
 *
 * Once a device is found, the baudrate is locked for that port.
 * Use --scan-all to continue scanning all combinations.
 *
 * Build: make debug_detect.exe
 * Run:   ./debug_detect.exe /dev/ttyUSB0
 *         ./debug_detect.exe /dev/ttyUSB0 --scan-all
 *         debug_detect.exe COM3              (Windows)
 */

#include "platform_compat.h"
#include "stark-sdk.h"
#include "stark_common.h"
#include <stdio.h>
#include <string.h>

#define msleep(ms) compat_sleep_ms(ms)

// Serial I/O headers for Protobuf raw detection
#ifndef _WIN32
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#ifdef __APPLE__
#include <IOKit/serial/ioss.h> // IOSSIOSPEED
#include <sys/ioctl.h>
#endif
#endif

// Scan config: baudrate + list of slave IDs to probe
struct ScanConfig {
  uint32_t baudrate;
  const uint8_t *slave_ids;
  int id_count;
};

static void print_device_info(CDeviceInfo *info, uint32_t baudrate,
                              uint8_t slave_id) {
  printf("  ----------------------------------\n");
  printf("  Baudrate:      %u\n", baudrate);
  printf("  Slave ID:      %d (0x%02X)\n", slave_id, slave_id);
  printf("  Serial Number: %s\n",
         info->serial_number ? info->serial_number : "N/A");
  printf("  Firmware:      %s\n",
         info->firmware_version ? info->firmware_version : "N/A");
  printf("  Hardware Type: %d (%s)\n", info->hardware_type,
         get_hardware_type_name_str(info->hardware_type));
  printf("  ----------------------------------\n\n");
}

// ============================================================
// Protobuf raw detection (no protobuf library needed)
// ============================================================

// Pre-built "get_motorboard_info" command for slave_id=10
// Protocol: BnCP + dst(0x0A) + src(0x01) + len + payload + CRC32
static const uint8_t PROTOBUF_CMD[] = {
    0x42, 0x6E, 0x43, 0x50, // "BnCP" magic
    0x0A,                   // dst: slave_id 10
    0x01,                   // src: APP (0x01)
    0x00, 0x04,             // payload length: 4 (big-endian)
    0x1A, 0x02, 0x08, 0x01, // payload: hand_info_req { req: true }
    0xCB, 0x05, 0x11, 0xE4, // CRC32 (big-endian)
};

// Check if buffer contains "BnCP" magic header
static bool has_bncp_magic(const uint8_t *buf, int len) {
  for (int i = 0; i <= len - 4; i++) {
    if (buf[i] == 0x42 && buf[i + 1] == 0x6E && buf[i + 2] == 0x43 &&
        buf[i + 3] == 0x50) {
      return true;
    }
  }
  return false;
}

#ifdef _WIN32
// ============================================================
// Windows serial implementation
// ============================================================

static HANDLE serial_open_raw_win(const char *port, uint32_t baudrate) {
  // Windows needs "\\.\COM3" format for COM ports >= COM10
  char full_port[64];
  if (strncmp(port, "\\\\.\\", 4) == 0 || strncmp(port, "COM", 3) != 0) {
    snprintf(full_port, sizeof(full_port), "%s", port);
  } else {
    snprintf(full_port, sizeof(full_port), "\\\\.\\%s", port);
  }

  HANDLE h = CreateFileA(full_port, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                         OPEN_EXISTING, 0, NULL);
  if (h == INVALID_HANDLE_VALUE)
    return h;

  // Configure serial port: 8N1, no flow control
  DCB dcb = {0};
  dcb.DCBlength = sizeof(dcb);
  if (!GetCommState(h, &dcb)) {
    CloseHandle(h);
    return INVALID_HANDLE_VALUE;
  }
  dcb.BaudRate = baudrate;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fBinary = TRUE;
  dcb.fParity = FALSE;
  dcb.fOutxCtsFlow = FALSE;
  dcb.fOutxDsrFlow = FALSE;
  dcb.fDtrControl = DTR_CONTROL_ENABLE;
  dcb.fRtsControl = RTS_CONTROL_ENABLE;
  dcb.fOutX = FALSE;
  dcb.fInX = FALSE;
  if (!SetCommState(h, &dcb)) {
    CloseHandle(h);
    return INVALID_HANDLE_VALUE;
  }

  // Set timeouts
  COMMTIMEOUTS timeouts = {0};
  timeouts.ReadIntervalTimeout = 50;
  timeouts.ReadTotalTimeoutConstant = 300;
  timeouts.ReadTotalTimeoutMultiplier = 10;
  timeouts.WriteTotalTimeoutConstant = 300;
  timeouts.WriteTotalTimeoutMultiplier = 10;
  SetCommTimeouts(h, &timeouts);

  // Flush buffers
  PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

  return h;
}

static int serial_read_timeout_win(HANDLE h, uint8_t *buf, int max_len,
                                   int timeout_ms) {
  COMMTIMEOUTS t = {0};
  t.ReadTotalTimeoutConstant = timeout_ms;
  t.ReadTotalTimeoutMultiplier = 0;
  SetCommTimeouts(h, &t);

  DWORD n = 0;
  if (!ReadFile(h, buf, max_len, &n, NULL))
    return 0;
  return (int)n;
}

static bool probe_protobuf(const char *port, uint32_t baudrate) {
  HANDLE h = serial_open_raw_win(port, baudrate);
  if (h == INVALID_HANDLE_VALUE)
    return false;

  PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
  msleep(50); // settle

  // Send command
  DWORD written = 0;
  if (!WriteFile(h, PROTOBUF_CMD, sizeof(PROTOBUF_CMD), &written, NULL) ||
      written != sizeof(PROTOBUF_CMD)) {
    CloseHandle(h);
    return false;
  }
  FlushFileBuffers(h); // Wait for transmission complete

  msleep(200); // Wait for device to process

  // Read response
  uint8_t resp[256];
  int n = serial_read_timeout_win(h, resp, sizeof(resp), 300);
  CloseHandle(h);

  return (n >= 4) && has_bncp_magic(resp, n);
}

#else
// ============================================================
// POSIX serial implementation (Linux / macOS)
// ============================================================

#ifndef __APPLE__
// Map baudrate value to termios speed constant (Linux only)
// macOS uses IOSSIOSPEED ioctl for arbitrary baud rates instead.
static speed_t get_termios_speed(uint32_t baudrate) {
  switch (baudrate) {
  case 9600:
    return B9600;
  case 19200:
    return B19200;
  case 38400:
    return B38400;
  case 57600:
    return B57600;
  case 115200:
    return B115200;
  case 230400:
    return B230400;
  case 460800:
    return B460800;
  case 500000:
    return B500000;
  case 576000:
    return B576000;
  case 921600:
    return B921600;
  case 1000000:
    return B1000000;
  case 1500000:
    return B1500000;
  case 2000000:
    return B2000000;
  default:
    return B115200;
  }
}
#endif

static int serial_open_raw(const char *port, uint32_t baudrate) {
  int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
    return -1;

  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd, &tty) != 0) {
    close(fd);
    return -1;
  }

#ifdef __APPLE__
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
#else
  speed_t spd = get_termios_speed(baudrate);
  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);
#endif

  // Raw mode: 8N1, no flow control
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP |
                   INLCR | IGNCR | ICRNL);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  tcflush(fd, TCIOFLUSH);
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    close(fd);
    return -1;
  }

#ifdef __APPLE__
  speed_t speed = (speed_t)baudrate;
  if (ioctl(fd, IOSSIOSPEED, &speed) < 0) {
    printf("  [WARN] IOSSIOSPEED failed for %u: %s\n", baudrate,
           strerror(errno));
  }
#endif

  return fd;
}

static int serial_read_timeout(int fd, uint8_t *buf, int max_len,
                               int timeout_ms) {
  fd_set readfds;
  struct timeval tv;
  FD_ZERO(&readfds);
  FD_SET(fd, &readfds);
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int ret = select(fd + 1, &readfds, NULL, NULL, &tv);
  if (ret > 0 && FD_ISSET(fd, &readfds)) {
    return read(fd, buf, max_len);
  }
  return 0;
}

/**
 * Probe for a Protobuf device on the given serial port.
 *
 * Sends a hardcoded "get_motorboard_info" command (slave_id=10)
 * and checks if the response starts with "BnCP" magic header.
 */
static bool probe_protobuf(const char *port, uint32_t baudrate) {
  int fd = serial_open_raw(port, baudrate);
  if (fd < 0)
    return false;

  tcflush(fd, TCIOFLUSH);
  msleep(50); // settle

  ssize_t written = write(fd, PROTOBUF_CMD, sizeof(PROTOBUF_CMD));
  if (written != sizeof(PROTOBUF_CMD)) {
    close(fd);
    return false;
  }
  tcdrain(fd);

  msleep(200); // Wait for device to process

  uint8_t resp[256];
  int n = serial_read_timeout(fd, resp, sizeof(resp), 300);
  close(fd);

  return (n >= 4) && has_bncp_magic(resp, n);
}

#endif // _WIN32

// ============================================================
// Main
// ============================================================

int main(int argc, char const *argv[]) {
  printf("=== Stark Auto-Detect (Modbus + Protobuf) ===\n\n");

  init_logging(LOG_LEVEL_INFO);

#ifdef _WIN32
  const char *port = "COM3";
#else
  const char *port = "/dev/ttyUSB0";
#endif
  bool scan_all = false;

  // Parse command line arguments
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--scan-all") == 0) {
      scan_all = true;
    } else {
      port = argv[i];
    }
  }

  // Priority-based scan order (matches Rust detect_modbus)
  const uint8_t revo2_ids[] = {0x7E, 0x7F};
  const uint8_t revo1_ids[] = {1, 2};

  const ScanConfig scan_configs[] = {
      {460800, revo2_ids, 2},  // Priority 1: Revo2 / Revo1 Advanced
      {115200, revo1_ids, 2},  // Priority 2: Revo1 Basic/Touch
      {460800, revo1_ids, 2},  // Priority 3: fallback
      {1000000, revo2_ids, 2}, // Priority 4: high-speed
      {2000000, revo2_ids, 2}, // Priority 5: high-speed
      {115200, revo2_ids, 2},  // Priority 6: legacy
  };
  const int config_count = sizeof(scan_configs) / sizeof(scan_configs[0]);

  printf("[INFO] Port: %s\n", port);
  printf("[INFO] Mode: %s\n", scan_all ? "scan-all" : "stop on first match");
  printf("[INFO] Scanning Modbus (%d configs) + Protobuf ...\n\n",
         config_count);

  int total_found = 0;
  uint32_t locked_baudrate = 0;
  uint8_t found_ids[32];
  int found_id_count = 0;

  // ---- Phase 1: Modbus scan ----
  printf("--- Phase 1: Modbus Scan ---\n");
  for (int c = 0; c < config_count; c++) {
    uint32_t baudrate = scan_configs[c].baudrate;

    // If a device was found, only continue with the same baudrate (same bus)
    if (locked_baudrate != 0 && baudrate != locked_baudrate) {
      continue;
    }

    // Stop early if not scan_all and already found
    if (total_found > 0 && !scan_all) {
      break;
    }

    printf("[SCAN] Trying baudrate %u ...\n", baudrate);

    DeviceHandler *handle = modbus_open(port, baudrate);
    if (handle == NULL) {
      printf("  [WARN] Failed to open port at %u, skipping\n", baudrate);
      break;
    }

    // Brief warm-up: let the FTDI USB driver and RS-485 transceiver
    // settle before the first probe. On a cold first open the kernel
    // driver may not have flushed its TX/RX FIFOs yet, causing the
    // first response to be missed and a false timeout.
    msleep(150); // 150ms warm-up

    for (int i = 0; i < scan_configs[c].id_count; i++) {
      uint8_t slave_id = scan_configs[c].slave_ids[i];

      // Skip if already found this slave ID
      bool already_found = false;
      for (int f = 0; f < found_id_count; f++) {
        if (found_ids[f] == slave_id) {
          already_found = true;
          break;
        }
      }
      if (already_found)
        continue;

      printf("  [PROBE] ID %d (0x%02X) @ %u ... ", slave_id, slave_id,
             baudrate);

      // Fast ping: try input register 3000
      uint16_t dummy_val = 0;
      bool ping_ok = stark_read_input_registers(handle, slave_id, 3000, 1,
                                                &dummy_val) == 0;

      if (!ping_ok) {
        printf("no response\n");
        continue;
      }

      printf("RESPONDED!\n");

      // Get full device info
      CDeviceInfo *info = stark_get_device_info(handle, slave_id);
      if (info != NULL) {
        printf("\n  ✅ Found Modbus device!\n");
        print_device_info(info, baudrate, slave_id);
        free_device_info(info);

        total_found++;
        locked_baudrate = baudrate;
        if (found_id_count < 32) {
          found_ids[found_id_count++] = slave_id;
        }

        if (!scan_all) {
          modbus_close(handle);
          goto done;
        }
      } else {
        printf("  [WARN] Ping OK but failed to get device info\n");
      }
    }

    modbus_close(handle);
  }

  // ---- Phase 2: Protobuf scan ----
  if (total_found == 0 || scan_all) {
    printf("\n--- Phase 2: Protobuf Scan ---\n");
    printf("[SCAN] Trying Protobuf @ 115200, slave_id=10 ...\n");
    printf("  [PROBE] Sending raw get_motorboard_info ... ");

    if (probe_protobuf(port, 115200)) {
      printf("RESPONDED! (BnCP magic detected)\n");
      printf("\n  ✅ Found Protobuf device!\n");
      printf("  ----------------------------------\n");
      printf("  Protocol:    Protobuf (legacy RS485)\n");
      printf("  Baudrate:    115200\n");
      printf("  Slave ID:    10 (0x0A)\n");
      printf("  Device Type: Revo1 Basic (Protobuf)\n");
      printf("  ----------------------------------\n\n");
      total_found++;
    } else {
      printf("no response\n");
    }
  }

done:
  printf("====================================\n");
  printf("[RESULT] Found %d device(s)\n", total_found);
  if (total_found > 0 && locked_baudrate > 0) {
    printf("[RESULT] Working Modbus baudrate: %u\n", locked_baudrate);
  }
  printf("\n=== Scan finished ===\n");
  return 0;
}
