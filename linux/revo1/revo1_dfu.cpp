#include <stdio.h>
#include "stark-sdk.h"
#include <unistd.h>
#include <signal.h>
#include <execinfo.h>

// 声明函数
// void get_device_info(DeviceHandler *handleint, uint8_t slave_id);

void handler(int sig)
{
  void *array[10];
  size_t size;

  // 获取堆栈帧

  size = backtrace(array, 10);

  // 打印所有堆栈帧到 stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

int main(int argc, char const *argv[])
{
  signal(SIGSEGV, handler); // Install our handler for SIGSEGV (segmentation fault)
  signal(SIGABRT, handler); // Install our handler for SIGABRT (abort signal)

  auto cfg = auto_detect_modbus_revo1(NULL, true);
  if (cfg == NULL)
  {
    fprintf(stderr, "Failed to auto-detect Modbus device configuration.\n");
    return -1;
  }

  auto handle = modbus_open(cfg->port_name, cfg->baudrate);
  uint8_t slave_id = cfg->slave_id;
  if (cfg != NULL) free_device_config(cfg);

  set_dfu_state_callback([](uint8_t slave_id, uint8_t state){ 
    printf("DFU State: %hhu\n", state); 
    if (state == 4) {
      printf("DFU finished\n");
      exit(0);
    }
  });
  set_dfu_progress_callback([](uint8_t slave_id, float progress){ printf("DFU Progress: %.2f%%\n", progress * 100); });
  start_dfu(handle, slave_id, "ota_bin/modbus/FW_MotorController_Release_SecureOTA_modbus_0.1.7.ota", 5);

  printf("Revo1 Modbus DFU, Waiting for DFU to complete...\n");
  useconds_t delay = 60 * 1000 * 1000; // 60s, wait for DFU to complete
  usleep(delay);
}
