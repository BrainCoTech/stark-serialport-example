#include <stdio.h>
#include "stark-sdk.h"
#include <unistd.h>
#include <signal.h>
#include <execinfo.h>

// 声明函数
void setup_canfd_callbacks();
void get_device_info(DeviceHandler *handleint, uint8_t slave_id);

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

  setup_canfd_callbacks(); // 设置读写回调

  init_cfg(STARK_PROTOCOL_TYPE_CAN_FD, LOG_LEVEL_INFO);
  const int MASTER_ID = 1;
  auto handle = canfd_init(MASTER_ID); // 这里不打开设备，只是初始化一个句柄
  get_device_info(handle, 0x7f);
  return 0;
}

void setup_canfd_callbacks()
{
  // 需要由接入方实现，开启CAN FD设备，和以下读写回调
  set_can_tx_callback([](uint8_t slave_id,
                         uint32_t can_id,
                         const uint8_t *data,
                         uintptr_t data_len) -> int
                      {
                        printf("CAN FD Send: Slave ID: %d, CAN ID: %d, Data Length: %zu\n", slave_id, can_id, data_len);
                        // TODO: sending data
                        return 0; // Return 0 to indicate success
                      });
  set_can_rx_callback([](uint8_t slave_id,
                         uint32_t *can_id_out,
                         uint8_t *data_out,
                         uintptr_t *data_len_out) -> int
                      {
                        printf("CAN FD Read: Slave ID: %d\n", slave_id);
                        // TODO: Read data

                        // Simulate read data returning
                        *can_id_out = 0x123; // Example CAN ID
                        *data_len_out = 64;  // Example data length
                        for (uintptr_t i = 0; i < *data_len_out; ++i)
                        {
                          data_out[i] = i;
                        }
                        return 0; // Return 0 to indicate success
                      });
}

void get_device_info(DeviceHandler *handle, uint8_t slave_id)
{
  DeviceInfo *info = stark_get_device_info(handle, slave_id);
  if (info != NULL)
  {
    printf("Slave[%hhu] Serial Number: %s\n", slave_id, info->serial_number);
    free_device_info(info);
  }
}
