#include <stdio.h>
#include "stark-sdk.h"
#include <unistd.h>
#include <signal.h>
#include <execinfo.h>

// 声明函数
void print_hex(unsigned char *data, int len);
void handler(int sig);
int modbus_operation_async(const uint8_t *values, int len, ModbusOperationResultCallback callback, void *user_data);
void get_device_info(DeviceHandler *handleint, uint8_t slave_id);

struct
{
  DeviceHandler *handle;
  uint8_t slave_id;
} set_modbus_operation_params;

int main(int argc, char const *argv[])
{
  signal(SIGSEGV, handler); // Install our handler for SIGSEGV (segmentation fault)
  signal(SIGABRT, handler); // Install our handler for SIGABRT (abort signal)

  init_cfg(STARK_PROTOCOL_TYPE_MODBUS, LOG_LEVEL_DEBUG); // 初始化配置
  auto cfg = auto_detect_modbus_revo2("/dev/ttyUSB0", true); // 替换为实际的串口名称, 传None会尝试自动检测
  if (cfg == NULL)
  {
    fprintf(stderr, "Failed to auto-detect Modbus device configuration.\n");
    return -1;
  }
  uint8_t slave_id = cfg->slave_id;
  auto handle = modbus_open(cfg->port_name, cfg->baudrate);
  free_device_config(cfg);
  
  get_device_info(handle, slave_id);

  while (true)
  {
    printf("Performing periodic operations...\n");
    // 模拟一些操作
    usleep(1000 * 1000); // 每秒执行一次
  }
  return 0;
}

// 线程执行函数
void *get_device_info_thread(void *arg)
{
  auto params = static_cast<decltype(set_modbus_operation_params) *>(arg);
  DeviceHandler *modbus_handle = params->handle;
  uint8_t slave_id = params->slave_id;

  uint32_t baudrate = stark_get_rs485_baudrate(modbus_handle, slave_id);
  printf("Slave[%hhu] Baudrate: %d\n", slave_id, baudrate);

  free(arg); // 释放参数内存
  pthread_exit(NULL);
  return NULL;
}

void get_device_info(DeviceHandler *handler, uint8_t slave_id)
{
  if (!handler)
  {
    fprintf(stderr, "Invalid Modbus handle.\n");
    return;
  }
  printf("Getting device info for slave ID: %hhu\n", slave_id);

  // 创建线程
  pthread_t thread_id;
  auto params = new decltype(set_modbus_operation_params);
  if (!params)
  {
    fprintf(stderr, "Failed to allocate memory for thread parameters.\n");
    return;
  }
  params->handle = handler;
  params->slave_id = slave_id;
  int result = pthread_create(&thread_id, NULL, get_device_info_thread, params);
  if (result != 0)
  {
    fprintf(stderr, "Failed to create thread: %d\n", result);
    return;
  }
  // 分离线程，让它在后台运行
  pthread_detach(thread_id);

  printf("Async get_device_info thread started.\n");
}

int modbus_operation_async(const uint8_t *values, int len, ModbusOperationResultCallback callback, void *user_data)
{
  if (len <= 0 || !values || !callback)
  {
    fprintf(stderr, "Invalid parameters for modbus_operation_async.\n");
    return -1;
  }
  printf("modbus_operation_async called, len=%d, data: ", len);
  print_hex((unsigned char *)values, len);

  // TODO: Implement the actual Modbus operation here
  // ...

  return 0;
}

// Function to print a byte array in hexadecimal
void print_hex(unsigned char *data, int len)
{
  for (int i = 0; i < len; i++)
  {
    printf("0x%02X", data[i]); // Print each byte as 2-digit hex
    if (i < len - 1)
    {
      printf(" "); // Add space between bytes, except for the last one
    }
  }
  printf("\r\n"); // Newline after the hex output
}

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
