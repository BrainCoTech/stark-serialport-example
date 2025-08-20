#include <stdio.h>
#include "stark-sdk.h"
#include <unistd.h>
#include <signal.h>
#include <execinfo.h>

// 声明函数
void setup_modbus_callbacks();
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

  setup_modbus_callbacks(); // 设置读写回调

  init_cfg(STARK_PROTOCOL_TYPE_MODBUS, LOG_LEVEL_DEBUG); // 初始化配置
  auto cfg = auto_detect_modbus_revo2("/dev/ttyUSB0", true);
  if (cfg == NULL)
  {
    fprintf(stderr, "Failed to auto-detect Modbus device configuration.\n");
    return -1;
  }
  uint8_t slave_id = cfg->slave_id;
  auto handle = modbus_open(cfg->port_name, cfg->baudrate);
  free_device_config(cfg);
  
  get_device_info(handle, slave_id);
  return 0;
}

void get_device_info(DeviceHandler *handle, uint8_t slave_id)
{
  uint32_t baudrate = stark_get_rs485_baudrate(handle, slave_id);
  printf("Slave[%hhu] Baudrate: %d\n", slave_id, baudrate);
}

void setup_modbus_callbacks()
{
  set_modbus_read_holding_callback([](uint8_t slave_id, uint16_t register_address, uint16_t *data_out, uint16_t count) -> int
                                   {
                                     printf("Read holding registers: Slave ID: %d, Address: %d\n", slave_id, register_address);
                                     // FIXME: Simulate reading data
                                     for (uint16_t i = 0; i < count; ++i)
                                     {
                                       data_out[i] = i;
                                     }
                                     return 0; // Return 0 to indicate success
                                   });
  set_modbus_read_input_callback([](uint8_t slave_id, uint16_t register_address, uint16_t *data_out, uint16_t count) -> int
                                 {
                                   printf("Read input registers: Slave ID: %d, Address: %d\n", slave_id, register_address);
                                   // FIXME: Simulate reading data
                                   for (uint16_t i = 0; i < count; ++i)
                                   {
                                     data_out[i] = i;
                                   }
                                   return 0; // Return 0 to indicate success
                                 });
  set_modbus_write_callback([](uint8_t slave_id, uint16_t register_address, const uint16_t *data_in, uint16_t count) -> int
                            {
                              printf("Write holding registers: Slave ID: %d, Address: %d\n", slave_id, register_address);
                              // FIXME: Simulate writing data
                              for (uint16_t i = 0; i < count; ++i)
                              {
                                printf("Data[%d]: %d\n", i, data_in[i]);
                              }
                              return 0; // Return 0 to indicate success
                            });
}
