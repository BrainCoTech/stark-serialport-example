#include <stdio.h>
#include "stark-sdk.h"
#include <unistd.h>
#include <signal.h>
#include <execinfo.h>

// 声明函数
void get_touch_status(ModbusHandle *handle, uint8_t slave_id);
void get_device_info(ModbusHandle *handleint, uint8_t slave_id);
void get_info(ModbusHandle *handle, uint8_t slave_id);

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

  init_cfg(STARK_FIRMWARE_TYPE_V1_STANDARD, STARK_PROTOCOL_TYPE_MODBUS, LOG_LEVEL_INFO);
  list_available_ports();

  // auto port_name = "COM1"; // Windows
  // auto port_name = "/dev/ttyUSB0"; // Linux
  // auto port_name = "/dev/ttyUSB1"; // Linux
  auto port_name = "/dev/tty.usbserial-D30JCEP2"; // Mac USB HUB
  // auto port_name = "/dev/tty.usbserial-FT9O53VF"; // Mac USB HUB
  uint32_t baudrate = 115200;
  uint8_t slave_id = 1;
  auto handle = modbus_open(port_name, baudrate);

  get_device_info(handle, slave_id);

  uint16_t positions_fist[] = {50, 50, 100, 100, 100, 100}; // 握拳
  uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};           // 张开

  useconds_t delay = 1000 * 1000; // 1000ms
  // modbus_set_finger_position(handle, slave_id, STARK_FINGER_ID_PINKY, 100);
  // usleep(delay);
  modbus_set_finger_positions(handle, slave_id, positions_fist, 6);
  usleep(delay);
  modbus_set_finger_positions(handle, slave_id, positions_open, 6);
  usleep(delay);

  auto finger_status = modbus_get_motor_status(handle, slave_id);
  if (finger_status != NULL)
  {
    printf("Positions: %hu, %hu, %hu, %hu, %hu, %hu\n", finger_status->positions[0], finger_status->positions[1], finger_status->positions[2], finger_status->positions[3], finger_status->positions[4], finger_status->positions[5]);
    printf("Speeds: %hhd, %hhd, %hhd, %hhd, %hhd, %hhd\n", finger_status->speeds[0], finger_status->speeds[1], finger_status->speeds[2], finger_status->speeds[3], finger_status->speeds[4], finger_status->speeds[5]);
    printf("Currents: %hhd, %hhd, %hhd, %hhd, %hhd, %hhd\n", finger_status->currents[0], finger_status->currents[1], finger_status->currents[2], finger_status->currents[3], finger_status->currents[4], finger_status->currents[5]);
    printf("States: %hhu, %hhu, %hhu, %hhu, %hhu, %hhu\n", finger_status->states[0], finger_status->states[1], finger_status->states[2], finger_status->states[3], finger_status->states[4], finger_status->states[5]);
    free_motor_status_data(finger_status);
  }

  return 0;
}

// 获取设备序列号、固件版本等信息
void get_device_info(ModbusHandle *handle, uint8_t slave_id)
{
  auto info = modbus_get_device_info(handle, slave_id);
  if (info != NULL)
  {
    printf("Slave[%hhu] SKU Type: %hhu, Serial Number: %s, Firmware Version: %s\n", slave_id, (uint8_t)info->sku_type, info->serial_number, info->firmware_version);
    free_device_info(info);
  }
  else
  {
    printf("Error: Failed to get device info\n");
  }
}

// 获取设备信息, 串口波特率, 从机地址, 电压, LED信息, 按键事件
void get_info(ModbusHandle *handle, uint8_t slave_id)
{
  auto baudrate = modbus_get_rs485_baudrate(handle, slave_id);
  printf("Slave[%hhu] Baudrate: %d\n", slave_id, baudrate);
  // 触觉版 deprecated
  // auto force_level = modbus_get_force_level(handle, slave_id);
  // printf("Slave[%hhu] Force Level: %d\n", slave_id, force_level);
  auto voltage = modbus_get_voltage(handle, slave_id);
  printf("Slave[%hhu] Voltage: %.2fV\n", slave_id, voltage / 1000.0);

  auto led_info = modbus_get_led_info(handle, slave_id);
  if (led_info != NULL)
  {
    printf("Slave[%hhu] LED Info: %hhu, %hhu\n", slave_id, led_info->mode, led_info->color);
    free_led_info(led_info);
  }

  auto button_event = modbus_get_button_event(handle, slave_id);
  if (button_event != NULL)
  {
    printf("Slave[%hhu] Button Event: %d, %d, %hhu\n", slave_id, button_event->timestamp, button_event->button_id, button_event->press_state);
    free_button_event(button_event);
  }
}
