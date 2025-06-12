#include <stdio.h>
#include <unistd.h>
#include "stark-sdk.h"

// 声明函数
void get_device_info(ModbusHandle *handleint, uint8_t slave_id);
void get_info(ModbusHandle *handle, uint8_t slave_id);

int main(int argc, char const *argv[])
{
  init_cfg(STARK_FIRMWARE_TYPE_V2_BASIC, STARK_PROTOCOL_TYPE_MODBUS, LOG_LEVEL_INFO, 1024);
  list_available_ports();

  // auto port_name = "COM1"; // Windows
  // auto port_name = "/dev/ttyUSB0"; // Linux
  auto port_name = "/dev/tty.usbserial-D30JAQW9"; // Mac USB HUB
  uint32_t baudrate = 460800;
  uint8_t slave_id_1 = 0x7e; // 左手默认ID为0x7e，右手默认ID为0x7f
  uint8_t slave_id_2 = 0x7f;
  auto handle = modbus_open(port_name, baudrate);

  get_device_info(handle, slave_id_1);
  get_device_info(handle, slave_id_2);

  uint16_t positions_fist[] = {50, 50, 100, 100, 100, 100}; // 握拳
  uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};           // 张开

  useconds_t delay = 1000 * 1000; // 1000ms
  modbus_set_finger_positions(handle, slave_id_1, positions_fist, 6);
  modbus_set_finger_positions(handle, slave_id_2, positions_fist, 6);
  usleep(delay);
  modbus_set_finger_positions(handle, slave_id_1, positions_open, 6);
  modbus_set_finger_positions(handle, slave_id_2, positions_open, 6);
  usleep(delay);

  auto finger_status = modbus_get_motor_status(handle, slave_id_1);
  printf("Positions: %hu, %hu, %hu, %hu, %hu, %hu\n", finger_status->positions[0], finger_status->positions[1], finger_status->positions[2], finger_status->positions[3], finger_status->positions[4], finger_status->positions[5]);
  printf("Speeds: %hhd, %hhd, %hhd, %hhd, %hhd, %hhd\n", finger_status->speeds[0], finger_status->speeds[1], finger_status->speeds[2], finger_status->speeds[3], finger_status->speeds[4], finger_status->speeds[5]);
  printf("Currents: %hhd, %hhd, %hhd, %hhd, %hhd, %hhd\n", finger_status->currents[0], finger_status->currents[1], finger_status->currents[2], finger_status->currents[3], finger_status->currents[4], finger_status->currents[5]);
  printf("States: %hhu, %hhu, %hhu, %hhu, %hhu, %hhu\n", finger_status->states[0], finger_status->states[1], finger_status->states[2], finger_status->states[3], finger_status->states[4], finger_status->states[5]);
  free_motor_status_data(finger_status);

  // 循环控制手指位置，仅用于测试，delay时间过短会影响电机使用寿命
  while (0)
  {
    printf("set_positions loop start\n");
    modbus_set_finger_positions(handle, slave_id_1, positions_fist, 6);
    modbus_set_finger_positions(handle, slave_id_2, positions_fist, 6);
    usleep(delay);
    modbus_set_finger_positions(handle, slave_id_1, positions_open, 6);
    modbus_set_finger_positions(handle, slave_id_2, positions_open, 6);
    usleep(delay);
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
  auto led_info = modbus_get_led_info(handle, slave_id);
  printf("Slave[%hhu] LED Info: %hhu, %hhu\n", slave_id, led_info->mode, led_info->color);
  free_led_info(led_info);
}
