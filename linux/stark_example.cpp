#include <stdio.h>
#include "stark-sdk.h"
#include <unistd.h>

// 声明函数
void get_touch_status(ModbusHandle *handle);
void get_device_info(ModbusHandle *handle);
void get_info(ModbusHandle *handle);

int main(int argc, char const *argv[])
{
  init_cfg(STARK_FIRMWARE_TYPE_V1_TOUCH, LOG_LEVEL_INFO);
  list_available_ports();

  // auto handle = modbus_open("COM1", 1, 115200);
  auto handle = modbus_open("/dev/tty.usbserial-D30JB26J", 1, 115200);
  // auto handle = modbus_open("/dev/ttyUSB0", 1, 115200);
  // auto handle1 = modbus_open("/dev/ttyUSB1", 2, 115200);

  get_device_info(handle);
  // get_device_info(handle1);

  // 启用全部触觉传感器
  modbus_enable_touch_sensor(handle, 0x1F);

  uint16_t positions_fist[] = {50, 50, 100, 100, 100, 100}; // 握拳
  uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};           // 张开

  useconds_t delay = 1000 * 1000; // 1000ms
  modbus_set_finger_positions(handle, positions_fist, 6);
  usleep(delay);
  modbus_set_finger_positions(handle, positions_open, 6);
  usleep(delay);

  auto finger_status = modbus_get_motor_status(handle);
  printf("Positions: %hu, %hu, %hu, %hu, %hu, %hu\n", finger_status->positions[0], finger_status->positions[1], finger_status->positions[2], finger_status->positions[3], finger_status->positions[4], finger_status->positions[5]);
  printf("Speeds: %hhd, %hhd, %hhd, %hhd, %hhd, %hhd\n", finger_status->speeds[0], finger_status->speeds[1], finger_status->speeds[2], finger_status->speeds[3], finger_status->speeds[4], finger_status->speeds[5]);
  printf("Currents: %hhd, %hhd, %hhd, %hhd, %hhd, %hhd\n", finger_status->currents[0], finger_status->currents[1], finger_status->currents[2], finger_status->currents[3], finger_status->currents[4], finger_status->currents[5]);
  printf("States: %hhu, %hhu, %hhu, %hhu, %hhu, %hhu\n", finger_status->states[0], finger_status->states[1], finger_status->states[2], finger_status->states[3], finger_status->states[4], finger_status->states[5]);
  free_motor_status_data(finger_status);

  get_touch_status(handle);

  // 循环控制手指位置，仅用于测试，delay时间过短会影响电机使用寿命
  while (1)
  {
    printf("set_positions loop start\n");
    modbus_set_finger_positions(handle, positions_fist, 6);
    usleep(delay);
    get_touch_status(handle); // 调用 get_touch_status 函数
    modbus_set_finger_positions(handle, positions_open, 6);
    usleep(delay);
  }

  return 0;
}

// 获取设备序列号、固件版本等信息
void get_device_info(ModbusHandle *handle)
{
  auto info = modbus_get_device_info(handle);
  if (info != NULL)
  {
    printf("SKU Type: %hhu, Serial Number: %s, Firmware Version: %s\n", (uint8_t)info->sku_type, info->serial_number, info->firmware_version);
    free_device_info(info);
  }
  else
  {
    printf("Error: Failed to get device info\n");
  }
}

// 获取触觉传感器状态，三维力数值、自接近、互接近电容值，以及传感器状态
void get_touch_status(ModbusHandle *handle)
{
  auto status = modbus_get_touch_status(handle);
  if (status != NULL)
  {
    auto data = status->data[1];
    printf("Touch Sensor Status At Index Finger:\n");
    printf("Normal Force 1: %hu\n", data.normal_force1);
    printf("Normal Force 2: %hu\n", data.normal_force2);
    printf("Normal Force 3: %hu\n", data.normal_force3);
    printf("Tangential Force 1: %hu\n", data.tangential_force1);
    printf("Tangential Force 2: %hu\n", data.tangential_force2);
    printf("Tangential Force 3: %hu\n", data.tangential_force3);
    printf("Tangential Direction 1: %hu\n", data.tangential_direction1);
    printf("Tangential Direction 2: %hu\n", data.tangential_direction2);
    printf("Tangential Direction 3: %hu\n", data.tangential_direction3);
    printf("Self Proximity 1: %u\n", data.self_proximity1);
    printf("Self Proximity 2: %u\n", data.self_proximity2);
    printf("Mutual Proximity: %u\n", data.mutual_proximity);
    printf("Status: %hu\n", data.status);

    // Free the allocated memory
    free_touch_status_data(status);
  }
  else
  {
    printf("Error: Failed to get touch sensor status\n");
  }
}

// 获取设备信息, 串口波特率, 从机地址, 电压, LED信息, 按键事件
void get_info(ModbusHandle *handle)
{
  auto baudrate = modbus_get_baudrate(handle);
  printf("Baudrate: %d\n", baudrate);
  auto slave_id = modbus_get_slave_id(handle);
  printf("Slave ID: %d\n", slave_id);
  // 触觉版 deprecated
  // auto force_level = modbus_get_force_level(handle);
  // printf("Force Level: %d\n", force_level);
  auto voltage = modbus_get_voltage(handle);
  printf("Voltage: %.2fV\n", voltage / 1000.0);
  auto led_info = modbus_get_led_info(handle);
  printf("LED Info: %hhu, %hhu\n", led_info->color, led_info->mode);
  free_led_info(led_info);
  auto button_event = modbus_get_button_event(handle);
  printf("Button Event: %d, %d, %hhu\n", button_event->timestamp, button_event->button_id, button_event->press_state);
  free_button_event(button_event);
}
