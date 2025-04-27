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

  init_cfg(STARK_FIRMWARE_TYPE_V1_TOUCH, STARK_PROTOCOL_TYPE_MODBUS, LOG_LEVEL_INFO);
  list_available_ports();

  // auto port_name = "COM1"; // Windows
  // auto port_name = "/dev/ttyUSB0"; // Linux
  // auto port_name = "/dev/ttyUSB1"; // Linux
  auto port_name = "/dev/tty.usbserial-D30JCEP2"; // Mac USB HUB
  // auto port_name = "/dev/tty.usbserial-FT9O53VF"; // Mac USB HUB
  uint32_t baudrate = 115200;
  uint8_t slave_id = 1;
  auto handle = modbus_open(port_name, baudrate, slave_id);

  get_device_info(handle, slave_id);

  // 启用全部触觉传感器
  modbus_enable_touch_sensor(handle, slave_id, 0x1F);
  usleep(1000 * 1000); // wait for touch sensor to be ready

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

  get_touch_status(handle, slave_id);

  // 循环控制手指位置，仅用于测试，delay时间过短会影响电机使用寿命
  while (0)
  {
    printf("set_positions loop start\n");
    modbus_set_finger_positions(handle, slave_id, positions_fist, 6);
    usleep(delay);
    get_touch_status(handle, slave_id); // 调用 get_touch_status 函数
    modbus_set_finger_positions(handle, slave_id, positions_open, 6);
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

// 获取触觉传感器状态，三维力数值、自接近、互接近电容值，以及传感器状态
void get_touch_status(ModbusHandle *handle, uint8_t slave_id)
{
  auto status = modbus_get_touch_status(handle, slave_id);
  if (status != NULL)
  {
    auto data = status->data[1];
    printf("Slave[%hhu] Touch Sensor Status At Index Finger:\n", slave_id);
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

  auto raw_data = modbus_get_touch_raw_data(handle, slave_id);
  if (raw_data != NULL)
  {
    printf("Slave[%hhu] Touch Raw Data:\n", slave_id);
    printf("Thumb: %u, %u, %u, %u, %u, %u, %u\n", raw_data->thumb[0], raw_data->thumb[1], raw_data->thumb[2], raw_data->thumb[3], raw_data->thumb[4], raw_data->thumb[5], raw_data->thumb[6]);
    printf("Index: %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u\n", raw_data->index[0], raw_data->index[1], raw_data->index[2], raw_data->index[3], raw_data->index[4], raw_data->index[5], raw_data->index[6], raw_data->index[7], raw_data->index[8], raw_data->index[9], raw_data->index[10]);
    printf("Middle: %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u\n", raw_data->middle[0], raw_data->middle[1], raw_data->middle[2], raw_data->middle[3], raw_data->middle[4], raw_data->middle[5], raw_data->middle[6], raw_data->middle[7], raw_data->middle[8], raw_data->middle[9], raw_data->middle[10]);
    printf("Ring: %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u\n", raw_data->ring[0], raw_data->ring[1], raw_data->ring[2], raw_data->ring[3], raw_data->ring[4], raw_data->ring[5], raw_data->ring[6], raw_data->ring[7], raw_data->ring[8], raw_data->ring[9], raw_data->ring[10]);
    printf("Pinky: %u, %u, %u, %u, %u, %u, %u\n", raw_data->pinky[0], raw_data->pinky[1], raw_data->pinky[2], raw_data->pinky[3], raw_data->pinky[4], raw_data->pinky[5], raw_data->pinky[6]);
    free_touch_raw_data(raw_data);
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
