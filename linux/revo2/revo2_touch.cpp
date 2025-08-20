
#include <stdio.h>
#include "stark-sdk.h"
#include <unistd.h>
#include <signal.h>
#include <execinfo.h>

// 声明函数
void get_touch_status(DeviceHandler *handle, uint8_t slave_id);
void get_device_info(DeviceHandler *handleint, uint8_t slave_id);
void get_info(DeviceHandler *handle, uint8_t slave_id);

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

  // uint8_t slave_id = 0x7f;
  // auto handle = modbus_open("/dev/ttyUSB0", 460800);

  get_device_info(handle, slave_id);

  // 设置手指控制参数的单位模式
  stark_set_finger_unit_mode(handle, slave_id, FINGER_UNIT_MODE_NORMALIZED);
  // stark_set_finger_unit_mode(handle, slave_id, FINGER_UNIT_MODE_PHYSICAL);

  auto mode = stark_get_finger_unit_mode(handle, slave_id);
  if (mode == FINGER_UNIT_MODE_NORMALIZED)
  {
    printf("Finger unit mode: Normalized\n");
  }
  else if (mode == FINGER_UNIT_MODE_PHYSICAL)
  {
    printf("Finger unit mode: Physical\n");
  }
  else
  {
    printf("Finger unit mode: Unknown\n");
  }

  // 设置手指参数，最大角度，最小角度，最大速度，最大电流，保护电流, 各个手指参数范围详见文档
  auto finger_id = STARK_FINGER_ID_MIDDLE;
  // stark_set_finger_min_position(handle, slave_id, finger_id, 0);
  // auto min_position = stark_get_finger_min_position(handle, slave_id, finger_id);
  // printf("Finger[%hhu] min position: %hu\n", finger_id, min_position);

  // stark_set_finger_max_position(handle, slave_id, finger_id, 80);
  // auto max_position = stark_get_finger_max_position(handle, slave_id, finger_id);
  // printf("Finger[%hhu] max position: %hu\n", finger_id, max_position);

  // stark_set_finger_max_speed(handle, slave_id, finger_id, 130);
  // auto max_speed = stark_get_finger_max_speed(handle, slave_id, finger_id);
  // printf("Finger[%hhu] max speed: %hu\n", finger_id, max_speed);

  // stark_set_finger_max_current(handle, slave_id, finger_id, 1000);
  // auto max_current = stark_get_finger_max_current(handle, slave_id, finger_id);
  // printf("Finger[%hhu] max current: %hu\n", finger_id, max_current);

  // stark_set_finger_protected_current(handle, slave_id, finger_id, 500);
  // auto protected_current = stark_get_finger_protected_current(handle, slave_id, finger_id);
  // printf("Finger[%hhu] protect current: %hu\n", finger_id, protected_current);

  useconds_t delay = 1000 * 1000; // 1000ms

  // 单个手指，按速度/电流/PWM控制
  // 其中符号表示方向，正表示为握紧方向，负表示为松开方向
  stark_set_finger_speed(handle, slave_id, finger_id, 500);    // -1000 ~ 1000
  usleep(delay);                                                // 等待手指到达目标位置
  stark_set_finger_current(handle, slave_id, finger_id, -300); // -1000 ~ 1000
  usleep(delay);                                                // 等待手指到达目标位置
  stark_set_finger_pwm(handle, slave_id, finger_id, 700);      // -1000 ~ 1000
  usleep(delay);                                                // 等待手指到达目标位置

  // 多个手指，按速度/电流/PWM控制
  // 其中符号表示方向，正表示为握紧方向，负表示为松开方向
  int16_t speeds[6] = {100, 100, 500, 500, 500, 500};
  stark_set_finger_speeds(handle, slave_id, speeds, 6);
  usleep(delay); // 等待手指到达目标位置
  int16_t currents[6] = {-300, -300, -300, -300, -300, -300};
  stark_set_finger_currents(handle, slave_id, currents, 6);
  usleep(delay); // 等待手指到达目标位置
  int16_t pwms[6] = {100, 100, 700, 700, 700, 700};
  stark_set_finger_pwms(handle, slave_id, pwms, 6);
  usleep(delay); // 等待手指到达目标位置

  // 单个手指，按位置+速度/期望时间，无符号
  stark_set_finger_position_with_millis(handle, slave_id, finger_id, 1000, 1000);
  usleep(delay); // 等待手指到达目标位置
  stark_set_finger_position_with_speed(handle, slave_id, finger_id, 1, 50);
  usleep(delay); // 等待手指到达目标位置

  // 多个手指，按位置+速度/期望时间，无符号
  uint16_t positions[6] = {300, 300, 500, 500, 500, 500};
  uint16_t durations[6] = {300, 300, 300, 300, 300, 300};
  stark_set_finger_positions_and_durations(handle, slave_id, positions, durations, 6);
  usleep(delay); // 等待手指到达目标位置

  uint16_t positions2[6] = {30, 30, 100, 100, 100, 100};
  uint16_t speeds2[6] = {500, 500, 500, 500, 500, 500};
  stark_set_finger_positions_and_speeds(handle, slave_id, positions2, speeds2, 6);
  usleep(delay); // 等待手指到达目标位置

  auto finger_status = stark_get_motor_status(handle, slave_id);
  if (finger_status != NULL)
  {
    printf("Positions: %hu, %hu, %hu, %hu, %hu, %hu\n", finger_status->positions[0], finger_status->positions[1], finger_status->positions[2], finger_status->positions[3], finger_status->positions[4], finger_status->positions[5]);
    printf("Speeds: %hd, %hd, %hd, %hd, %hd, %hd\n", finger_status->speeds[0], finger_status->speeds[1], finger_status->speeds[2], finger_status->speeds[3], finger_status->speeds[4], finger_status->speeds[5]);
    printf("Currents: %hd, %hd, %hd, %hd, %hd, %hd\n", finger_status->currents[0], finger_status->currents[1], finger_status->currents[2], finger_status->currents[3], finger_status->currents[4], finger_status->currents[5]);
    printf("States: %hhu, %hhu, %hhu, %hhu, %hhu, %hhu\n", finger_status->states[0], finger_status->states[1], finger_status->states[2], finger_status->states[3], finger_status->states[4], finger_status->states[5]);
    free_motor_status_data(finger_status);
  }

  return 0;
}

// 获取设备序列号、固件版本等信息
void get_device_info(DeviceHandler *handle, uint8_t slave_id)
{
  auto info = stark_get_device_info(handle, slave_id);
  if (info != NULL)
  {
    printf("Slave[%hhu] SKU Type: %hhu, Serial Number: %s, Firmware Version: %s\n", slave_id, (uint8_t)info->sku_type, info->serial_number, info->firmware_version);
    if (info->hardware_type == STARK_HARDWARE_TYPE_REVO1_TOUCH || info->hardware_type == STARK_HARDWARE_TYPE_REVO2_TOUCH)
    {
      // 启用全部触觉传感器
      stark_enable_touch_sensor(handle, slave_id, 0x1F);
      usleep(1000 * 1000); // wait for touch sensor to be ready
    }
    free_device_info(info);
  }
}

// 获取设备信息, 波特率, LED信息, 按键事件
void get_info(DeviceHandler *handle, uint8_t slave_id)
{
  // RS485串口波特率
  auto baudrate = stark_get_rs485_baudrate(handle, slave_id);
  printf("Slave[%hhu] Baudrate: %d\n", slave_id, baudrate);

  // CANFD波特率
  auto canfd_baudrate = stark_get_canfd_baudrate(handle, slave_id);
  printf("Slave[%hhu] CANFD Baudrate: %d\n", slave_id, canfd_baudrate);

  auto led_info = stark_get_led_info(handle, slave_id);
  if (led_info != NULL)
  {
    printf("Slave[%hhu] LED Info: %hhu, %hhu\n", slave_id, led_info->mode, led_info->color);
    free_led_info(led_info);
  }

  auto button_event = stark_get_button_event(handle, slave_id);
  if (button_event != NULL)
  {
    printf("Slave[%hhu] Button Event: %d, %d, %hhu\n", slave_id, button_event->timestamp, button_event->button_id, button_event->press_state);
    free_button_event(button_event);
  }
}

// 获取触觉传感器状态，三维力数值、接近值，以及传感器状态
void get_touch_status(DeviceHandler *handle, uint8_t slave_id)
{
  auto status = stark_get_touch_status(handle, slave_id);
  if (status != NULL)
  {
    auto data = status->items[1];
    printf("Slave[%hhu] Touch Sensor Status At Index Finger:\n", slave_id);
    printf("Normal Force: %hu\n", data.normal_force1);
    printf("Tangential Force: %hu\n", data.tangential_force1);
    printf("Tangential Direction: %hu\n", data.tangential_direction1);
    printf("Proximity: %u\n", data.self_proximity1);
    printf("Status: %hu\n", data.status);

    // Free the allocated memory
    free_touch_finger_data(status);
  }
  else
  {
    printf("Error: Failed to get touch sensor status\n");
  }

  auto raw_data = stark_get_touch_raw_data(handle, slave_id);
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
