#include "stark-sdk.h"
#include <execinfo.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

// 声明函数
void get_touch_status(DeviceHandler *handle, uint8_t slave_id);
void* motor_pdo_thread_func(void *arg);
void* touch_pdo_thread_func(void *arg);
void handler(int sig) {
  void *array[10];
  size_t size;

  // 获取堆栈帧
  size = backtrace(array, 10);

  // 打印所有堆栈帧到 stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

DeviceHandler *device_handle;
uint16_t slave_pos = 0; // 设备位置
bool is_touch_hand = false;

void start_pdo_thread() {
  // 创建线程
  pthread_t thread_id;
  int result = pthread_create(&thread_id, NULL, motor_pdo_thread_func, NULL);
  if (result != 0) {
    fprintf(stderr, "Failed to create thread: %d\n", result);
    return;
  }
  // 分离线程，让它在后台运行
  pthread_detach(thread_id);

  printf("Motor thread started.\n");
}

void start_pdo_thread_touch() {
  // 创建线程
  pthread_t thread_id;
  int result = pthread_create(&thread_id, NULL, touch_pdo_thread_func, NULL);
  if (result != 0) {
    fprintf(stderr, "Failed to create thread: %d\n", result);
    return;
  }
  // 分离线程，让它在后台运行
  pthread_detach(thread_id);

  printf("Touch thread started.\n");
}

int main(int argc, char const *argv[]) {
  printf("main\n");
  signal(SIGSEGV, handler); // Install our handler for SIGSEGV (segmentation fault)
  signal(SIGABRT, handler); // Install our handler for SIGABRT (abort signal)

  init_cfg(STARK_PROTOCOL_TYPE_ETHER_CAT, LOG_LEVEL_INFO);
  printf("ethercat_open_master...\n");
  device_handle = ethercat_open_master(0);
  ethercat_setup_sdo(device_handle, slave_pos);
  printf("setup_sdo done\n");

  uint8_t slave_pos_i = (uint8_t)slave_pos;
  auto info = stark_get_device_info(device_handle, slave_pos_i);
  if (info != NULL) {
    is_touch_hand = is_touch_hand_by_sn(info->serial_number);
    printf("is_touch_hand: %d\n", is_touch_hand);
    printf(
        "Slave[%hu] SKU Type: %hhu, Serial Number: %s, Firmware Version: %s\n",
        slave_pos, (uint8_t)info->sku_type, info->serial_number,
        info->firmware_version);
    free_device_info(info);
  } else {
    printf("stark_get_device_info empty\n");
  }

  // 设置手指控制参数的单位模式
  stark_set_finger_unit_mode(device_handle, slave_pos,
                             FINGER_UNIT_MODE_NORMALIZED);
  // stark_set_finger_unit_mode(device_handle, slave_pos,
  // FINGER_UNIT_MODE_PHYSICAL);

  // ethercat_start_dfu(device_handle, slave_pos, ETHER_CAT_FOE_TYPE_CONTROL,
  // "firmware.bin");

  // 设置手指保护电流
  auto finger_id = STARK_FINGER_ID_MIDDLE;
  // stark_set_finger_Ced_current(device_handle, slave_pos, finger_id, 500);
  auto protected_current =
      stark_get_finger_protected_current(device_handle, slave_pos, finger_id);
  printf("Finger[%hhu] protect current: %hu\n", finger_id, protected_current);

  // usleep(1 * 1000 * 1000); // wait test
  const uint16_t slave_positions[] = {slave_pos}; // 设备位置数组
  printf("start_loop...\n");
  ethercat_reserve_master(device_handle);

  ethercat_start_loop(device_handle, slave_positions, 1, 0, 500000, 0, 0, 0); // 启动PDO循环Thread，不启用DC同步
  printf("start_loop done\n");

  start_pdo_thread();
  if (is_touch_hand) {
    start_pdo_thread_touch();
  }
  usleep(500 * 1000 * 1000); // wait test
  printf("test done\n");
  ethercat_stop_loop(device_handle);
  printf("stop_loop done\n");
  ethercat_close(device_handle);
  return 0;
}

void* touch_pdo_thread_func(void *arg) {
  printf("Touch thread started.\n");

  while (1) {
    get_touch_status(device_handle, slave_pos);
    usleep(500); // 0.5ms
  }

  pthread_exit(NULL);
  return NULL;
}

// 控制/获取马达状态的线程
void* motor_pdo_thread_func(void *arg) {
  printf("Motor thread started.\n");
  useconds_t delay = 1000 * 1000; // 1000ms

  // 单个手指，按速度/电流/PWM控制
  // 其中符号表示方向，正表示为握紧方向，负表示为松开方向
  auto finger_id = STARK_FINGER_ID_RING;
  stark_set_finger_speed(device_handle, slave_pos, finger_id, 500); // -1000 ~ 1000
  usleep(delay);               // 等待手指到达目标位置
  stark_set_finger_current(device_handle, slave_pos, finger_id, -300); // -1000 ~ 1000
  usleep(delay);                  // 等待手指到达目标位置
  stark_set_finger_pwm(device_handle, slave_pos, finger_id, 700); // -1000 ~ 1000
  usleep(delay);             // 等待手指到达目标位置

  // 多个手指，按速度/电流/PWM控制
  // 其中符号表示方向，正表示为握紧方向，负表示为松开方向
  int16_t speeds[6] = {500, 500, 500, 500, 500, 500};
  stark_set_finger_speeds(device_handle, slave_pos, speeds, 6);
  usleep(delay); // 等待手指到达目标位置
  int16_t currents[6] = {-300, -300, -300, -300, -300, -300};
  stark_set_finger_currents(device_handle, slave_pos, currents, 6);
  usleep(delay); // 等待手指到达目标位置
  int16_t pwms[6] = {700, 700, 700, 700, 700, 700};
  stark_set_finger_pwms(device_handle, slave_pos, pwms, 6);
  usleep(delay); // 等待手指到达目标位置

  // 单个手指，按位置+速度/期望时间，无符号
  stark_set_finger_position_with_millis(device_handle, slave_pos, finger_id,
                                        1000, 1000);
  usleep(delay); // 等待手指到达目标位置
  stark_set_finger_position_with_speed(device_handle, slave_pos, finger_id, 1,
                                       50);
  usleep(delay); // 等待手指到达目标位置

  // 多个手指，按位置+速度/期望时间，无符号
  uint16_t positions[6] = {500, 500, 500, 500, 500, 500};
  uint16_t durations[6] = {300, 300, 300, 300, 300, 300};
  stark_set_finger_positions_and_durations(device_handle, slave_pos, positions,
                                           durations, 6);
  usleep(delay); // 等待手指到达目标位置

  uint16_t positions2[6] = {100, 100, 100, 100, 100, 100};
  uint16_t speeds2[6] = {500, 500, 500, 500, 500, 500};
  stark_set_finger_positions_and_speeds(device_handle, slave_pos, positions2,
                                        speeds2, 6);
  usleep(delay); // 等待手指到达目标位置

  while (1) {
    auto finger_status = stark_get_motor_status(device_handle, slave_pos);
    if (finger_status != NULL) {
      printf("Positions: %hu, %hu, %hu, %hu, %hu, %hu\n",
             finger_status->positions[0], finger_status->positions[1],
             finger_status->positions[2], finger_status->positions[3],
             finger_status->positions[4], finger_status->positions[5]);
      printf("Speeds: %hd, %hd, %hd, %hd, %hd, %hd\n", finger_status->speeds[0],
             finger_status->speeds[1], finger_status->speeds[2],
             finger_status->speeds[3], finger_status->speeds[4],
             finger_status->speeds[5]);
      printf("Currents: %hd, %hd, %hd, %hd, %hd, %hd\n",
             finger_status->currents[0], finger_status->currents[1],
             finger_status->currents[2], finger_status->currents[3],
             finger_status->currents[4], finger_status->currents[5]);
      printf("States: %hhu, %hhu, %hhu, %hhu, %hhu, %hhu\n",
             finger_status->states[0], finger_status->states[1],
             finger_status->states[2], finger_status->states[3],
             finger_status->states[4], finger_status->states[5]);
      free_motor_status_data(finger_status);
      usleep(500); // 0.5ms
    }
  }

  pthread_exit(NULL);
  return NULL;
}

// 获取触觉传感器状态，三维力数值、接近值，以及传感器状态
void get_touch_status(DeviceHandler *handle, uint8_t slave_id) {
  auto status = stark_get_touch_status(handle, slave_id);
  if (status != NULL) {
    auto data = status->items[3];
    printf("Slave[%hhu] Touch Sensor Status At Pink Finger:\n", slave_id);
    printf("Normal Force: %hu\n", data.normal_force1);
    printf("Tangential Force: %hu\n", data.tangential_force1);
    printf("Tangential Direction: %hu\n", data.tangential_direction1);
    printf("Proximity: %u\n", data.self_proximity1);
    printf("Status: %hu\n", data.status);

    // Free the allocated memory
    free_touch_finger_data(status);
  } else {
    printf("Error: Failed to get touch sensor status\n");
  }
}
