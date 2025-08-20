#include "stark-sdk.h"
#include <execinfo.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

// 声明函数
void *pdo_thread(void *arg);
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

DeviceHandler *device_handle;
uint16_t slave_pos = 0; // 设备位置

void start_pdo_thread()
{
  // 创建线程
  pthread_t thread_id;
  int result = pthread_create(&thread_id, NULL, pdo_thread, NULL);
  if (result != 0)
  {
    fprintf(stderr, "Failed to create thread: %d\n", result);
    return;
  }
  // 分离线程，让它在后台运行
  pthread_detach(thread_id);

  printf("Async thread started.\n");
}

int main(int argc, char const *argv[])
{
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
  if (info != NULL)
  {
    printf(
        "Slave[%hu] SKU Type: %hhu, Serial Number: %s, Firmware Version: %s\n",
        slave_pos, (uint8_t)info->sku_type, info->serial_number,
        info->firmware_version);
    free_device_info(info);
  }
  else
  {
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
  // # 常见 assign_activate 位定义：
  // # 位位置	掩码（十六进制）	功能说明
  // # bit 0	0x0001	启用 SYNC0 事件
  // # bit 1	0x0002	启用 SYNC1 事件
  // # bit 2	0x0004	启用 Latch0
  // # bit 3	0x0008	启用 Latch1
  // # bit 4	0x0010	启用时间启动（STARTTIME）同步
  // # bit 5	0x0020	启用应用层时间过滤
  // # bit 8	0x0100	启用周期同步
  // # bit 9	0x0200	启用周期和起始时间
  // ethercat_start_loop(device_handle, slave_positions, 1, 0x301, 500000, 0, 0, 0); // 启动PDO循环Thread, 同时启用周期同步 + SYNC0（推荐用于主同步）
  // ethercat_start_loop(device_handle, slave_positions, 1, 0x303, 500000, 0, 1000, 0); // 启动PDO循环Thread, 同时启用 SYNC0/SYNC1 和周期同步
  ethercat_start_loop(device_handle, slave_positions, 1, 0, 500000, 0, 0, 0); // 启动PDO循环Thread，不启用DC同步
  printf("start_loop done\n");

  start_pdo_thread();
  usleep(500 * 1000 * 1000); // wait test
  printf("test done\n");
  ethercat_stop_loop(device_handle);
  printf("stop_loop done\n");
  ethercat_close(device_handle);
  return 0;
}

// template <typename Func, typename... Args>
// void run_in_thread(Func func, Args... args)
// {
//   std::thread t([=]()
//                 { func(args...); });
//   t.detach();
// }

// 获取设备序列号、固件版本等信息
void *pdo_thread(void *arg)
{
  useconds_t delay = 50 * 1000; // 1000ms

  // 单个手指，按速度/电流/PWM控制
  // 其中符号表示方向，正表示为握紧方向，负表示为松开方向
  auto finger_id = STARK_FINGER_ID_RING;
  stark_set_finger_speed(device_handle, slave_pos, finger_id,
                         500); // -1000 ~ 1000
  usleep(delay);               // 等待手指到达目标位置
  stark_set_finger_current(device_handle, slave_pos, finger_id,
                           -300); // -1000 ~ 1000
  usleep(delay);                  // 等待手指到达目标位置
  stark_set_finger_pwm(device_handle, slave_pos, finger_id,
                       700); // -1000 ~ 1000
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

  while (1)
  {
    auto finger_status = stark_get_motor_status(device_handle, slave_pos);
    if (finger_status != NULL)
    {
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
    }
  }

  pthread_exit(NULL);
  return NULL;
}
