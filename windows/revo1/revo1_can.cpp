// 本示例为Revo2 结合 ZQWL USB-CAN 设备的简单控制示例
// USBCANFD-200U
// USBCANFD-100U
// USBCANFD-100U-mini
// 需要下载厂商提供的.dll
// http://39.108.220.80/download/user/ZQWL/UCANFD/
#include <stdio.h>
#include <windows.h>
#include "stark-sdk.h"
#include "zlgcan/zlgcan.h"

// 全局变量声明
static long device_handle_ = INVALID_DEVICE_HANDLE;
static Handle_chl channel_handle_ = {INVALID_CHANNEL_HANDLE};

// 函数声明
bool init_can_device(int device_type, int channel_index);
bool setup_can_baudrate(int channel_index, int baudrate);
bool start_can_channel(int channel_index);
void setup_can_callbacks();
void get_device_info(DeviceHandler *handle, uint8_t slave_id);
void cleanup_resources();

int main(int argc, char const *argv[])
{
  const int device_type = ZCAN_USBCANFD_100U;
  const int channel_index = 0;
  const int baudrate = 1000000; // 1Mbps

  // 初始化 CAN 设备
  if (!init_can_device(device_type, channel_index))
  {
    return -1;
  }

  // 启动 CAN 通道
  if (!start_can_channel(channel_index))
  {
    cleanup_resources();
    return -1;
  }

  // 设置波特率和时序参数
  if (!setup_can_baudrate(channel_index, baudrate))
  {
    cleanup_resources();
    return -1;
  }

  setup_can_callbacks(); // 设置读写回调

  init_cfg(STARK_PROTOCOL_TYPE_CAN, LOG_LEVEL_DEBUG);
  auto handle = create_device_handler();
  get_device_info(handle, slave_id);

  uint16_t positions_fist[] = {50, 50, 100, 100, 100, 100}; // 握拳
  uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};           // 张开
  int delay = 1000;                                         // 1000ms
  // stark_set_finger_position(handle, slave_id, STARK_FINGER_ID_PINKY, 100);
  // Sleep(delay);
  stark_set_finger_positions(handle, slave_id, positions_fist, 6);
  Sleep(delay);
  stark_set_finger_positions(handle, slave_id, positions_open, 6);
  Sleep(delay);

  // 清理资源
  cleanup_resources();
  return 0;
}

bool init_can_device(int device_type, int channel_index)
{
  device_handle_ = ZCAN_OpenDevice(device_type, channel_index, 0);
  if (device_handle_ == INVALID_DEVICE_HANDLE)
  {
    printf("Failed to open device\n");
    return false;
  }
  return true;
}

bool setup_can_baudrate(int channel_index, int baudrate)
{
  // 设置波特率
  char path[50] = {0};
  char value[10] = {0};
  sprintf_s(path, "%d/baud_rate", channel_index);
  sprintf_s(value, "%d", baudrate);
  ZCAN_SetValue(device_handle_, path, value);

  // 设置 CAN 时序参数（timing0, timing1）
  // 1Mbps 推荐 timing0=0x00, timing1=0x14
  char timing_path0[50] = {0};
  char timing_value0[10] = {0};
  char timing_path1[50] = {0};
  char timing_value1[10] = {0};
  sprintf_s(timing_path0, "%d/abit_timing0", channel_index);
  sprintf_s(timing_value0, "0x00");
  sprintf_s(timing_path1, "%d/abit_timing1", channel_index);
  sprintf_s(timing_value1, "0x14");
  ZCAN_SetValue(device_handle_, timing_path0, timing_value0);
  ZCAN_SetValue(device_handle_, timing_path1, timing_value1);

  return true;
}

bool start_can_channel(int channel_index)
{
  ZCAN_CHANNEL_INIT_CONFIG config;
  memset(&config, 0, sizeof(config));
  config.can_type = TYPE_CAN;
  config.can.mode = 0; // 0: 正常模式, 1: 只听模式

  channel_handle_ = ZCAN_InitCAN(device_handle_, channel_index, config)[0];
  if (INVALID_CHANNEL_HANDLE == channel_handle_.handle)
  {
    printf("Failed to initialize CAN channel\n");
    return false;
  }

  if (ZCAN_StartCAN(channel_handle_) != STATUS_OK)
  {
    printf("Failed to start CAN channel\n");
    return false;
  }

  return true;
}

void setup_can_callbacks()
{
  // CAN 发送回调
  set_can_tx_callback([](uint8_t slave_id,
                         uint32_t can_id,
                         const uint8_t *data,
                         uintptr_t data_len) -> int
                      {
                        printf("CAN Send: Slave ID: %d, CAN ID: 0x%X, Data Length: %zu\n", slave_id, can_id, data_len);
                        printf("Data: ");
                        for (uintptr_t i = 0; i < data_len; ++i)
                        {
                          printf("%02x ", data[i]);
                        }
                        printf("\n");

                        // 构造 CAN 发送数据结构体
                        ZCAN_Transmit_Data can_data;
                        memset(&can_data, 0, sizeof(can_data));

                        // 设置 CAN ID，标准帧 eff=0，扩展帧 eff=1
                        // rtr=0:数据帧，rtr=1:远程帧；err=0:正常帧，err=1:错误帧
                        can_data.frame.can_id = MAKE_CAN_ID(can_id, 0, 0, 0); // 标准帧
                        can_data.frame.can_dlc = data_len;
                        can_data.transmit_type = 0; // 正常发送

                        // 填充数据
                        for (uintptr_t i = 0; i < data_len && i < 8; ++i)
                        {
                          can_data.frame.data[i] = data[i];
                        }

                        // 发送 CAN 帧
                        int result = ZCAN_Transmit(channel_handle_, &can_data, 1);
                        return result == 1 ? 0 : -1; // 0 表示成功
                      });

  // CAN 读取回调
  set_can_rx_callback([](uint8_t slave_id,
                         uint32_t *can_id_out,
                         uint8_t *data_out,
                         uintptr_t *data_len_out) -> int
                      {
                        printf("CAN Read: Slave ID: %d\n", slave_id);

                        // 读取数据
                        ZCAN_Receive_Data can_data[1000];
                        int len = ZCAN_GetReceiveNum(channel_handle_, TYPE_CAN);
                        len = ZCAN_Receive(channel_handle_, can_data, 1000, 50);
                        if (len < 1)
                        {
                          return -1;
                        }

                        // 拼接多个 CAN 帧，多个 CAN 帧的 CAN ID 应该一致
                        *can_id_out = can_data[0].frame.can_id;

                        int idx = 0;
                        int total_dlc = 0;

                        for (int i = 0; i < len; i++)
                        {
                          ZCAN_Receive_Data recv_data = can_data[i];
                          int can_dlc = recv_data.frame.can_dlc;
                          for (int j = 0; j < can_dlc; j++)
                          {
                            data_out[idx++] = recv_data.frame.data[j];
                          }
                          total_dlc += can_dlc;
                        }

                        *data_len_out = total_dlc;
                        return 0; // 成功返回 0
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

void cleanup_resources()
{
  if (channel_handle_.handle != INVALID_CHANNEL_HANDLE)
  {
    ZCAN_ResetCAN(channel_handle_);
  }
  if (device_handle_ != INVALID_DEVICE_HANDLE)
  {
    ZCAN_CloseDevice(device_handle_);
  }
}
