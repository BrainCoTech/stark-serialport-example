#include <stdio.h>
#include <windows.h>
#include "stark-sdk.h"
#include "zlgcan/zlgcan.h"

// 全局变量声明
static long device_handle_ = INVALID_DEVICE_HANDLE;
static Handle_chl channel_handle_ = {INVALID_CHANNEL_HANDLE};

// 函数声明
bool init_canfd_device(int device_type, int channel_index);
bool setup_canfd_baudrate(int channel_index, int abit_baud, int dbit_baud);
bool start_canfd_channel(int channel_index);
void setup_canfd_callbacks();
void get_device_info(DeviceHandler *handle, uint8_t slave_id);
void cleanup_resources();

int main(int argc, char const *argv[])
{
  const int device_type = ZCAN_USBCANFD_100U;
  const int channel_index = 0;
  const int abit_baudrate = 1000000; // 仲裁域波特率 1Mbps
  const int dbit_baudrate = 5000000; // 数据域波特率 5Mbps

  // 初始化 CANFD 设备
  if (!init_canfd_device(device_type, channel_index))
  {
    return -1;
  }

  // 启动 CANFD 通道
  if (!start_canfd_channel(channel_index))
  {
    cleanup_resources();
    return -1;
  }

  // 设置 CANFD 波特率参数
  if (!setup_canfd_baudrate(channel_index, abit_baudrate, dbit_baudrate))
  {
    cleanup_resources();
    return -1;
  }

  setup_canfd_callbacks(); // 设置读写回调

  // 打开设备并获取信息
  const uint8_t MASTER_ID = 1;
  init_cfg(STARK_PROTOCOL_TYPE_CAN_FD, LOG_LEVEL_DEBUG);
  auto handle = canfd_init(MASTER_ID);

  // uint8_t slave_id = 0x7e; // 0x7e为左手
  uint8_t slave_id = 0x7f; // 0x7f为右手
  get_device_info(handle, slave_id);

  int delay = 1000; // 1000ms
  uint16_t positions[6] = {500, 500, 500, 500, 500, 500};
  uint16_t durations[6] = {300, 300, 300, 300, 300, 300};
  stark_set_finger_positions_and_durations(handle, slave_id, positions, durations, 6);
  Sleep(delay); // 等待手指到达目标位置

  // 清理资源
  cleanup_resources();
  return 0;
}

bool init_canfd_device(int device_type, int channel_index)
{
  device_handle_ = ZCAN_OpenDevice(device_type, channel_index, 0);
  if (device_handle_ == INVALID_DEVICE_HANDLE)
  {
    printf("Failed to open CANFD device\n");
    return false;
  }
  return true;
}

bool setup_canfd_baudrate(int channel_index, int abit_baud, int dbit_baud)
{
  char abit_path[50] = {0};
  char abit_value[20] = {0};
  char dbit_path[50] = {0};
  char dbit_value[20] = {0};
  char filter_path[50] = {0};

  // 设置仲裁域波特率
  sprintf_s(abit_path, "%d/canfd_abit_baud_rate", channel_index);
  sprintf_s(abit_value, "%d", abit_baud);
  ZCAN_SetValue(device_handle_, abit_path, abit_value);

  // 设置数据域波特率
  sprintf_s(dbit_path, "%d/canfd_dbit_baud_rate", channel_index);
  sprintf_s(dbit_value, "%d", dbit_baud);
  ZCAN_SetValue(device_handle_, dbit_path, dbit_value);

  // 清除滤波器
  sprintf_s(filter_path, "%d/filter_clear", channel_index);
  char filter_value[] = "0";
  ZCAN_SetValue(device_handle_, filter_path, filter_value);

  return true;
}

bool start_canfd_channel(int channel_index)
{
  ZCAN_CHANNEL_INIT_CONFIG config;
  memset(&config, 0, sizeof(config));
  config.can_type = TYPE_CANFD; // 使用 CANFD 类型
  config.canfd.mode = 0;        // 0: 正常模式, 1: 只听模式

  channel_handle_ = ZCAN_InitCAN(device_handle_, channel_index, config)[0];
  if (INVALID_CHANNEL_HANDLE == channel_handle_.handle)
  {
    printf("Failed to initialize CANFD channel\n");
    return false;
  }

  if (ZCAN_StartCAN(channel_handle_) != STATUS_OK)
  {
    printf("Failed to start CANFD channel\n");
    return false;
  }

  return true;
}

void setup_canfd_callbacks()
{
  // CANFD 发送回调
  set_can_tx_callback([](uint8_t slave_id,
                         uint32_t can_id,
                         const uint8_t *data,
                         uintptr_t data_len) -> int
                      {
                        printf("CANFD Send: Slave ID: %d, CAN ID: 0x%X, Data Length: %zu\n", slave_id, can_id, data_len);
                        printf("Data: ");
                        for (uintptr_t i = 0; i < data_len; ++i)
                        {
                          printf("%02x ", data[i]);
                        }
                        printf("\n");

                        // 构造 CANFD 发送数据结构体
                        ZCAN_TransmitFD_Data canfd_data;
                        memset(&canfd_data, 0, sizeof(canfd_data));

                        // 设置 CAN ID，扩展帧 eff=1，数据帧 rtr=0，正常帧 err=0
                        canfd_data.frame.can_id = MAKE_CAN_ID(can_id, 1, 0, 0); // 扩展帧
                        canfd_data.frame.len = data_len;
                        canfd_data.transmit_type = 0;        // 正常发送
                        canfd_data.frame.flags |= CANFD_BRS; // 使用 BRS 位速率切换

                        // 填充数据, 数据长度最大64
                        for (uintptr_t i = 0; i < data_len && i < 64; ++i)
                        { 
                          canfd_data.frame.data[i] = data[i];
                        }

                        // 发送 CANFD 帧
                        int result = ZCAN_TransmitFD(channel_handle_, &canfd_data, 1);
                        return result == 1 ? 0 : -1; // 0 表示成功
                      });

  // CANFD 读取回调
  set_can_rx_callback([](uint8_t slave_id,
                         uint32_t *can_id_out,
                         uint8_t *data_out,
                         uintptr_t *data_len_out) -> int
                      {
                        printf("CANFD Read: Slave ID: %d\n", slave_id);
                        // Sleep(1000); // 等待数据准备

                        // 读取 CANFD 数据
                        ZCAN_ReceiveFD_Data canfd_data[1000];
                        int len = ZCAN_GetReceiveNum(channel_handle_, TYPE_CANFD);
                        printf("ZCAN_GetReceiveNum, len: %d\n", len);
                        len = ZCAN_ReceiveFD(channel_handle_, canfd_data, 1000, 50);
                        printf("ZCAN_ReceiveFD, len: %d\n", len);
                        if (len < 1)
                        {
                          return -1;
                        }

                        // 处理第一帧数据
                        ZCAN_ReceiveFD_Data recv_data = canfd_data[0];
                        canfd_frame frame = recv_data.frame;

                        *can_id_out = frame.can_id;
                        *data_len_out = frame.len; // CANFD 使用 len 而不是 can_dlc

                        // 填充数据, 数据长度最大64
                        for (int i = 0; i < frame.len && i < 64; i++)
                        {
                          data_out[i] = frame.data[i];
                        }

                        return 0; // 成功返回 0
                      });
}

void get_device_info(DeviceHandler *handle, uint8_t slave_id)
{
  DeviceInfo *info = stark_get_device_info(handle, slave_id);
  if (info != NULL)
  {
    printf("Slave[%hhu] Serial Number: %s, FW: %s\n", slave_id, info->serial_number, info->firmware_version);
    if (info->hardware_type != STARK_HARDWARE_TYPE_REVO2_BASIC && info->hardware_type != STARK_HARDWARE_TYPE_REVO2_TOUCH)
    {
      printf("Not Revo2, hardware type: %hhu\n", info->hardware_type);
      exit(1);
    }
    free_device_info(info);
  } else {
    printf("Error: Failed to get device info\n");
    exit(1);
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
