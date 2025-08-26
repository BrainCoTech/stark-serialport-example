#include <stdio.h>
#include <cstring>
#include <unistd.h>
#include "stark-sdk.h"
#include "zlgcan/zcan.h"

// 常量定义
#define ZCANFD_TYPE_USBCANFD 33 // 设备类型
#define ZCANFD_CARD_INDEX 0     // 卡索引
#define ZCANFD_CHANNEL_INDEX 0  // 通道索引
#define MAX_CHANNELS 2          // 最大通道数量
#define RX_WAIT_TIME 100        // 接收等待时间
#define RX_BUFF_SIZE 1000       // 接收缓冲区大小

// 函数声明
bool init_canfd_device();
bool start_canfd_channel();
void cleanup_resources();
void setup_canfd_callbacks();
void get_device_info(DeviceHandler *handle, uint8_t slave_id);

int main(int argc, char const *argv[])
{
  // 初始化 CANFD 设备
  if (!init_canfd_device())
  {
    return -1;
  }

  // 启动 CANFD 通道
  if (!start_canfd_channel())
  {
    cleanup_resources();
    return -1;
  }

  setup_canfd_callbacks(); // 设置读写回调

  init_cfg(STARK_PROTOCOL_TYPE_CAN_FD, LOG_LEVEL_DEBUG);
  const uint8_t MASTER_ID = 1; // 主设备 ID
  auto handle = canfd_init(MASTER_ID);
  uint8_t slave_id = 0x7e; // 二代手左手ID默认为0x7e
  uint8_t slave_id_right = 0x7f; // 二代手右手ID默认为0x7f
  get_device_info(handle, slave_id);
  get_device_info(handle, slave_id_right);

  // stark_set_canfd_baudrate(handle, slave_id_right, 2000); // 设置为2Mbps
  // stark_set_canfd_baudrate(handle, slave_id_right, 5000); // 设置为5Mbps
  // return 0;

  useconds_t delay = 1000 * 1000;                           // 1000ms

  while(1) {
    uint16_t durations[6] = {300, 300, 300, 300, 300, 300};
    uint16_t positions_fist_1000[] = {300, 300, 1000, 1000, 1000, 1000}; // 握拳
    uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};           // 张开
    // 位置+期望时间模式
    stark_set_finger_positions_and_durations(handle, slave_id, positions_fist_1000, durations, 6);
    stark_set_finger_positions_and_durations(handle, slave_id_right, positions_fist_1000, durations, 6);
    usleep(delay);
    stark_set_finger_positions_and_durations(handle, slave_id, positions_open, durations, 6);
    stark_set_finger_positions_and_durations(handle, slave_id_right, positions_open, durations, 6);
    usleep(delay);

    // 位置+期望速度模式
    // uint16_t speeds[6] = {500, 500, 500, 500, 500, 500};
    // stark_set_finger_positions_and_speeds(handle, slave_id, positions_fist, speeds, 6);
    // stark_set_finger_positions_and_speeds(handle, slave_id_right, positions_fist, speeds, 6);
    // usleep(delay);

    // 位置模式-百分比
    // uint16_t positions_fist[] = {30, 30, 100, 100, 100, 100}; // 握拳
    // stark_set_finger_positions(handle, slave_id, positions_fist, 6); 
    // stark_set_finger_positions(handle, slave_id_right, positions_fist, 6);
    // usleep(delay);
    // stark_set_finger_positions(handle, slave_id, positions_open, 6);
    // stark_set_finger_positions(handle, slave_id_right, positions_open, 6);
    // usleep(delay);
  }
  
  // 清理资源
  cleanup_resources();
  return 0;
}

bool init_canfd_device()
{
  // 打开设备
  if (!VCI_OpenDevice(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX))
  {
    printf("Failed to open device\n");
    return false;
  }
  return true;
}

bool start_canfd_channel()
{
  ZCAN_INIT init;      // 波特率结构体，数据根据zcanpro的波特率计算器得出
  init.mode = 0;       // 0-正常
  init.clk = 60000000; // clock: 60M(V1.01) 80M(V1.03即以上)

  // 仲裁域 1Mbps - 80%采样
  // init.aset.sjw = 1;
  // init.aset.brp = 5;
  // init.aset.tseg1 = 6;
  // init.aset.tseg2 = 1;
  // init.aset.smp = 0;

  // 仲裁域 1Mbps - 75%采样
  init.aset.sjw = 1;
  init.aset.brp = 4;
  init.aset.tseg1 = 7;
  init.aset.tseg2 = 2;
  init.aset.smp = 0;

  // 数据域 5Mbps - 75%采样
  init.dset.sjw = 1;
  init.dset.brp = 0;
  init.dset.tseg1 = 7;
  init.dset.tseg2 = 2;
  init.dset.smp = 0;
  printf("数据域 5Mbps - 75%采样\n");

  // 数据域 2Mbps - 73.33%采样
  // init.dset.sjw = 1;
  // init.dset.brp = 1;
  // init.dset.tseg1 = 9;
  // init.dset.tseg2 = 3;
  // init.dset.smp = 0;
  // printf("数据域 2Mbps - 73.33%采样\n");

  // 数据域 1Mbps - 75%采样
  // init.dset.sjw = 1;
  // init.dset.brp = 4;
  // init.dset.tseg1 = 7;
  // init.dset.tseg2 = 2;
  // init.dset.smp = 0;

  // 初始化 CANFD 通道
  if (!VCI_InitCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, &init))
  {
    printf("Failed to initialize CANFD channel\n");
    return false;
  }

  // 终端电阻禁用/使能
  U32 on = 0; // on=0 禁用，on=1 使能
  if (!VCI_SetReference(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, CMD_CAN_TRES, &on)) // 终端电阻
  {   
    printf("CMD_CAN_TRES fail\n");
  } else {
    printf("CMD_CAN_TRES success\n");
  }
 
  // 启动 CANFD 通道
  if (!VCI_StartCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX))
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
                           uint32_t canfd_id,
                           const uint8_t *data,
                           uintptr_t data_len) -> int
                        {
                          printf("CANFD Send: Slave ID: %d, CANFD ID: 0x%X, Data Length: %zu\n", slave_id, canfd_id, data_len);
                          printf("Data: ");
                          for (uintptr_t i = 0; i < data_len; ++i)
                          {
                            printf("%02x ", data[i]);
                          }
                          printf("\n");

                          // 构造 CANFD 发送数据结构体
                          ZCAN_FD_MSG canfd_msg;
                          memset(&canfd_msg, 0, sizeof(ZCAN_FD_MSG));

                          canfd_msg.hdr.inf.txm = 0; // 0-正常发送
                          canfd_msg.hdr.inf.fmt = 1; // 0-CAN, 1-CANFD
                          canfd_msg.hdr.inf.sdf = 0; // 0-数据帧, CANFD只有数据帧!
                          canfd_msg.hdr.inf.sef = 1; // 0-标准帧, 1-扩展帧
                          canfd_msg.hdr.inf.brs = 1; // CANFD专用, 1-使能数据域加速
                          // canfd_msg.hdr.inf.echo = 1;    // 发送回显

                          canfd_msg.hdr.id = canfd_id;              // ID
                          canfd_msg.hdr.chn = ZCANFD_CHANNEL_INDEX; // 通道
                          canfd_msg.hdr.len = data_len;             // 数据长度
                          // canfd_msg.hdr.len = 64;                // 数据长度最大64

                          // 填充数据, 数据长度最大64
                          for (uintptr_t i = 0; i < data_len && i < 64; ++i)
                          {
                            canfd_msg.dat[i] = data[i];
                          }

                          // 发送 CANFD 帧
                          printf("Transmitting CANFD frame...\n");
                          int result = VCI_TransmitFD(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, &canfd_msg, 1);
                          printf("Transmit result: %d\n", result);
                          return result == 1 ? 0 : -1; // 0 表示成功
                        });

  // CANFD 读取回调
  set_can_rx_callback([](uint8_t slave_id,
                           uint32_t *canfd_id_out,
                           uint8_t *data_out,
                           uintptr_t *data_len_out) -> int
                        {
                          printf("CANFD Read: Slave ID: %d\n", slave_id);

                          // 读取数据
                          ZCAN_FD_MSG canfd_data[RX_BUFF_SIZE];
                          int len = VCI_ReceiveFD(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, canfd_data, RX_BUFF_SIZE, RX_WAIT_TIME); // CANFD
                          if (len < 1)
                          {
                            printf("VCI_ReceiveFD, len: %d\n", len);
                            return -1;
                          }

                          // 处理第一帧数据
                          ZCAN_FD_MSG recv_data = canfd_data[0];
                          int canfd_dlc = recv_data.hdr.len;
                          // printf("canfd_dlc: %d\n", canfd_dlc);
                          *canfd_id_out = recv_data.hdr.id;
                          *data_len_out = canfd_dlc;

                          for (int j = 0; j < canfd_dlc && j < 64; j++)
                          {
                            data_out[j] = recv_data.dat[j];
                          }
                          return 0; // 成功返回 0
                        });
}

void cleanup_resources()
{
  VCI_ResetCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX); // 重置 CANFD 通道
  VCI_CloseDevice(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX);
  printf("Resources cleaned up.\n");
}

void get_device_info(DeviceHandler *handle, uint8_t slave_id)
{
  DeviceInfo *info = stark_get_device_info(handle, slave_id);
  if (info != NULL)
  {
    printf("Slave[%hhu] Serial Number: %s, Fw: %s\n", slave_id, info->serial_number, info->firmware_version);
    free_device_info(info);
  }
}
