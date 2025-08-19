#include <stdio.h>
#include <cstring>
#include <unistd.h>
#include "stark-sdk.h"
#include "zlgcan/zcan.h"

// 常量定义
#define ZCAN_TYPE_USBCANFD 33 // 设备类型
#define ZCAN_CARD_INDEX 0     // 卡索引
#define ZCAN_CHANNEL_INDEX 0  // 通道索引
#define MAX_CHANNELS 2        // 最大通道数量
#define RX_WAIT_TIME 100      // 接收等待时间
#define RX_BUFF_SIZE 1000     // 接收缓冲区大小

// 函数声明
bool init_can_device();
bool start_can_channel();
void cleanup_resources();
void setup_can_callbacks();
void get_device_info(DeviceHandler *handle, uint8_t slave_id);

int main(int argc, char const *argv[])
{
  // 初始化 CAN 设备
  if (!init_can_device())
  {
    return -1;
  }

  // 启动 CAN 通道
  if (!start_can_channel())
  {
    cleanup_resources();
    return -1;
  }

  setup_can_callbacks(); // 设置读写回调

  init_cfg(STARK_PROTOCOL_TYPE_CAN, LOG_LEVEL_DEBUG);
  auto handle = create_device_handler();
  uint8_t slave_id = 1; // 一代手ID默认为1
  get_device_info(handle, slave_id);

  uint16_t positions_fist[] = {50, 50, 100, 100, 100, 100}; // 握拳
  uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};           // 张开
  useconds_t delay = 1000 * 1000;                           // 1000ms
  stark_set_finger_positions(handle, slave_id, positions_fist, 6);
  usleep(delay);
  stark_set_finger_positions(handle, slave_id, positions_open, 6);
  usleep(delay);

  // 清理资源
  cleanup_resources();
  return 0;
}

bool init_can_device()
{
  // 打开设备
  if (!VCI_OpenDevice(ZCAN_TYPE_USBCANFD, ZCAN_CARD_INDEX, ZCAN_CHANNEL_INDEX))
  {
    printf("Failed to open device\n");
    return false;
  }
  return true;
}

bool start_can_channel()
{
  ZCAN_INIT init;      // 波特率结构体，数据根据zcanpro的波特率计算器得出
  init.mode = 0;       // 0-正常
  init.clk = 60000000; // clock: 60M(V1.01) 80M(V1.03即以上)

  // 仲裁域 1Mbps
  init.aset.sjw = 1;
  init.aset.brp = 5;
  init.aset.tseg1 = 6;
  init.aset.tseg2 = 1;
  init.aset.smp = 0;

  // 数据域 5Mbps
  init.dset.sjw = 1;
  init.dset.brp = 0;
  init.dset.tseg1 = 7;
  init.dset.tseg2 = 2;
  init.dset.smp = 0;

  // 初始化 CAN 通道
  if (!VCI_InitCAN(ZCAN_TYPE_USBCANFD, ZCAN_CARD_INDEX, ZCAN_CHANNEL_INDEX, &init))
  {
    printf("Failed to initialize CAN channel\n");
    return false;
  }

  // 启动 CAN 通道
  if (!VCI_StartCAN(ZCAN_TYPE_USBCANFD, ZCAN_CARD_INDEX, ZCAN_CHANNEL_INDEX))
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
                        ZCAN_20_MSG can_msg;
                        memset(&can_msg, 0, sizeof(ZCAN_20_MSG));

                        can_msg.hdr.inf.txm = 0; // 0-正常发送
                        can_msg.hdr.inf.fmt = 0; // 0-CAN
                        can_msg.hdr.inf.sdf = 0; // 0-数据帧, 1-远程帧
                        can_msg.hdr.inf.sef = 0; // 0-标准帧, 1-扩展帧
                        // can_msg.hdr.inf.echo = 1;    // 发送回显

                        can_msg.hdr.id = can_id;              // ID
                        can_msg.hdr.chn = ZCAN_CHANNEL_INDEX; // 通道
                        can_msg.hdr.len = data_len;           // 数据长度

                        // 填充数据
                        for (uintptr_t i = 0; i < data_len && i < 8; ++i)
                        {
                          can_msg.dat[i] = data[i];
                        }

                        // 发送 CAN 帧
                        printf("Transmitting CAN frame...\n");
                        int result = VCI_Transmit(ZCAN_TYPE_USBCANFD, ZCAN_CARD_INDEX, ZCAN_CHANNEL_INDEX, &can_msg, 1);
                        printf("Transmit result: %d\n", result);
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
                        ZCAN_20_MSG can_data[RX_BUFF_SIZE];
                        int len = VCI_Receive(ZCAN_TYPE_USBCANFD, ZCAN_CARD_INDEX, ZCAN_CHANNEL_INDEX, can_data, RX_BUFF_SIZE, RX_WAIT_TIME); // CAN
                        printf("VCI_Receive, len: %d\n", len);
                        if (len < 1)
                        {
                          return -1;
                        }

                        // 拼接多个 CAN 帧，多个 CAN 帧的 CAN ID 应该一致
                        *can_id_out = can_data[0].hdr.id & 0x1FFFFFFF;

                        int idx = 0;
                        int total_dlc = 0;

                        for (int i = 0; i < len; i++)
                        {
                          ZCAN_20_MSG recv_data = can_data[i];
                          int can_dlc = recv_data.hdr.len;
                          for (int j = 0; j < can_dlc; j++)
                          {
                            data_out[idx++] = recv_data.dat[j];
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
  VCI_ResetCAN(ZCAN_TYPE_USBCANFD, ZCAN_CARD_INDEX, ZCAN_CHANNEL_INDEX); // 重置 CAN 通道
  VCI_CloseDevice(ZCAN_TYPE_USBCANFD, ZCAN_CARD_INDEX);
  printf("Resources cleaned up.\n");
}
