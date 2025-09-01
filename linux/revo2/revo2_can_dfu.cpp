// 本示例为Revo2 结合 ZLG USB-CAN FD设备的固件升级示例
// USBCANFD-200U
// USBCANFD-100U
// USBCANFD-100U-mini
// 需要下载厂商提供的.so
// https://manual.zlg.cn/web/#/146
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <execinfo.h>
#include <cstring>
#include <sys/time.h>
#include "zlgcan/zcan.h"
#include "stark-sdk.h"

// ================== 常量定义 ==================
#define ZCANFD_TYPE_USBCANFD 33 // 设备类型
#define ZCANFD_CARD_INDEX 0     // 卡索引
#define ZCANFD_CHANNEL_INDEX 0  // 通道索引
#define MAX_CHANNELS 2          // 最大通道数量
#define RX_WAIT_TIME 100        // 接收等待时间
#define RX_BUFF_SIZE 1000       // 接收缓冲区大小

// ================== 全局变量 ==================
static int g_start_time = 0;

// ================== 函数声明 ==================
void handler(int sig);
bool init_can_device();
bool start_can_channel();
void cleanup_resources();
void setup_can_callbacks();
void get_device_info(DeviceHandler *handle, uint8_t slave_id);
int get_current_time_ms();
void dfu_state_callback(uint8_t slave_id, uint8_t state);
void dfu_progress_callback(uint8_t slave_id, float progress);

// ================== 主函数 ==================

int main(int argc, char const *argv[])
{
  // 设置信号处理器
  signal(SIGSEGV, handler);
  signal(SIGABRT, handler);

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

  setup_can_callbacks();

  // 初始化 STARK SDK
  init_cfg(STARK_PROTOCOL_TYPE_CAN, LOG_LEVEL_DEBUG);
  auto handle = create_device_handler();
  uint8_t slave_id = 1; // 二代手左手ID默认为1
  // uint8_t slave_id = 2; // 二代手右手ID默认为2
  get_device_info(handle, slave_id);

  // 设置 DFU 回调
  g_start_time = get_current_time_ms();
  set_dfu_state_callback(dfu_state_callback);
  set_dfu_progress_callback(dfu_progress_callback);

  // 启动 DFU 升级
  start_dfu(handle, slave_id, "ota_bin/stark2/Revo2_V1.0.4_2508291545.bin", 5);

  printf("Waiting for DFU to complete...\n");
  useconds_t delay = 300 * 1000 * 1000; // 300s, wait for DFU to complete
  usleep(delay);

  // 清理资源
  cleanup_resources();
  return 0;
}

void dfu_state_callback(uint8_t slave_id, uint8_t state)
{
  printf("DFU State: %hhu\n", state);
  if (state == 4)
  {
    printf("DFU finished, elapsed time: %d ms\n", get_current_time_ms() - g_start_time);
    cleanup_resources();
    exit(0);
  }
}

void dfu_progress_callback(uint8_t slave_id, float progress)
{
  printf("DFU Progress: %.2f%%\n", progress * 100);
}

// ================== CANFD 设备管理 ==================

bool init_can_device()
{
  // 打开设备
  if (!VCI_OpenDevice(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX))
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
  if (!VCI_InitCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, &init))
  {
    printf("Failed to initialize CANFD channel\n");
    return false;
  }

  // 启动 CAN 通道
  if (!VCI_StartCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX))
  {
    printf("Failed to start CANFD channel\n");
    return false;
  }

  return true;
}

// ================== CAN 回调设置 ==================

void setup_can_callbacks()
{
  // CAN 发送回调
  set_can_tx_callback([](uint8_t slave_id,
                         uint32_t can_id,
                         const uint8_t *data,
                         uintptr_t data_len) -> int
                      {
                        // printf("CAN Send: Slave ID: %d, CAN ID: 0x%X, Data Length: %zu\n", slave_id, can_id, data_len);
                        // printf("Data: ");
                        // for (uintptr_t i = 0; i < data_len; ++i)
                        // {
                        //   printf("%02x ", data[i]);
                        // }
                        // printf("\n");

                        // 构造 CAN 发送数据结构体
                        ZCAN_20_MSG can_msg;
                        memset(&can_msg, 0, sizeof(ZCAN_20_MSG));

                        can_msg.hdr.inf.txm = 0; // 0-正常发送
                        can_msg.hdr.inf.fmt = 0; // 0-CAN
                        can_msg.hdr.inf.sdf = 0; // 0-数据帧, 1-远程帧
                        can_msg.hdr.inf.sef = 0; // 0-标准帧, 1-扩展帧

                        can_msg.hdr.id = can_id;                // ID
                        can_msg.hdr.chn = ZCANFD_CHANNEL_INDEX; // 通道
                        can_msg.hdr.len = data_len;             // 数据长度

                        // 填充数据
                        for (uintptr_t i = 0; i < data_len && i < 8; ++i)
                        {
                          can_msg.dat[i] = data[i];
                        }

                        // 发送 CAN 帧
                        int result = VCI_Transmit(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, &can_msg, 1);
                        if (result != 1)
                          printf("Transmit result: %d\n", result);
                        return result == 1 ? 0 : -1; // 0 表示成功
                      });

  // CANFD 读取回调
  set_can_rx_callback([](uint8_t slave_id,
                         uint32_t *can_id_out,
                         uint8_t *data_out,
                         uintptr_t *data_len_out) -> int
                      {
                        // printf("CAN Read: Slave ID: %d\n", slave_id);

                        // 读取数据
                        ZCAN_20_MSG can_data[RX_BUFF_SIZE];
                        int len = VCI_Receive(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, can_data, RX_BUFF_SIZE, RX_WAIT_TIME); // CAN
                        if (len < 1)
                        {
                          printf("ZCAN Receive, len: %d\n", len);
                          // 重试一次, 最后一包响应耗时比较长, 固件将数据写入到Flash，等待2s后继续
                          usleep(2000 * 1000);
                          printf("Retrying ZCAN Receive...\n");
                          len = VCI_Receive(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, can_data, RX_BUFF_SIZE, RX_WAIT_TIME); // CANFD
                          printf("ZCAN Receive, len: %d\n", len);
                        }
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
    printf("Slave[%hhu] Serial Number: %s, FW: %s\n", slave_id, info->serial_number, info->firmware_version);
    free_device_info(info);
  }
}

// ================== 信号处理和工具函数 ==================
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

int get_current_time_ms()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (int)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}
