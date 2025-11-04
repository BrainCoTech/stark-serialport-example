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
bool init_canfd_device();
bool start_canfd_channel();
void cleanup_resources();
void setup_canfd_callbacks();
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

  setup_canfd_callbacks();

  // 初始化 STARK SDK
  init_cfg(STARK_PROTOCOL_TYPE_CAN_FD, LOG_LEVEL_DEBUG);
  const uint8_t MASTER_ID = 1;
  auto handle = canfd_init(MASTER_ID);
  // uint8_t slave_id = 0x7e; // 二代手左手ID默认为0x7e
  uint8_t slave_id = 0x7f; // 二代手右手ID默认为0x7f

  get_device_info(handle, slave_id);

  // 设置 DFU 回调
  g_start_time = get_current_time_ms();
  set_dfu_state_callback(dfu_state_callback);
  set_dfu_progress_callback(dfu_progress_callback);

  // 启动 DFU 升级
  // start_dfu(handle, slave_id, "ota_bin/stark2/stark2_fw_V0.0.14_20250723135853.bin", 5);
  start_dfu(handle, slave_id, "ota_bin/stark2/Revo2_V1.0.2.C_2511031119_can.bin", 5);

  printf("Revo2 CANFD DFU, Waiting for DFU to complete...\n");
  useconds_t delay = 100 * 1000 * 1000; // 100s, wait for DFU to complete
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

  // 初始化 CANFD 通道
  if (!VCI_InitCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, &init))
  {
    printf("Failed to initialize CANFD channel\n");
    return false;
  }

  // 启动 CANFD 通道
  if (!VCI_StartCAN(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX))
  {
    printf("Failed to start CANFD channel\n");
    return false;
  }

  return true;
}

// ================== CANFD 回调设置 ==================

void setup_canfd_callbacks()
{
  // CANFD 发送回调
  set_can_tx_callback([](uint8_t slave_id,
                         uint32_t canfd_id,
                         const uint8_t *data,
                         uintptr_t data_len) -> int
                      {
                        // printf("CANFD Send: Slave ID: %d, CANFD ID: 0x%X, Data Length: %zu\n", slave_id, canfd_id, data_len);
                        // printf("Data: ");
                        // for (uintptr_t i = 0; i < data_len; ++i)
                        // {
                        //   printf("%02x ", data[i]);
                        // }
                        // printf("\n");

                        // 构造 CANFD 发送数据结构体
                        ZCAN_FD_MSG canfd_msg;
                        memset(&canfd_msg, 0, sizeof(ZCAN_FD_MSG));

                        canfd_msg.hdr.inf.txm = 0; // 0-正常发送
                        canfd_msg.hdr.inf.fmt = 1; // 0-CAN, 1-CANFD
                        canfd_msg.hdr.inf.sdf = 0; // 0-数据帧, CANFD只有数据帧!
                        canfd_msg.hdr.inf.sef = 1; // 0-标准帧, 1-扩展帧
                        // canfd_msg.hdr.inf.echo = 1;    // 发送回显

                        canfd_msg.hdr.id = canfd_id;              // ID
                        canfd_msg.hdr.chn = ZCANFD_CHANNEL_INDEX; // 通道
                        canfd_msg.hdr.len = data_len;             // 数据长度
                        // canfd_msg.hdr.len = 64;                   // 数据长度

                        // 填充数据, 数据长度最大64
                        for (uintptr_t i = 0; i < data_len && i < 64; ++i)
                        {
                          canfd_msg.dat[i] = data[i];
                        }

                        // 发送 CANFD 帧
                        int result = VCI_TransmitFD(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, &canfd_msg, 1);
                        if (result != 1)
                          printf("Transmit result: %d\n", result);
                        return result == 1 ? 0 : -1; // 0 表示成功
                      });

  // CANFD 读取回调
  set_can_rx_callback([](uint8_t slave_id,
                         uint32_t *canfd_id_out,
                         uint8_t *data_out,
                         uintptr_t *data_len_out) -> int
                      {
                        // printf("CANFD Read: Slave ID: %d\n", slave_id);

                        // 读取数据
                        ZCAN_FD_MSG canfd_data[RX_BUFF_SIZE];
                        int len = VCI_ReceiveFD(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, canfd_data, RX_BUFF_SIZE, RX_WAIT_TIME); // CANFD
                        if (len < 1)
                        {
                          printf("VCI_ReceiveFD, len: %d\n", len);
                          // 重试一次, 最后一包响应耗时比较长, 固件将数据写入到Flash，等待2s后继续
                          usleep(2000 * 1000);
                          printf("Retrying VCI_ReceiveFD...\n");
                          len = VCI_ReceiveFD(ZCANFD_TYPE_USBCANFD, ZCANFD_CARD_INDEX, ZCANFD_CHANNEL_INDEX, canfd_data, RX_BUFF_SIZE, RX_WAIT_TIME); // CANFD
                          printf("VCI_ReceiveFD, len: %d\n", len);
                        }
                        if (len < 1)
                        {
                          return -1;
                        }

                        // 处理第一帧数据
                        ZCAN_FD_MSG recv_data = canfd_data[0];
                        int canfd_dlc = recv_data.hdr.len;
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
