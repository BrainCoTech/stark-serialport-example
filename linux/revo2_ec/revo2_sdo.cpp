#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include <sched.h>
#include <execinfo.h>
#include <stdint.h>
#include <stdbool.h>
#include <thread>
#include <arpa/inet.h> // 为了使用 htonl/ntohl
#include "ecrt.h"

/****************************************************************************/
// 宏定义
/****************************************************************************/
#define FREQUENCY 1000 // 1000 Hz
#define MAX_WAIT_LOOP FREQUENCY * 0.1
#define CLOCK_TO_USE CLOCK_MONOTONIC

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
                       (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

#define STARK_VENDOR_ID     0x00bc0000
#define STARK_PRODUCT_CODE  0x00009252

// SDO配置参数索引定义
#define CONFIG_OBJECT_INDEX 0x8000

// 通用配置子索引
#define HAND_TYPE_SUBINDEX 0x01
#define LED_SWITCH_SUBINDEX 0x02
#define BUZZER_SWITCH_SUBINDEX 0x03
#define VIBRATOR_SWITCH_SUBINDEX 0x04
#define UNIT_MODE_SUBINDEX 0x05
#define POS_CALIBRATION_SUBINDEX 0x06
#define AUTO_CALIBRATION_SUBINDEX 0x07
#define TURBO_MODE_SUBINDEX 0x08
#define TURBO_PARAM_SUBINDEX 0x09

// 保护电流子索引
#define THUMB_FLEX_PRO_CUR_SUBINDEX 0x0A
#define THUMB_AUX_PRO_CUR_SUBINDEX 0x0B
#define INDEX_PRO_CUR_SUBINDEX 0x0C
#define MID_PRO_CUR_SUBINDEX 0x0D
#define RING_PRO_CUR_SUBINDEX 0x0E
#define PINK_PRO_CUR_SUBINDEX 0x0F
#define THUMB_AUX_LOCK_CUR_SUBINDEX 0x10

// 固件信息子索引
#define CTRL_WRIST_FW_VERSION_SUBINDEX 0x11
#define CTRL_SN_SUBINDEX 0x12
#define WRIST_FW_VERSION_SUBINDEX 0x13
#define WRIST_SN_SUBINDEX 0x14

/****************************************************************************/
// 枚举定义
/****************************************************************************/

// 手型枚举
typedef enum
{
  LEFT_HAND = 0,
  RIGHT_HAND = 1
} hand_type_t;

// 开关状态枚举
typedef enum
{
  SWITCH_OFF = 0,
  SWITCH_ON = 1
} switch_state_t;

// 单位模式枚举
typedef enum
{
  NORMAL = 0, // 无量纲
  PHYSICAL = 1  // 物理单位
} unit_mode_t;

/****************************************************************************/
// 结构体定义
/****************************************************************************/

// 保护电流配置结构体
typedef struct
{
  uint16_t thumb_flex_pro_cur; // 拇指尖保护电流
  uint16_t thumb_aux_pro_cur;  // 拇指根保护电流
  uint16_t index_pro_cur;      // 食指保护电流
  uint16_t mid_pro_cur;        // 中指保护电流
  uint16_t ring_pro_cur;       // 无名指保护电流
  uint16_t pink_pro_cur;       // 小指保护电流
  uint16_t thumb_aux_lock_cur; // 拇指副关节锁定电流
} protection_current_config_t;

// 通用配置结构体
typedef struct
{
  uint8_t hand_type;        // 手型
  uint8_t led_switch;       // LED开关
  uint8_t buzzer_switch;    // 蜂鸣器开关
  uint8_t vibrator_switch;  // 振动马达开关
  uint8_t unit_mode;        // 单位模式
  uint8_t auto_calibration; // 自动校准
  uint8_t turbo_mode;       // Turbo模式
  uint32_t turbo_param;     // Turbo参数（32位：duration[16] + interval[16]）
} general_config_t;

// 辅助函数：创建turbo参数
uint32_t make_turbo_param(uint16_t interval, uint16_t duration) {
  return ((uint32_t)interval << 16) | duration;
}

// 辅助函数：解析turbo参数
void parse_turbo_param(uint32_t turbo_param, uint16_t *interval, uint16_t *duration)
{
  *interval = (turbo_param >> 16) & 0xFFFF;
  *duration = turbo_param & 0xFFFF;
}

// 固件信息结构体
typedef struct
{
  char ctrl_wrist_fw_version[21]; // 控制板固件版本, 20字节 + 1结束符
  char ctrl_sn[19];               // 控制板序列号, 18字节 + 1结束符
  char wrist_fw_version[21];      // 手腕板固件版本, 20字节 + 1结束符
  char wrist_sn[19];              // 手腕板序列号, 18字节 + 1结束符
} firmware_info_t;

/****************************************************************************/
// 全局变量
/****************************************************************************/

static ec_master_t *master = NULL;
static ec_slave_config_t *slave_config = NULL;

/****************************************************************************/
// SDO读取函数
/****************************************************************************/

// 通用SDO读取函数
int read_sdo_generic(uint16_t index, uint8_t subindex, void *value, size_t size, bool big_endian)
{
  size_t result_size;
  uint32_t abort_code;

  int ret = ecrt_master_sdo_upload(master, 0, index, subindex,
                                   (uint8_t *)value, size,
                                   &result_size, &abort_code);

  if (ret)
  {
    printf("Failed to read SDO 0x%04X:0x%02X - Error: %d, Abort code: 0x%08X\n",
           index, subindex, ret, abort_code);
    return ret;
  }

  if (result_size != size)
  {
    printf("Unexpected data size for SDO 0x%04X:0x%02X - Expected: %zu, Got: %zu\n",
           index, subindex, size, result_size);
    return -1;
  }

  // 字节序转换（仅对多字节类型）
  if (big_endian && size > 1)
  {
    if (size == 2)
      *(uint16_t *)value = ntohs(*(uint16_t *)value);
    else if (size == 4)
      *(uint32_t *)value = ntohl(*(uint32_t *)value);
  }

  // 打印结果
  if (size == 1)
    printf("Read SDO 0x%04X:0x%02X = %u\n", index, subindex, *(uint8_t *)value);
  else if (size == 2)
    printf("Read SDO 0x%04X:0x%02X = %u\n", index, subindex, *(uint16_t *)value);
  else if (size == 4)
    printf("Read SDO 0x%04X:0x%02X = 0x%08X\n", index, subindex, *(uint32_t *)value);

  return 0;
}

// 读取UINT8类型SDO
int read_sdo_uint8(uint16_t index, uint8_t subindex, uint8_t *value)
{
  return read_sdo_generic(index, subindex, value, sizeof(uint8_t), false);
}

// 读取UINT16类型SDO
int read_sdo_uint16(uint16_t index, uint8_t subindex, uint16_t *value)
{
  return read_sdo_generic(index, subindex, value, sizeof(uint16_t), false);
}

// 读取UINT32类型SDO-小端序
int read_sdo_uint32_little_endian(uint16_t index, uint8_t subindex, uint32_t *value)
{
  return read_sdo_generic(index, subindex, value, sizeof(uint32_t), false);
}

// 读取UINT32类型SDO-大端序
int read_sdo_uint32_big_endian(uint16_t index, uint8_t subindex, uint32_t *value)
{
  return read_sdo_generic(index, subindex, value, sizeof(uint32_t), true);
}

// 读取STRING类型SDO
int read_sdo_string(uint16_t index, uint8_t subindex, char *value, size_t max_size)
{
  size_t result_size;
  uint32_t abort_code;

  int ret = ecrt_master_sdo_upload(master, 0, index, subindex,
                                   (uint8_t *)value, max_size - 1,
                                   &result_size, &abort_code);

  if (ret)
  {
    printf("Failed to read SDO 0x%04X:0x%02X - Error: %d, Abort code: 0x%08X\n",
           index, subindex, ret, abort_code);
    return ret;
  }

  value[result_size] = '\0'; // 确保字符串结束
  printf("Read SDO 0x%04X:0x%02X = \"%s\"\n", index, subindex, value);
  return 0;
}

/****************************************************************************/
// SDO写入函数
/****************************************************************************/

// 写入UINT8类型SDO
int write_sdo_uint8(uint16_t index, uint8_t subindex, uint8_t value)
{
  uint32_t abort_code;

  int ret = ecrt_master_sdo_download(master, 0, index, subindex,
                                     (uint8_t *)&value, sizeof(uint8_t),
                                     &abort_code);

  if (ret)
  {
    printf("Failed to write SDO 0x%04X:0x%02X - Error: %d, Abort code: 0x%08X\n",
           index, subindex, ret, abort_code);
    return ret;
  }

  printf("Write SDO 0x%04X:0x%02X = %u\n", index, subindex, value);
  return 0;
}

// 写入UINT16类型SDO
int write_sdo_uint16(uint16_t index, uint8_t subindex, uint16_t value)
{
  uint32_t abort_code;

  int ret = ecrt_master_sdo_download(master, 0, index, subindex,
                                     (uint8_t *)&value, sizeof(uint16_t),
                                     &abort_code);

  if (ret)
  {
    printf("Failed to write SDO 0x%04X:0x%02X - Error: %d, Abort code: 0x%08X\n",
           index, subindex, ret, abort_code);
    return ret;
  }

  printf("Write SDO 0x%04X:0x%02X = %u\n", index, subindex, value);
  return 0;
}

// 写入UINT32类型SDO
int write_sdo_uint32(uint16_t index, uint8_t subindex, uint32_t value)
{
  uint32_t abort_code;
  uint32_t network_value = htonl(value); // 主机字节序转网络字节序(大端)

  int ret = ecrt_master_sdo_download(master, 0, index, subindex,
                                     (uint8_t *)&network_value, sizeof(uint32_t),
                                     &abort_code);

  if (ret)
  {
    printf("Failed to write SDO 0x%04X:0x%02X - Error: %d, Abort code: 0x%08X\n",
           index, subindex, ret, abort_code);
    return ret;
  }

  printf("Write SDO 0x%04X:0x%02X = 0x%08X\n", index, subindex, value);
  return 0;
}

// 写入STRING类型SDO
int write_sdo_string(uint16_t index, uint8_t subindex, const char *value)
{
  uint32_t abort_code;
  size_t len = strlen(value);

  int ret = ecrt_master_sdo_download(master, 0, index, subindex,
                                     (uint8_t *)value, len,
                                     &abort_code);

  if (ret)
  {
    printf("Failed to write SDO 0x%04X:0x%02X - Error: %d, Abort code: 0x%08X\n",
           index, subindex, ret, abort_code);
    return ret;
  }

  printf("Write SDO 0x%04X:0x%02X = \"%s\"\n", index, subindex, value);
  return 0;
}

/****************************************************************************/
// 高级配置函数
/****************************************************************************/

// 读取通用配置
int read_general_config(general_config_t *config)
{
  int ret = 0;

  printf("=== Reading General Configuration ===\n");

  ret |= read_sdo_uint8(CONFIG_OBJECT_INDEX, HAND_TYPE_SUBINDEX, &config->hand_type);
  ret |= read_sdo_uint8(CONFIG_OBJECT_INDEX, LED_SWITCH_SUBINDEX, &config->led_switch);
  ret |= read_sdo_uint8(CONFIG_OBJECT_INDEX, BUZZER_SWITCH_SUBINDEX, &config->buzzer_switch);
  ret |= read_sdo_uint8(CONFIG_OBJECT_INDEX, VIBRATOR_SWITCH_SUBINDEX, &config->vibrator_switch);
  ret |= read_sdo_uint8(CONFIG_OBJECT_INDEX, UNIT_MODE_SUBINDEX, &config->unit_mode);
  ret |= read_sdo_uint8(CONFIG_OBJECT_INDEX, AUTO_CALIBRATION_SUBINDEX, &config->auto_calibration);
  ret |= read_sdo_uint8(CONFIG_OBJECT_INDEX, TURBO_MODE_SUBINDEX, &config->turbo_mode);

  // 读取Turbo参数（32位，大端序）
  ret |= read_sdo_uint32_big_endian(CONFIG_OBJECT_INDEX, TURBO_PARAM_SUBINDEX, &config->turbo_param);
  
  printf("=====================================\n");
  return ret;
}

// 写入通用配置
int write_general_config(const general_config_t *config)
{
  int ret = 0;

  printf("=== Writing General Configuration ===\n");

  ret |= write_sdo_uint8(CONFIG_OBJECT_INDEX, HAND_TYPE_SUBINDEX, config->hand_type);
  ret |= write_sdo_uint8(CONFIG_OBJECT_INDEX, LED_SWITCH_SUBINDEX, config->led_switch);
  ret |= write_sdo_uint8(CONFIG_OBJECT_INDEX, BUZZER_SWITCH_SUBINDEX, config->buzzer_switch);
  ret |= write_sdo_uint8(CONFIG_OBJECT_INDEX, VIBRATOR_SWITCH_SUBINDEX, config->vibrator_switch);
  ret |= write_sdo_uint8(CONFIG_OBJECT_INDEX, UNIT_MODE_SUBINDEX, config->unit_mode);
  ret |= write_sdo_uint8(CONFIG_OBJECT_INDEX, AUTO_CALIBRATION_SUBINDEX, config->auto_calibration);
  ret |= write_sdo_uint8(CONFIG_OBJECT_INDEX, TURBO_MODE_SUBINDEX, config->turbo_mode);

  // 写入Turbo参数（32位，大端序）
  ret |= write_sdo_uint32(CONFIG_OBJECT_INDEX, TURBO_PARAM_SUBINDEX, config->turbo_param);

  printf("=====================================\n");
  return ret;
}

// 读取保护电流配置
int read_protection_current_config(protection_current_config_t *config)
{
  int ret = 0;

  printf("=== Reading Protection Current Configuration ===\n");

  ret |= read_sdo_uint16(CONFIG_OBJECT_INDEX, THUMB_FLEX_PRO_CUR_SUBINDEX, &config->thumb_flex_pro_cur);
  ret |= read_sdo_uint16(CONFIG_OBJECT_INDEX, THUMB_AUX_PRO_CUR_SUBINDEX, &config->thumb_aux_pro_cur);
  ret |= read_sdo_uint16(CONFIG_OBJECT_INDEX, INDEX_PRO_CUR_SUBINDEX, &config->index_pro_cur);
  ret |= read_sdo_uint16(CONFIG_OBJECT_INDEX, MID_PRO_CUR_SUBINDEX, &config->mid_pro_cur);
  ret |= read_sdo_uint16(CONFIG_OBJECT_INDEX, RING_PRO_CUR_SUBINDEX, &config->ring_pro_cur);
  ret |= read_sdo_uint16(CONFIG_OBJECT_INDEX, PINK_PRO_CUR_SUBINDEX, &config->pink_pro_cur);
  ret |= read_sdo_uint16(CONFIG_OBJECT_INDEX, THUMB_AUX_LOCK_CUR_SUBINDEX, &config->thumb_aux_lock_cur);

  printf("===============================================\n");
  return ret;
}

// 写入保护电流配置
int write_protection_current_config(const protection_current_config_t *config)
{
  int ret = 0;

  printf("=== Writing Protection Current Configuration ===\n");

  ret |= write_sdo_uint16(CONFIG_OBJECT_INDEX, THUMB_FLEX_PRO_CUR_SUBINDEX, config->thumb_flex_pro_cur);
  ret |= write_sdo_uint16(CONFIG_OBJECT_INDEX, THUMB_AUX_PRO_CUR_SUBINDEX, config->thumb_aux_pro_cur);
  ret |= write_sdo_uint16(CONFIG_OBJECT_INDEX, INDEX_PRO_CUR_SUBINDEX, config->index_pro_cur);
  ret |= write_sdo_uint16(CONFIG_OBJECT_INDEX, MID_PRO_CUR_SUBINDEX, config->mid_pro_cur);
  ret |= write_sdo_uint16(CONFIG_OBJECT_INDEX, RING_PRO_CUR_SUBINDEX, config->ring_pro_cur);
  ret |= write_sdo_uint16(CONFIG_OBJECT_INDEX, PINK_PRO_CUR_SUBINDEX, config->pink_pro_cur);
  ret |= write_sdo_uint16(CONFIG_OBJECT_INDEX, THUMB_AUX_LOCK_CUR_SUBINDEX, config->thumb_aux_lock_cur);

  printf("===============================================\n");
  return ret;
}

// 读取固件信息
int read_firmware_info(firmware_info_t *info)
{
  int ret = 0;

  printf("=== Reading Firmware Information ===\n");

  ret |= read_sdo_string(CONFIG_OBJECT_INDEX, CTRL_WRIST_FW_VERSION_SUBINDEX,
                         info->ctrl_wrist_fw_version, sizeof(info->ctrl_wrist_fw_version));
  ret |= read_sdo_string(CONFIG_OBJECT_INDEX, CTRL_SN_SUBINDEX,
                         info->ctrl_sn, sizeof(info->ctrl_sn));
  ret |= read_sdo_string(CONFIG_OBJECT_INDEX, WRIST_FW_VERSION_SUBINDEX,
                         info->wrist_fw_version, sizeof(info->wrist_fw_version));
  ret |= read_sdo_string(CONFIG_OBJECT_INDEX, WRIST_SN_SUBINDEX,
                         info->wrist_sn, sizeof(info->wrist_sn));

  printf("====================================\n");
  return ret;
}

// 写入序列号信息
int write_serial_numbers(const char *ctrl_sn, const char *wrist_sn)
{
  int ret = 0;

  printf("=== Writing Serial Numbers ===\n");

  if (ctrl_sn)
  {
    ret |= write_sdo_string(CONFIG_OBJECT_INDEX, CTRL_SN_SUBINDEX, ctrl_sn);
  }

  if (wrist_sn)
  {
    ret |= write_sdo_string(CONFIG_OBJECT_INDEX, WRIST_SN_SUBINDEX, wrist_sn);
  }

  printf("==============================\n");
  return ret;
}

/****************************************************************************/
// 专用功能函数
/****************************************************************************/

// 设置手型
int set_hand_type(hand_type_t hand_type)
{
  printf("Setting hand type to: %s\n", hand_type == LEFT_HAND ? "LEFT_HAND" : "RIGHT_HAND");
  return write_sdo_uint8(CONFIG_OBJECT_INDEX, HAND_TYPE_SUBINDEX, hand_type);
}

// 控制LED
int control_led(switch_state_t state)
{
  printf("Setting LED to: %s\n", state == SWITCH_ON ? "ON" : "OFF");
  return write_sdo_uint8(CONFIG_OBJECT_INDEX, LED_SWITCH_SUBINDEX, state);
}

// 控制蜂鸣器
int control_buzzer(switch_state_t state)
{
  printf("Setting buzzer to: %s\n", state == SWITCH_ON ? "ON" : "OFF");
  return write_sdo_uint8(CONFIG_OBJECT_INDEX, BUZZER_SWITCH_SUBINDEX, state);
}

// 控制振动马达
int control_vibrator(switch_state_t state)
{
  printf("Setting vibrator to: %s\n", state == SWITCH_ON ? "ON" : "OFF");
  return write_sdo_uint8(CONFIG_OBJECT_INDEX, VIBRATOR_SWITCH_SUBINDEX, state);
}

// 设置单位模式
int set_unit_mode(unit_mode_t mode)
{
  printf("Setting unit mode to: %s\n", mode == NORMAL ? "NORMAL" : "PHYSICAL");
  return write_sdo_uint8(CONFIG_OBJECT_INDEX, UNIT_MODE_SUBINDEX, mode);
}

// 执行手动位置校准
int perform_manual_calibration()
{
  printf("Performing manual position calibration...\n");
  return write_sdo_uint8(CONFIG_OBJECT_INDEX, POS_CALIBRATION_SUBINDEX, 1);
}

// 设置自动校准
int set_auto_calibration(switch_state_t state)
{
  printf("Setting auto calibration to: %s\n", state == SWITCH_ON ? "ON" : "OFF");
  return write_sdo_uint8(CONFIG_OBJECT_INDEX, AUTO_CALIBRATION_SUBINDEX, state);
}

// 设置Turbo模式
int set_turbo_mode(switch_state_t state, uint16_t interval, uint16_t duration)
{
  int ret = 0;
  uint32_t turbo_param = make_turbo_param(interval, duration);

  printf("Setting turbo mode to: %s, interval: %u, duration: %u (param: 0x%08X)\n",
         state == SWITCH_ON ? "ON" : "OFF", interval, duration, turbo_param);

  ret |= write_sdo_uint8(CONFIG_OBJECT_INDEX, TURBO_MODE_SUBINDEX, state);

  if (state == SWITCH_ON)
  {
    // 写入Turbo参数（32位，大端序）
    ret |= write_sdo_uint32(CONFIG_OBJECT_INDEX, TURBO_PARAM_SUBINDEX, turbo_param);
  }

  return ret;
}

/****************************************************************************/
// 信号处理函数
/****************************************************************************/

void handler(int sig)
{
  void *array[10];
  size_t size;

  size = backtrace(array, 10);
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

/****************************************************************************/
// 主函数
/****************************************************************************/

int main(int argc, char **argv)
{
  signal(SIGSEGV, handler);
  signal(SIGABRT, handler);

  printf("=== REVO2 SDO Configuration Tool ===\n");

  // 1. 初始化 EtherCAT 主站
  printf("Requesting master...\n");
  master = ecrt_request_master(0);
  if (!master)
  {
    fprintf(stderr, "Failed to request master.\n");
    return -1;
  }

  // 2. 创建从站配置
  printf("Requesting slave configuration...\n");
  slave_config = ecrt_master_slave_config(master, 0, 0, STARK_VENDOR_ID, STARK_PRODUCT_CODE);
  if (!slave_config)
  {
    fprintf(stderr, "Failed to get slave configuration.\n");
    ecrt_release_master(master);
    return -1;
  }

  // 3. 演示各种SDO操作
  printf("\n=== SDO Configuration Demo ===\n");

  // 读取固件信息
  firmware_info_t fw_info;
  if (read_firmware_info(&fw_info) == 0)
  {
    printf("\nFirmware Information:\n");
    printf("  Control Board FW: %s\n", fw_info.ctrl_wrist_fw_version);
    printf("  Control Board SN: %s\n", fw_info.ctrl_sn);
    printf("  Wrist Board FW:   %s\n", fw_info.wrist_fw_version);
    printf("  Wrist Board SN:   %s\n", fw_info.wrist_sn);
  }

  // 读取当前通用配置
  general_config_t gen_config;
  if (read_general_config(&gen_config) == 0)
  {
    uint16_t interval, duration;
    parse_turbo_param(gen_config.turbo_param, &interval, &duration);

    printf("\nCurrent General Configuration:\n");
    printf("  Hand Type: %s\n", gen_config.hand_type == LEFT_HAND ? "LEFT_HAND" : "RIGHT_HAND");
    printf("  LED: %s\n", gen_config.led_switch ? "ON" : "OFF");
    printf("  Buzzer: %s\n", gen_config.buzzer_switch ? "ON" : "OFF");
    printf("  Vibrator: %s\n", gen_config.vibrator_switch ? "ON" : "OFF");
    printf("  Unit Mode: %s\n", gen_config.unit_mode ? "PHYSICAL" : "NORMAL");
    printf("  Auto Calibration: %s\n", gen_config.auto_calibration ? "ON" : "OFF");
    printf("  Turbo Mode: %s\n", gen_config.turbo_mode ? "ON" : "OFF");
    printf("  Turbo Param: 0x%08X (interval: %u, duration: %u)\n",
           gen_config.turbo_param, interval, duration);
  }

  // 读取保护电流配置
  protection_current_config_t prot_config;
  if (read_protection_current_config(&prot_config) == 0)
  {
    printf("\nProtection Current Configuration:\n");
    printf("  Thumb Flex: %u\n", prot_config.thumb_flex_pro_cur);
    printf("  Thumb Aux:  %u\n", prot_config.thumb_aux_pro_cur);
    printf("  Index:      %u\n", prot_config.index_pro_cur);
    printf("  Middle:     %u\n", prot_config.mid_pro_cur);
    printf("  Ring:       %u\n", prot_config.ring_pro_cur);
    printf("  Pinky:      %u\n", prot_config.pink_pro_cur);
    printf("  Thumb Lock: %u\n", prot_config.thumb_aux_lock_cur);
  }

  // 演示配置修改
  printf("\n=== Configuration Demo ===\n");

  // 控制LED闪烁
  printf("LED blink demo...\n");
  control_led(SWITCH_ON);
  sleep(0.5);
  control_led(SWITCH_OFF);
  sleep(0.5);
  control_led(SWITCH_ON);

  // 设置手型
  // set_hand_type(RIGHT_HAND);
  // sleep(1);
  // set_hand_type(LEFT_HAND);

  // 设置单位模式
  set_unit_mode(PHYSICAL);
  sleep(0.5);
  set_unit_mode(NORMAL);

  // 设置Turbo模式
  set_turbo_mode(SWITCH_ON, 1000, 2000); // interval=1000, duration=2000
  sleep(0.5);
  set_turbo_mode(SWITCH_OFF, 0, 0);

  printf("\n=== SDO Demo Completed ===\n");

  // 4. 清理资源
  ecrt_release_master(master);
  return 0;
}
