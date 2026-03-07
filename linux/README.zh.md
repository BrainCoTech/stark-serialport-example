# BrainCo 灵巧手 C++ SDK（Linux/Ubuntu）

> ⚠️ **已弃用**: 此文件夹已弃用，将在未来版本中删除。请使用统一的跨平台 `c/` 文件夹：[C++ 开发指南](../c/README.md)

[English](README.md) | [中文](README.zh.md)

BrainCo 灵巧手设备在 Linux/Ubuntu 平台上的完整 C++ SDK 和示例。

## 📋 目录

- [系统要求](#-系统要求)
- [安装](#-安装)
- [快速开始](#-快速开始)
- [通信协议](#-通信协议)
- [API 参考](#-api-参考)
- [示例程序](#-示例程序)
- [构建系统](#-构建系统)

## 💻 系统要求

- **操作系统**：Ubuntu 20.04/22.04 LTS (x86_64/aarch64), glibc ≥ 2.31
- **编译器**：支持 C++11 的 GCC
- **构建工具**：make、pkg-config
- **依赖项**：Stark SDK 库（自动下载）

## 📦 安装

### 下载 SDK 库

首先，下载所需的 Stark SDK 库：

```bash
# 从项目根目录执行
rm VERSION
./download-lib.sh
```

此脚本会将最新的 SDK 库下载到 `dist/` 目录。

### 验证安装

```bash
# 检查库文件是否已下载
ls dist/
# 应该看到：libstark-sdk.so 和其他库文件
```

## 🚀 快速开始

### 基础控制示例（Revo1）

```cpp
#include "stark-sdk.h"
#include "stark_common.h"
#include <unistd.h>

int main() {
    // 自动检测并连接设备
    auto cfg = auto_detect_modbus_revo1(NULL, true);
    if (cfg == NULL) return -1;
    
    auto handle = modbus_open(cfg->port_name, cfg->baudrate);
    uint8_t slave_id = cfg->slave_id;
    free_device_config(cfg);
    
    // 控制手指 - 闭合抓握
    uint16_t positions_fist[] = {500, 500, 1000, 1000, 1000, 1000};
    stark_set_finger_positions(handle, slave_id, positions_fist, 6);
    usleep(1000 * 1000); // 等待 1 秒
    
    // 张开手指
    uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};
    stark_set_finger_positions(handle, slave_id, positions_open, 6);
    
    return 0;
}
```

### 基础控制示例（Revo2）

```cpp
#include "stark-sdk.h"
#include "stark_common.h"
#include <unistd.h>

int main() {
    // 初始化日志
    init_logging(LOG_LEVEL_INFO);
    
    // 自动检测并连接设备
    auto cfg = auto_detect_modbus_revo2("/dev/ttyUSB0", true);
    if (cfg == NULL) return -1;
    
    auto handle = modbus_open(cfg->port_name, cfg->baudrate);
    uint8_t slave_id = cfg->slave_id;
    free_device_config(cfg);
    
    // 控制手指
    uint16_t positions[] = {400, 400, 1000, 1000, 1000, 1000};
    stark_set_finger_positions(handle, slave_id, positions, 6);
    
    return 0;
}
```

## 🔌 通信协议

### Revo1 支持的协议

| 协议 | 说明 | 示例目录 | 所需硬件 |
|------|------|---------|---------|
| RS-485 (Modbus) | 串口通信 | [revo1/](revo1/) | USB 转 RS485 适配器 |
| CAN | 控制器局域网络 | [revo1/](revo1/) | ZLG USB-CAN 设备或 SocketCAN |

### Revo2 支持的协议

| 协议 | 说明 | 示例目录 | 所需硬件 |
|------|------|---------|---------|
| RS-485 (Modbus) | 串口通信 | [revo2/](revo2/) | USB 转 RS485 适配器 |
| CAN | 控制器局域网络 | [revo2/](revo2/) | ZLG USB-CAN 设备或 SocketCAN |
| CANFD | 灵活数据速率 CAN | [revo2/](revo2/) | ZLG USB-CANFD 设备或 SocketCAN |

## 📚 API 参考

### 核心 SDK 头文件：`stark-sdk.h`

在代码中包含 SDK：
```cpp
#include "stark-sdk.h"
```

### 初始化和配置

#### `init_logging(log_level)`
初始化 SDK 日志。

**参数：**
- `log_level` (LogLevel)：日志级别
  - `LOG_LEVEL_DEBUG`、`LOG_LEVEL_INFO`、`LOG_LEVEL_WARN`、`LOG_LEVEL_ERROR`

**示例：**
```cpp
init_logging(LOG_LEVEL_INFO);
```

#### `init_device_handler(protocol_type, master_id)` (v1.1.0 新增)
为 CAN/CANFD/EtherCAT 协议初始化设备处理器。

**参数：**
- `protocol_type` (StarkProtocolType)：协议类型
  - `STARK_PROTOCOL_TYPE_MODBUS`
  - `STARK_PROTOCOL_TYPE_CAN`
  - `STARK_PROTOCOL_TYPE_CAN_FD`
  - `STARK_PROTOCOL_TYPE_ETHER_CAT`
- `master_id` (uint8_t)：主站 ID（默认：0）

**返回值：** `DeviceHandler*` - 设备处理器实例

**示例：**
```cpp
auto handle = init_device_handler(STARK_PROTOCOL_TYPE_CAN_FD, 0);
```

### 连接管理

#### `auto_detect_modbus_revo1(port_name, quick)`
自动检测并通过 Modbus 连接 Revo1 设备。

**参数：**
- `port_name` (const char*)：串口名称。`NULL` 表示自动检测。
- `quick` (bool)：快速检测模式。`true` = 更快，`false` = 更全面。

**返回值：** `DeviceConfig*` - 配置结构（必须使用 `free_device_config` 释放）

**示例：**
```cpp
auto cfg = auto_detect_modbus_revo1(NULL, true);
if (cfg != NULL) {
    auto handle = modbus_open(cfg->port_name, cfg->baudrate);
    uint8_t slave_id = cfg->slave_id;
    free_device_config(cfg);
}
```

#### `modbus_open(port_name, baudrate)`
使用指定参数打开 Modbus 连接。

**参数：**
- `port_name` (const char*)：串口名称（例如："/dev/ttyUSB0"）
- `baudrate` (int)：通信波特率

**返回值：** `DeviceHandler*` - 设备处理器实例

### 设备信息

#### `stark_get_device_info(handle, slave_id)`
获取设备信息和配置。

**返回值：** `DeviceInfo*` - 设备信息结构（必须使用 `free_device_info` 释放）

**DeviceInfo 结构：**
- `serial_number` (char*)：设备序列号
- `firmware_version` (char*)：固件版本字符串
- `hardware_type` (StarkHardwareType)：硬件类型枚举
  - `STARK_HARDWARE_TYPE_REVO1`、`STARK_HARDWARE_TYPE_REVO1_TOUCH`
  - `STARK_HARDWARE_TYPE_REVO2`、`STARK_HARDWARE_TYPE_REVO2_TOUCH`

**示例：**
```cpp
auto info = stark_get_device_info(handle, slave_id);
if (info != NULL) {
    printf("序列号: %s, 固件: %s\n", 
           info->serial_number, info->firmware_version);
    free_device_info(info);
}
```

### 手指控制

#### 手指 ID 枚举

```cpp
typedef enum {
    STARK_FINGER_ID_THUMB = 0,      // 大拇指
    STARK_FINGER_ID_THUMB_AUX = 1,  // 大拇指辅助
    STARK_FINGER_ID_INDEX = 2,      // 食指
    STARK_FINGER_ID_MIDDLE = 3,     // 中指
    STARK_FINGER_ID_RING = 4,       // 无名指
    STARK_FINGER_ID_PINKY = 5       // 小指
} StarkFingerId;
```

#### `stark_set_finger_positions(handle, slave_id, positions, count)`
设置所有手指的目标位置。

**参数：**
- `positions` (uint16_t[])：6 个关节的位置值 [0-1000]
- `count` (size_t)：位置数量（应为 6）

**位置范围：** 0（完全张开）到 1000（完全闭合）

**示例：**
```cpp
// 闭合抓握
uint16_t positions_fist[] = {600, 600, 1000, 1000, 1000, 1000};
stark_set_finger_positions(handle, slave_id, positions_fist, 6);

// 张开所有手指
uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};
stark_set_finger_positions(handle, slave_id, positions_open, 6);
```

#### `stark_set_finger_position(handle, slave_id, finger_id, position)`
设置单个手指的位置。

**示例：**
```cpp
// 仅闭合小指
stark_set_finger_position(handle, slave_id, STARK_FINGER_ID_PINKY, 1000);
```

#### Revo2 高级控制函数

**速度控制：**
```cpp
// 单个手指速度控制
stark_set_finger_speed(handle, slave_id, STARK_FINGER_ID_MIDDLE, 500);

// 多个手指速度控制
int16_t speeds[] = {100, 100, 500, 500, 500, 500};
stark_set_finger_speeds(handle, slave_id, speeds, 6);
```

**电流控制：**
```cpp
// 单个手指电流控制
stark_set_finger_current(handle, slave_id, STARK_FINGER_ID_INDEX, -300);

// 多个手指电流控制
int16_t currents[] = {-300, -300, -300, -300, -300, -300};
stark_set_finger_currents(handle, slave_id, currents, 6);
```

**位置+速度控制：**
```cpp
// 单个手指：位置 + 速度
stark_set_finger_position_with_speed(handle, slave_id, 
                                     STARK_FINGER_ID_MIDDLE, 1000, 50);

// 多个手指：位置 + 速度
uint16_t positions[] = {300, 300, 500, 500, 500, 500};
uint16_t speeds[] = {500, 500, 500, 500, 500, 500};
stark_set_finger_positions_and_speeds(handle, slave_id, positions, speeds, 6);
```

**位置+时长控制：**
```cpp
// 单个手指：位置 + 时长（毫秒）
stark_set_finger_position_with_millis(handle, slave_id, 
                                      STARK_FINGER_ID_THUMB, 1000, 1000);

// 多个手指：位置 + 时长
uint16_t positions[] = {300, 300, 500, 500, 500, 500};
uint16_t durations[] = {300, 300, 300, 300, 300, 300};
stark_set_finger_positions_and_durations(handle, slave_id, positions, durations, 6);
```

### 电机状态

#### `stark_get_motor_status(handle, slave_id)`
获取当前电机状态，包括位置、速度、电流和状态。

**返回值：** `MotorStatusData*` - 电机状态结构（必须使用 `free_motor_status_data` 释放）

**MotorStatusData 结构：**
- `positions[6]` (uint16_t)：当前位置 [0-1000]
- `speeds[6]` (int16_t)：当前速度
- `currents[6]` (int16_t)：电流值
- `states[6]` (uint8_t)：电机状态标志

**示例：**
```cpp
auto status = stark_get_motor_status(handle, slave_id);
if (status != NULL) {
    printf("位置: %hu, %hu, %hu, %hu, %hu, %hu\n",
           status->positions[0], status->positions[1],
           status->positions[2], status->positions[3],
           status->positions[4], status->positions[5]);
    free_motor_status_data(status);
}
```

### 手指配置（Revo2）

```cpp
// 设置单位模式
stark_set_finger_unit_mode(handle, slave_id, FINGER_UNIT_MODE_NORMALIZED);

// 设置/获取最小位置
stark_set_finger_min_position(handle, slave_id, finger_id, 0);
auto min_pos = stark_get_finger_min_position(handle, slave_id, finger_id);

// 设置/获取最大位置
stark_set_finger_max_position(handle, slave_id, finger_id, 1000);
auto max_pos = stark_get_finger_max_position(handle, slave_id, finger_id);

// 设置/获取最大速度
stark_set_finger_max_speed(handle, slave_id, finger_id, 130);
auto max_speed = stark_get_finger_max_speed(handle, slave_id, finger_id);

// 设置/获取最大电流
stark_set_finger_max_current(handle, slave_id, finger_id, 1000);
auto max_current = stark_get_finger_max_current(handle, slave_id, finger_id);

// 设置/获取保护电流
stark_set_finger_protected_current(handle, slave_id, finger_id, 500);
auto protected_current = stark_get_finger_protected_current(handle, slave_id, finger_id);
```

### 触觉传感器（触觉设备）

```cpp
// 启用所有触觉传感器
stark_enable_touch_sensor(handle, slave_id, 0x1F);
```

### 工具函数（stark_common.h）

```cpp
// 设置信号处理器（用于崩溃调试）
setup_signal_handlers();

// 验证设备类型并打印信息
bool is_revo1 = verify_device_is_revo1(handle, slave_id);
bool is_revo2 = verify_device_is_revo2(handle, slave_id);

// 获取并打印设备信息
get_and_print_device_info(handle, slave_id);

// 获取并打印扩展信息
get_and_print_extended_info(handle, slave_id);
```

## 📂 示例程序

### Revo1 示例

| 示例 | 说明 | 文件 |
|------|------|------|
| 基础控制 | 获取设备信息，控制手指 | [revo1_ctrl.cpp](revo1/revo1_ctrl.cpp) |
| 多手控制 | 控制多只手 | [revo1_ctrl_multi.cpp](revo1/revo1_ctrl_multi.cpp) |
| CAN 控制 | 通过 CAN 协议控制 | [revo1_can.cpp](revo1/revo1_can.cpp) |
| 自定义 CAN | 自定义 CAN 实现 | [revo1_can_customed.cpp](revo1/revo1_can_customed.cpp) |
| 自定义 Modbus | 自定义 Modbus 实现 | [revo1_customed_modbus.cpp](revo1/revo1_customed_modbus.cpp) |
| 异步 Modbus | 异步 Modbus 控制 | [revo1_customed_modbus_async.cpp](revo1/revo1_customed_modbus_async.cpp) |
| 触觉传感器 | 读取触觉传感器数据 | [revo1_touch.cpp](revo1/revo1_touch.cpp) |

**详细指南：** [Revo1 README](revo1/README.md)

### Revo2 示例

| 示例 | 说明 | 文件 |
|------|------|------|
| 基础控制 | 获取设备信息，控制手指 | [revo2_ctrl.cpp](revo2/revo2_ctrl.cpp) |
| 多手控制 | 控制多只手 | [revo2_ctrl_multi.cpp](revo2/revo2_ctrl_multi.cpp) |
| CAN 控制 | 通过 CAN 协议控制 | [revo2_can_ctrl.cpp](revo2/revo2_can_ctrl.cpp) |
| CANFD 控制 | 通过 CANFD 协议控制 | [revo2_canfd.cpp](revo2/revo2_canfd.cpp) |
| CANFD 触觉 | 通过 CANFD 控制触觉版本 | [revo2_canfd_touch.cpp](revo2/revo2_canfd_touch.cpp) |
| 自定义 CANFD | 自定义 CANFD 实现 | [revo2_canfd_customed.cpp](revo2/revo2_canfd_customed.cpp) |
| 自定义 Modbus | 自定义 Modbus 实现 | [revo2_customed_modbus.cpp](revo2/revo2_customed_modbus.cpp) |
| 异步 Modbus | 异步 Modbus 控制 | [revo2_customed_modbus_async.cpp](revo2/revo2_customed_modbus_async.cpp) |
| 触觉传感器 | 读取触觉传感器数据 | [revo2_touch.cpp](revo2/revo2_touch.cpp) |

**详细指南：** [Revo2 README](revo2/README.md)

## 🛠️ 构建系统

### 智能构建和运行

Makefile 提供智能构建命令，可自动检测正确的模式：

```bash
# 智能编译 + 运行（自动检测模式）
make run revo1_ctrl           # Modbus 程序（默认）
make run revo1_can            # 自动检测 CAN 模式
make run revo2_ctrl           # Modbus 程序
make run revo2_canfd          # 自动检测 CANFD 模式

# 显示可用目标
make run                      # 显示使用帮助
```

### 传统构建命令

```bash
# 清理构建产物
make clean

# 使用特定模式构建
make                          # 使用默认模式构建（Modbus）
make MODE=can                 # 使用 CAN 接口模式构建

# 仅运行（必须先编译）
make run_revo1_ctrl           # 运行 revo1_ctrl 示例
make run_revo2_ctrl           # 运行 revo2_ctrl 示例
```

### 构建模式

| 模式 | 说明 | 所需硬件 |
|------|------|---------|
| (默认) | Modbus/RS-485 | USB 转 RS485 适配器 |
| `MODE=can` | CAN/CANFD | ZLG USB-CAN(FD) 设备或 SocketCAN 适配器 |

若使用 ZLG USB-CAN(FD)，请确保已安装 `libusbcanfd.so`。缺失时可运行
`./download-lib.sh` 写入 `dist/shared/linux`，或设置 `ZLG_LIB_DIR=/path/to/lib`。

### 编译标志

构建系统自动包含：
- `-I../../dist/include` - SDK 头文件
- `-L../../dist/lib` - SDK 库
- `-lstark-sdk` - Stark SDK 库
- `-ldl` - 动态加载库（用于 ZLG 运行时加载）
- `-std=c++11` - C++11 标准

### CAN 后端选择（运行时）

所有 CAN 后端默认编译，运行时通过环境变量选择：

```bash
# 构建（默认编译所有后端）
make MODE=can

# 禁用 CAN 支持
make STARK_NO_CAN=1
```

运行时后端选择：

```bash
# SocketCAN（Linux 默认，无第三方依赖）
export STARK_CAN_BACKEND=socketcan
export STARK_SOCKETCAN_IFACE=can0

# ZLG USB-CANFD（动态加载，无编译时依赖）
export STARK_CAN_BACKEND=zlg
export STARK_ZLG_LIB_PATH=/path/to/libusbcanfd.so  # 可选自定义路径
```

典型 CANFD 接口配置示例（仅供参考）：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
```

运行示例：

```bash
# SocketCAN（Linux 默认后端）
STARK_SOCKETCAN_IFACE=can0 make run revo1_can
STARK_SOCKETCAN_IFACE=can0 make run revo2_can_ctrl
STARK_SOCKETCAN_IFACE=can0 make run revo2_canfd

# ZLG 后端
STARK_CAN_BACKEND=zlg make run revo2_canfd
```

## 📖 其他资源

- [官方文档](https://www.brainco-hz.com/docs/revolimb-hand/index.html)
- [ROS/ROS2 集成](https://github.com/BrainCoTech/brainco_hand_ros2)

## 🤝 技术支持

获取技术支持：
- 查看子目录中的示例代码
- 查阅上方的 API 文档
- 联系 BrainCo 技术支持团队

## 📝 注意事项

- 始终在程序开始时调用 `setup_signal_handlers()` 以便更好地调试
- 记得释放分配的结构（`free_device_config`、`free_device_info`、`free_motor_status_data` 等）
- 所有设备的位置值范围为 0（张开）到 1000（闭合）
- 对于 Revo2，在连接设备前使用 `init_cfg()` 初始化
- 触觉设备需要在读取传感器数据前调用 `stark_enable_touch_sensor()`

---

**版本：** 兼容 Stark SDK v1.1.9
