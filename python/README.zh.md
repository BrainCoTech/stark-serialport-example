# BrainCo 灵巧手 Python SDK

[English](README.md) | [中文](README.zh.md)

BrainCo 灵巧手设备（Revo1 和 Revo2 系列）的完整 Python SDK 和示例。

## 📋 目录

- [系统要求](#-系统要求)
- [安装](#-安装)
- [快速开始](#-快速开始)
- [通信协议](#-通信协议)
- [API 参考](#-api-参考)
- [示例程序](#-示例程序)
- [工具函数](#-工具函数)

## 💻 系统要求

- **Python**：3.8 ~ 3.12
- **Linux**：Ubuntu 20.04/22.04 LTS (x86_64/aarch64), glibc ≥ 2.31
- **macOS**：10.15+
- **Windows**：10/11

## 📦 安装

```bash
cd python

# 安装依赖
pip3 install -r requirements.txt
```

### 依赖包

- `bc-stark-sdk==1.1.9` - BrainCo Stark SDK 核心库
- `asyncio>=3.4.3` - 异步 I/O 支持
- `colorlog>=6.9.0` - 彩色日志输出

## 🚀 快速开始

### 基础控制示例（Revo1）

```python
import asyncio
from revo1_utils import open_modbus_revo1, libstark

async def main():
    # 自动检测并连接设备
    client, slave_id = await open_modbus_revo1()
    
    # 获取设备信息
    device_info = await client.get_device_info(slave_id)
    print(f"设备: {device_info.description}")
    
    # 控制手指 - 闭合抓握
    await client.set_finger_positions(slave_id, [600, 600, 1000, 1000, 1000, 1000])
    await asyncio.sleep(1)
    
    # 张开手指
    await client.set_finger_positions(slave_id, [0] * 6)
    
    # 清理资源
    libstark.modbus_close(client)

asyncio.run(main())
```

### 基础控制示例（Revo2）

```python
import asyncio
from revo2_utils import open_modbus_revo2, libstark

async def main():
    # 自动检测并连接设备
    client, slave_id = await open_modbus_revo2()
    
    # 控制手指
    await client.set_finger_positions(slave_id, [400, 400, 1000, 1000, 1000, 1000])
    await asyncio.sleep(1)
    
    # 清理资源
    libstark.modbus_close(client)

asyncio.run(main())
```

## 🔌 通信协议

### Revo1 支持的协议

| 协议 | 说明 | 示例目录 |
|------|------|---------|
| RS-485 (Modbus) | 通过 RS-485 串口通信 | [revo1/](revo1/) |
| CAN | 控制器局域网络 | [revo1_can/](revo1_can/) |

### Revo2 支持的协议

| 协议 | 说明 | 示例目录 |
|------|------|---------|
| RS-485 (Modbus) | 通过 RS-485 串口通信 | [revo2/](revo2/) |
| CAN | 控制器局域网络 | [revo2_can/](revo2_can/) |
| CANFD | 灵活数据速率 CAN | [revo2_canfd/](revo2_canfd/) |
| EtherCAT | 工业以太网协议 | [revo2_ethercat/](revo2_ethercat/) |

## 📚 API 参考

### 核心 SDK 模块：`bc_stark_sdk`

导入 SDK：
```python
from bc_stark_sdk import main_mod
libstark = main_mod
```

### 连接管理

#### `auto_detect_modbus_revo1(port_name=None, quick=True)`
自动检测并通过 Modbus 连接 Revo1 设备。

**参数：**
- `port_name` (str, 可选)：串口名称。`None` 表示自动检测。
- `quick` (bool)：快速检测模式。`True` = 更快，`False` = 更全面。

**返回值：** `(protocol, port_name, baudrate, slave_id)`

**示例：**
```python
protocol, port, baud, slave_id = await libstark.auto_detect_modbus_revo1(None, True)
```

#### `auto_detect_modbus_revo2(port_name=None, quick=True)`
自动检测并通过 Modbus 连接 Revo2 设备。

**参数：** 与 `auto_detect_modbus_revo1` 相同

#### `modbus_open(port_name, baudrate)`
使用指定参数打开 Modbus 连接。

**参数：**
- `port_name` (str)：串口名称（例如："/dev/ttyUSB0"、"COM3"）
- `baudrate` (int)：通信波特率

**返回值：** `DeviceContext` - 客户端实例

**示例：**
```python
client = await libstark.modbus_open("/dev/ttyUSB0", 115200)
```

#### `modbus_close(client)`
关闭 Modbus 连接并释放资源。

**参数：**
- `client` (DeviceContext)：要关闭的客户端实例

#### `auto_detect()` (v1.1.0 新增)
统一自动检测所有协议（Modbus、CAN、CANFD）。

**参数：**
- `scan_all` (bool)：如果为 True，扫描所有设备。默认：False
- `port` (str, 可选)：指定扫描的端口。默认：None（扫描所有）
- `protocol` (str, 可选)：使用的协议。默认：None（尝试所有）

**返回值：** `list[DetectedDevice]` - 检测到的设备列表

**示例：**
```python
devices = await libstark.auto_detect()
if devices:
    ctx = await libstark.init_from_detected(devices[0])
```

#### `init_from_detected(device)` (v1.1.0 新增)
从检测到的设备信息初始化设备处理器。

**参数：**
- `device` (DetectedDevice)：来自 auto_detect() 的设备

**返回值：** `DeviceContext` - 可直接使用的设备上下文

#### `init_device_handler(protocol_type, master_id)` (v1.1.0 新增)
为 CAN/CANFD/EtherCAT 协议初始化设备处理器。

**参数：**
- `protocol_type` (StarkProtocolType)：协议类型枚举
- `master_id` (int)：主站 ID（默认：0）

**返回值：** `DeviceContext` - 设备上下文

**示例：**
```python
ctx = libstark.init_device_handler(libstark.StarkProtocolType.CanFd, 0)
```

#### `protobuf_open(port_name, slave_id)` (v1.1.2 新增)
打开 Protobuf 协议连接（Revo1 旧版串口协议）。

**参数：**
- `port_name` (str)：串口名称
- `slave_id` (int)：从站 ID（默认：10，范围：10-254）

**返回值：** `DeviceContext` - 客户端实例

**注意：** Protobuf 使用固定波特率 115200。位置范围由 SDK 自动转换。

**示例：**
```python
ctx = await libstark.protobuf_open("/dev/ttyUSB0")  # 默认 slave_id=10
ctx = await libstark.protobuf_open("/dev/ttyUSB0", 11)  # 自定义 slave_id
```

#### `init_socketcan_canfd(iface)` / `init_socketcan_can(iface)` (v1.1.9 新增)
初始化 SDK 内置 SocketCAN 支持（仅限 Linux）。

**参数：**
- `iface` (str)：CAN 接口名称（例如："can0"）

**示例：**
```python
# 初始化 SocketCAN CANFD
libstark.init_socketcan_canfd("can0")
ctx = libstark.init_device_handler(libstark.StarkProtocolType.CanFd, 0)

# 初始化 SocketCAN CAN 2.0
libstark.init_socketcan_can("can0")
ctx = libstark.init_device_handler(libstark.StarkProtocolType.Can, 0)
```

#### `close_socketcan()` (v1.1.9 新增)
关闭 SocketCAN 连接。

#### `socketcan_scan_devices()` (v1.1.9 新增)
扫描 SocketCAN 接口上的设备。

**返回值：** `list[DetectedDevice]` - 检测到的设备列表

### 设备信息

#### `client.get_device_info(slave_id)`
获取设备信息和配置。

**返回值：** `DeviceInfo` 对象，包含以下属性：
- `description` (str)：设备描述
- `uses_revo1_motor_api()` (bool)：检查是否使用 Revo1 电机 API
- `uses_revo2_motor_api()` (bool)：检查是否使用 Revo2 电机 API
- `uses_revo1_touch_api()` (bool)：检查是否使用 Revo1 触觉 API
- `uses_revo2_touch_api()` (bool)：检查是否使用 Revo2 触觉 API
- `is_touch()` (bool)：检查是否有触觉传感器

**示例：**
```python
device_info = await client.get_device_info(slave_id)
print(device_info.description)
if device_info.uses_revo2_touch_api():
    print("Revo2 触觉版本设备")
```

#### `client.get_voltage(slave_id)`
获取设备电池电压。

**返回值：** `float` - 电压值（毫伏 mV）

**示例：**
```python
voltage = await client.get_voltage(slave_id)
print(f"电池电压: {voltage:.1f} mV")
```

#### `client.get_serialport_baudrate(slave_id)`
获取当前串口波特率。

**返回值：** `int` - 波特率值

### 手指控制

#### `client.set_finger_positions(slave_id, positions)`
设置所有手指的目标位置。

**参数：**
- `slave_id` (int)：设备 ID
- `positions` (list[int])：6 个关节的位置值 [0-1000]
  - 索引 0：大拇指
  - 索引 1：大拇指辅助
  - 索引 2：食指
  - 索引 3：中指
  - 索引 4：无名指
  - 索引 5：小指

**位置范围：** 0（完全张开）到 1000（完全闭合）

**示例：**
```python
# 闭合抓握
await client.set_finger_positions(slave_id, [600, 600, 1000, 1000, 1000, 1000])

# 张开所有手指
await client.set_finger_positions(slave_id, [0, 0, 0, 0, 0, 0])

# 自定义位置
await client.set_finger_positions(slave_id, [500, 500, 800, 800, 600, 400])
```

#### `client.set_finger_position(slave_id, finger_id, position)`
设置单个手指的位置。

**参数：**
- `slave_id` (int)：设备 ID
- `finger_id` (FingerId)：手指标识符枚举
  - `libstark.FingerId.Thumb` - 大拇指
  - `libstark.FingerId.ThumbAux` - 大拇指辅助
  - `libstark.FingerId.Index` - 食指
  - `libstark.FingerId.Middle` - 中指
  - `libstark.FingerId.Ring` - 无名指
  - `libstark.FingerId.Pinky` - 小指
- `position` (int)：目标位置 [0-1000]

**示例：**
```python
# 仅闭合小指
await client.set_finger_position(slave_id, libstark.FingerId.Pinky, 1000)
```

#### `client.set_finger_speeds(slave_id, speeds)`
设置所有手指的运动速度（速度控制模式）。

**参数：**
- `slave_id` (int)：设备 ID
- `speeds` (list[int])：6 个关节的速度值
  - 正值：闭合方向
  - 负值：张开方向
  - 范围：-1000 到 +1000

**示例：**
```python
# 以速度 500 闭合所有手指
await client.set_finger_speeds(slave_id, [500] * 6)

# 以速度 -500 张开所有手指
await client.set_finger_speeds(slave_id, [-500] * 6)
```

### 电机状态

#### `client.get_motor_status(slave_id)`
获取当前电机状态，包括位置、电流和状态。

**返回值：** `MotorStatusData` 对象，包含以下属性：
- `positions` (list[int])：6 个关节的当前位置 [0-1000]
- `currents` (list[int])：6 个电机的电流值
- `states` (list[int])：6 个电机的状态标志
- `description` (str)：人类可读的状态描述
- `is_idle()` (bool)：检查电机是否空闲
- `is_closed()` (bool)：检查手指是否闭合
- `is_opened()` (bool)：检查手指是否张开

**示例：**
```python
status = await client.get_motor_status(slave_id)
print(f"位置: {list(status.positions)}")
print(f"电流: {list(status.currents)}")
print(f"是否空闲: {status.is_idle()}")
print(f"是否闭合: {status.is_closed()}")
```

### 力度控制（Revo1基础版）

#### `client.get_force_level(slave_id)`
获取当前力度等级。

**返回值：** `int` - 力度等级

#### `client.set_force_level(slave_id, level)`
设置力度等级。

**参数：**
- `slave_id` (int)：设备 ID
- `level` (int)：力度等级值

**注意：** 仅适用于非触觉设备。触觉设备使用电流控制。

### 工具函数

#### 端口检测

```python
from revo1.revo1_utils import get_stark_port_name
# 或者在 revo1/ 目录下：
# from revo1_utils import get_stark_port_name

# 获取第一个可用端口
port_name = get_stark_port_name()
```

#### 角度/位置转换（Revo1）

```python
from revo1.revo1_utils import convert_to_position, convert_to_angle

# 将角度转换为位置百分比
angles = [30, 45, 35, 35, 35, 35]  # 度
positions = convert_to_position(angles)  # [0-100]

# 将位置百分比转换为角度
positions = [50, 50, 50, 50, 50, 50]
angles = convert_to_angle(positions)  # 度
```

#### 电流转换（Revo1）

```python
from revo1.revo1_utils import convert_to_mA

# 将原始电流值转换为毫安
raw_currents = [100, 120, 110, 115, 105, 108]
currents_mA = convert_to_mA(raw_currents)
```

#### 关闭事件处理器

```python
from common_utils import setup_shutdown_event

async def main():
    shutdown_event = setup_shutdown_event(logger)
    
    # 你的代码...
    
    # 等待 Ctrl+C 或关闭信号
    await shutdown_event.wait()
```

### 日志记录

```python
from logger import getLogger
import logging

# 获取 INFO 级别的日志记录器
logger = getLogger(logging.INFO)

# 获取 DEBUG 级别的日志记录器
logger = getLogger(logging.DEBUG)

# 使用日志记录器
logger.info("信息消息")
logger.debug("调试消息")
logger.warning("警告消息")
logger.error("错误消息")
```

日志会自动保存到 `logs/` 目录，文件名带有时间戳。

## 📂 示例程序

### Revo1 示例

| 示例 | 说明 | 文件 |
|------|------|------|
| 自动控制 | 自动抓握/张开循环 | [revo1_ctrl.py](revo1/revo1_ctrl.py) |
| 双手控制 | 同时控制两只手 | [revo1_ctrl_dual.py](revo1/revo1_ctrl_dual.py) |
| 多手控制 | 控制多只手 | [revo1_ctrl_multi.py](revo1/revo1_ctrl_multi.py) |
| 动作序列 | 执行预定义动作序列 | [revo1_action_seq.py](revo1/revo1_action_seq.py) |
| 配置管理 | 读取/写入设备配置 | [revo1_cfg.py](revo1/revo1_cfg.py) |
| 触觉传感器 | 读取触觉传感器数据 | [revo1_touch.py](revo1/revo1_touch.py) |

**详细指南：** [Revo1 RS-485 README](revo1/README.md)

### Revo1 CAN 示例

**详细指南：** [Revo1 CAN README](revo1_can/README.md)

### Revo2 示例

| 示例 | 说明 | 文件 |
|------|------|------|
| 基础控制 | 获取设备信息，控制手指 | [revo2_ctrl.py](revo2/revo2_ctrl.py) |
| 左手控制 | 控制左手 | [revo2_ctrl_left.py](revo2/revo2_ctrl_left.py) |
| 右手控制 | 控制右手 | [revo2_ctrl_right.py](revo2/revo2_ctrl_right.py) |
| 双手控制 | 同时控制两只手 | [revo2_ctrl_dual.py](revo2/revo2_ctrl_dual.py) |
| 多手控制 | 控制多只手 | [revo2_ctrl_multi.py](revo2/revo2_ctrl_multi.py) |
| 动作序列 | 执行预定义动作序列 | [revo2_action_seq.py](revo2/revo2_action_seq.py) |
| 配置管理 | 读取/写入设备配置 | [revo2_cfg.py](revo2/revo2_cfg.py) |
| 触觉传感器 | 读取触觉传感器数据 | [revo2_touch.py](revo2/revo2_touch.py) |
| 触觉压力 | 压力传感器数据 | [revo2_touch_pressure.py](revo2/revo2_touch_pressure.py) |
| 触觉数据采集 | 触觉数据采集 | [revo2_touch_collector.py](revo2/revo2_touch_collector.py) |
| 压力数据采集 | 压力数据采集 | [revo2_touch_pressure_collector.py](revo2/revo2_touch_pressure_collector.py) |

**详细指南：** [Revo2 RS-485 README](revo2/README.md)

### Revo2 CAN 示例

**详细指南：** [Revo2 CAN README](revo2_can/README.md)

### Revo2 CANFD 示例

支持 ZLG USBCAN-FD 和 SocketCAN 设备。

**详细指南：** [Revo2 CANFD README](revo2_canfd/README.md)

### Revo2 EtherCAT 示例

| 示例 | 说明 |
|------|------|
| SDO 操作 | 服务数据对象读写 |
| PDO 操作 | 过程数据对象控制 |
| 固件更新 | 通过 EtherCAT 进行 OTA |

**详细指南：** [Revo2 EtherCAT README](revo2_ethercat/README.md)

## 🛠️ 工具函数

### 通用工具（`common_utils.py`）

- `setup_shutdown_event(logger)` - 异步应用的优雅关闭处理器

### 日志记录器（`logger.py`）

- RFC3339 时间戳格式
- 彩色控制台输出
- 自动文件日志记录到 `logs/` 目录
- 可配置的日志级别

### 通用模块

- **common_imports.py** - 统一的 SDK 导入、日志和硬件类型辅助函数
- **common_init.py** - 统一的设备初始化（`parse_args_and_init`、`DeviceContext`、`cleanup_context`）
- **common_utils.py** - 关闭事件处理器、触觉传感器打印工具
- **common_socketcan.py** - SocketCAN 工具函数

### 设备特定工具

- **Revo1**：`revo1/revo1_utils.py` - 连接辅助函数、角度/位置转换、电流转换
- **Revo2**：`revo2/revo2_utils.py` - 连接辅助函数、位置状态检查

## 📖 其他资源

- [官方文档](https://www.brainco-hz.com/docs/revolimb-hand/index.html)
- [ROS/ROS2 集成](https://github.com/BrainCoTech/brainco_hand_ros2)

## 🤝 技术支持

获取技术支持：
- 查看子目录中的示例代码
- 查阅上方的 API 文档
- 联系 BrainCo 技术支持团队

## 📝 注意事项

- 所有异步函数必须在异步上下文中使用 `await` 调用
- 使用完毕后务必使用 `libstark.modbus_close(client)` 关闭连接
- 所有设备的位置值范围为 0（张开）到 1000（闭合）
- 触觉版本设备使用电流控制而非力度等级
- 在生产环境中使用快速检测模式以加快连接速度

---

**版本：** 兼容 bc-stark-sdk 1.1.9
