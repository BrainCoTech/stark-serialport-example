# Linux 专用 C++ 示例

[English](README.md) | [中文](README.zh.md)

此目录包含仅限 Linux 的示例，需要 macOS/Windows 上不可用的库。

## 目录结构

```
platform/linux/
├── revo2/           # 使用 Stark SDK 的 EtherCAT 示例（需要 libethercat）
└── revo2_ec/        # 独立 EtherCAT 库（不依赖 SDK）
```

## 跨平台示例

大多数示例已整合到跨平台 demo 目录：

```bash
# 跨平台示例（Linux/macOS/Windows）
cd ../../demo/
make
./hand_demo.exe    # 自动检测设备和协议
```

## Linux 专用功能

### ZLG USB-CAN/CANFD SocketCAN 驱动

ZLG USB-CAN/CANFD 适配器可以通过 Linux SocketCAN 子系统使用。

官方文档: https://manual.zlg.cn/web/#/42/13456

#### 1. 下载驱动

从 ZLG 官网下载 SocketCAN 驱动: https://manual.zlg.cn/web/#/42/13456

#### 2. 解压驱动

```bash
unzip usbcanfd200_400u_2.10.zip
cd usbcanfd200_400u_2.10
```

#### 3. 加载驱动

```bash
# 加载依赖模块
sudo modprobe can-dev

# 如果遇到 "Unknown symbol in module" 错误，先加载 sja1000 模块
sudo modprobe sja1000

# 加载 ZLG SocketCAN 驱动
sudo insmod usbcanfd.ko

# 卸载驱动
sudo rmmod usbcanfd
```

> **注意**: SocketCAN 驱动和 ZLG 官方 DLL 是两种独立的使用方式：
> - **SocketCAN 驱动**: 通过 Linux 内核 CAN 子系统，使用 `ip link` 和 `candump` 等标准工具
> - **ZLG 官方 DLL**: 通过 `libusbcanfd.so` 直接访问 USB 设备
>
> 两者不能同时使用。如果加载了 SocketCAN 驱动，ZLG DLL 将无法打开设备；反之亦然。
> 使用 DLL 前需要先卸载 SocketCAN 驱动：`sudo rmmod usbcanfd`

#### 4. 验证设备

```bash
# 检查 USB 设备是否识别
lsusb | grep -E "04cc:1240|3068:0009"

# 检查 CAN 接口是否创建
ls /sys/class/net | grep can
```

#### 5. 配置 CAN 接口

```bash
# CAN 2.0 模式
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# CANFD 模式（推荐）
sudo ip link set can0 type can fd on bitrate 1000000 dbitrate 5000000 sample-point 0.75 dsample-point 0.75
sudo ip link set can0 up
```

#### 6. 使用 candump 测试

```bash
# 监控 CAN 总线数据
candump can0 -t A
```

### EtherCAT

```bash
cd revo2/
make
make run
```

要求：
- 已安装 libethercat.so
- 已配置 EtherCAT 主站
- 适当的权限

### SocketCAN

SocketCAN 在 Linux 上默认编译，运行时选择：

```bash
cd ../../demo/

# SocketCAN 是 Linux 默认后端
export STARK_SOCKETCAN_IFACE=can0
./hand_demo.exe -s can0 1       # CAN 2.0
./hand_demo.exe -S can0 127     # CANFD
```

### ZLG USB-CAN/CANFD

ZLG 适配器支持（运行时需要 libusbcanfd.so）：

```bash
cd ../../demo/

# 运行时选择 ZLG 后端
STARK_CAN_BACKEND=zlg ./hand_demo.exe -z 1            # ZLG CAN 2.0
STARK_CAN_BACKEND=zlg ./hand_demo.exe -Z 127          # ZLG CANFD
```

## EtherCAT 库 (revo2_ec/)

独立 EtherCAT 库和示例。**不依赖 Stark SDK** - 直接使用 IgH EtherCAT Master。

| 示例 | 说明 |
|------|------|
| revo2_sdo.cpp | SDO 读写操作 |
| revo2_pdo.cpp | PDO 周期控制 |
| revo2_multi_pdo.cpp | 多从站 PDO 控制 |
| revo2_touch_sdo.cpp | 通过 SDO 读取触觉传感器 |
| revo2_touch_pdo.cpp | 通过 PDO 读取触觉传感器 |
| revo2_touch_pressure_pdo.cpp | 压力传感器数据 |

详见 [revo2_ec/README.md](revo2_ec/README.md)。
