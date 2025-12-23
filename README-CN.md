# BrainCo 灵巧手 SDK 开发示例

[![版本](https://img.shields.io/badge/版本-v1.0.1-blue.svg)](VERSION)
[![许可证](https://img.shields.io/badge/许可证-专有-red.svg)]()

[English](README.md) | [中文](README-CN.md)

本仓库提供了 BrainCo 灵巧手设备（包括 Revo1 和 Revo2 系列）的完整 SDK 开发示例。包含 C++ 和 Python 的即用代码示例，帮助开发者快速集成和控制灵巧机械手。

## 📖 官方文档

详细的技术规格和 API 参考文档，请访问：[BrainCo 灵巧手开发文档](https://www.brainco-hz.com/docs/revolimb-hand/index.html)

## 🚀 快速开始

### 系统要求

- **Python 版本**：3.8 ~ 3.12
- **操作系统**：
  - macOS 10.15 或更高版本
  - Windows 10 build 10.0.15063 或更高版本
  - Ubuntu 20.04 LTS 或更高版本

### 安装步骤

1. 克隆本仓库：
```bash
git clone https://github.com/BrainCoTech/brainco_hand_sdk.git
cd brainco_hand_sdk
```

2. Python 开发环境配置：
```bash
cd python
pip3 install -r requirements.txt
```

3. Linux C++ 开发环境配置：
```bash
# 下载所需的库文件
./download-lib.sh
```

## 📚 平台示例

### Python 示例

Python 开发示例支持多种通信协议：
- **Revo1**：RS-485、CAN
- **Revo2**：RS-485、CANFD、EtherCAT

详细说明请参考：[Python 开发指南](python/README.md)

### C++ 示例（Linux/Ubuntu）

Linux 环境下的 C++ 开发示例和编译说明：
- **Revo1**：RS-485、CAN
- **Revo2**：RS-485、CAN、CANFD、EtherCAT

详细说明请参考：[Ubuntu C++ 开发指南](linux/README.md)

### Windows 示例

Windows 平台下的 Revo1 和 Revo2 设备示例。

详见：[Windows 示例](windows/)

### ROS/ROS2 集成

ROS/ROS2 集成和示例：[ROS 开发指南](https://github.com/BrainCoTech/brainco_hand_ros2)

## 🔌 支持的通信协议

| 设备型号 | RS-485 | CAN | CANFD | EtherCAT |
|---------|--------|-----|-------|----------|
| Revo1   | ✅     | ✅  | ❌    | ❌       |
| Revo2   | ✅     | ✅  | ✅    | ✅       |

## 📁 仓库结构

```
.
├── python/             # Python 示例和 SDK
│   ├── revo1/          # Revo1 RS-485 示例
│   ├── revo1_can/      # Revo1 CAN 示例
│   ├── revo2/          # Revo2 RS-485 示例
│   ├── revo2_canfd/    # Revo2 CANFD 示例
│   └── revo2_ethercat/ # Revo2 EtherCAT 示例
├── linux/              # Linux C++ 示例
│   ├── revo1/          # Revo1 示例
│   ├── revo2/          # Revo2 CAN/CANFD 示例
│   └── revo2_ec/       # Revo2 EtherCAT 示例
├── windows/            # Windows 平台示例
├── dll/                # Windows 所需的 DLL 文件
└── dist/               # 发布文件

```

## 🛠️ 开发指南

### 编译 C++ 示例

详细的编译说明请参考 [Linux README](linux/README.md)。

### 运行 Python 示例

每个示例目录都包含独立的 README 文件，提供具体的使用说明。

## 📝 版本信息

当前 SDK 版本：**v1.0.1**

更新历史请查看 [VERSION](VERSION) 文件。

## 🤝 技术支持

如需技术支持和咨询：
- 访问我们的[官方文档](https://www.brainco-hz.com/docs/revolimb-hand/index.html)
- 查看各示例目录中的具体 README 文件
- 联系 BrainCo 技术支持团队

## 📄 许可证

版权所有 © BrainCo Technology。保留所有权利。

---

**注意**：本 SDK 仅供开发和集成使用。使用前请确保您拥有必要的硬件设备和使用权限。
