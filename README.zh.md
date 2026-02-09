# BrainCo 灵巧手 SDK 开发示例

[![版本](https://img.shields.io/badge/版本-v1.1.4-blue.svg)](VERSION)
[![许可证](https://img.shields.io/badge/许可证-专有-red.svg)]()

[English](README.md) | [中文](README.zh.md)

本仓库提供了 BrainCo 灵巧手设备（包括 Revo1 和 Revo2 系列）的完整 SDK 开发示例。包含 C++ 和 Python 的即用代码示例，帮助开发者快速集成和控制灵巧机械手。

## 📖 官方文档

详细的技术规格和 API 参考文档，请访问：[BrainCo 灵巧手开发文档](https://www.brainco-hz.com/docs/revolimb-hand/index.html)

## 🚀 快速开始

### 系统要求

- **Python**：3.8 ~ 3.12
- **Linux**：Ubuntu 20.04/22.04 LTS (x86_64/aarch64), glibc ≥ 2.31
- **macOS**：10.15+
- **Windows**：10/11

### 安装步骤

1. 克隆本仓库：
```bash
git clone https://github.com/BrainCoTech/stark-serialport-example.git
cd stark-serialport-example
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

### C++ 示例（跨平台）

跨平台 C++ 开发示例，支持 Linux、macOS 和 Windows：
- **Revo1**：RS-485、CAN
- **Revo2**：RS-485、CAN、CANFD、EtherCAT

详细说明请参考：[C++ 开发指南](c/README.md)

> 💡 `c/demo/` 目录下的 C++ 演示程序与 `python/` 目录下的 Python 示例功能对应，两者提供等效的设备控制、监控和固件升级功能。

### 旧版 C++ 示例（已弃用）

> ⚠️ **已弃用**：以下文件夹将在未来版本中删除，请迁移至 `c/` 文件夹。

- [Linux 示例](linux/) - 旧版 Linux 专用示例
- [Windows 示例](windows/) - 旧版 Windows 专用示例

### ROS/ROS2 集成

ROS/ROS2 集成和示例：[ROS 开发指南](https://github.com/BrainCoTech/brainco_hand_ros2)

## 🔌 支持的通信协议

| 设备型号 | RS-485 | Protobuf | CAN | CANFD | EtherCAT |
|---------|--------|----------|-----|-------|----------|
| Revo1   | ✅     | ✅       | ✅  | ❌    | ❌       |
| Revo2   | ✅     | ❌       | ✅  | ✅    | ✅       |

## 📁 仓库结构

```
.
├── c/                   # ⭐ 跨平台 C++ 示例（推荐）
│   ├── demo/           # 主要演示程序（hand_demo、hand_monitor、hand_dfu）
│   ├── common/         # 共享代码库
│   └── platform/       # 平台特定代码
├── python/             # Python 示例和 SDK
│   ├── demo/           # ⭐ 统一演示程序（hand_demo、hand_monitor、hand_dfu）
│   ├── gui/            # GUI 调试工具
│   ├── revo1/          # Revo1 RS-485 示例
│   ├── revo1_can/      # Revo1 CAN 示例
│   ├── revo2/          # Revo2 RS-485 示例
│   ├── revo2_can/      # Revo2 CAN 示例
│   ├── revo2_canfd/    # Revo2 CANFD 示例
│   └── revo2_ethercat/ # Revo2 EtherCAT 示例
├── linux/              # ⚠️ 已弃用 - 请使用 c/ 文件夹
├── windows/            # ⚠️ 已弃用 - 请使用 c/ 文件夹
├── dll/                # Windows 所需的 DLL 文件
└── dist/               # 发布文件

```

> ⚠️ **弃用通知**：`linux/` 和 `windows/` 文件夹已弃用，将在未来版本中删除。请使用统一的 `c/` 文件夹进行跨平台 C++ 开发。

## 🛠️ 开发指南

### 编译 C++ 示例

详细的编译说明请参考 [Linux README](linux/README.md)。

### 运行 Python 示例

每个示例目录都包含独立的 README 文件，提供具体的使用说明。

## 📝 版本信息

当前 SDK 版本：**v1.1.4**

更新历史请查看 [CHANGELOG](CHANGELOG.md) 文件。

## 🤝 技术支持

如需技术支持和咨询：
- 访问我们的[官方文档](https://www.brainco-hz.com/docs/revolimb-hand/index.html)
- 查看各示例目录中的具体 README 文件
- 联系 BrainCo 技术支持团队

## 📄 许可证

版权所有 © BrainCo Technology。保留所有权利。

---

**注意**：本 SDK 仅供开发和集成使用。使用前请确保您拥有必要的硬件设备和使用权限。
