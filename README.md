# BrainCo RevoHand SDK Examples

[![Version](https://img.shields.io/badge/version-v1.1.5-blue.svg)](VERSION)
[![License](https://img.shields.io/badge/license-Proprietary-red.svg)]()

[English](README.md) | [中文](README.zh.md)

This repository provides comprehensive SDK examples for BrainCo RevoHand devices, including both Revo1 and Revo2 series. It contains ready-to-use code samples in C++ and Python to help developers quickly integrate and control the dexterous robotic hand.

## 📖 Official Documentation

For detailed technical specifications and API references, visit: [BrainCo RevoHand Documentation](https://www.brainco-hz.com/docs/revolimb-hand/index.html)

## 🚀 Quick Start

### System Requirements

- **Python**: 3.8 ~ 3.12
- **Linux**: Ubuntu 20.04/22.04 LTS (x86_64/aarch64), glibc ≥ 2.31
- **macOS**: 10.15+
- **Windows**: 10/11

### Installation

1. Clone this repository:
```bash
git clone https://github.com/BrainCoTech/stark-serialport-example.git
cd stark-serialport-example
```

2. For Python development:
```bash
cd python
pip3 install -r requirements.txt
```

3. For C++ development on Linux:
```bash
# Download required libraries
./download-lib.sh
```

## 📚 Examples by Platform

### Python Examples

Python development examples support multiple communication protocols:
- **Revo1**: RS-485, CAN
- **Revo2**: RS-485, CANFD, EtherCAT

For detailed instructions: [Python Development Guide](python/README.md)

### C++ Examples (Cross-platform)

Cross-platform C++ development examples supporting Linux, macOS, and Windows:
- **Revo1**: RS-485, CAN
- **Revo2**: RS-485, CAN, CANFD, EtherCAT

For detailed instructions: [C++ Development Guide](c/README.md)

> 💡 The C++ demos in `c/demo/` correspond to the Python demos in `python/` - both provide equivalent functionality for device control, monitoring, and firmware upgrade.

### Legacy C++ Examples (Deprecated)

> ⚠️ **Deprecated**: The following folders will be removed in future versions. Please migrate to the `c/` folder.

- [Linux Examples](linux/) - Legacy Linux-specific examples
- [Windows Examples](windows/) - Legacy Windows-specific examples

### ROS/ROS2 Integration

For ROS/ROS2 integration and examples: [ROS Development Guide](https://github.com/BrainCoTech/brainco_hand_ros2)

## 🔌 Supported Communication Protocols

| Device | RS-485 | Protobuf | CAN | CANFD | EtherCAT |
|--------|--------|----------|-----|-------|----------|
| Revo1  | ✅     | ✅       | ✅  | ❌    | ❌       |
| Revo2  | ✅     | ❌       | ✅  | ✅    | ✅       |

## 📁 Repository Structure

```
.
├── c/                   # ⭐ Cross-platform C++ examples (recommended)
│   ├── demo/           # Main demos (hand_demo, hand_monitor, hand_dfu)
│   ├── common/         # Shared code library
│   └── platform/       # Platform-specific code
├── python/              # Python examples and SDK
│   ├── demo/           # ⭐ Unified demos (hand_demo, hand_monitor, hand_dfu)
│   ├── gui/            # GUI debugging tool
│   ├── revo1/          # Revo1 RS-485 examples
│   ├── revo1_can/      # Revo1 CAN examples
│   ├── revo2/          # Revo2 RS-485 examples
│   ├── revo2_can/      # Revo2 CAN examples
│   ├── revo2_canfd/    # Revo2 CANFD examples
│   ├── revo2_ethercat/ # Revo2 EtherCAT examples
│   └── revo2_tactile_grasp/ # Revo2 tactile grasping examples
├── linux/              # ⚠️ DEPRECATED - use c/ folder instead
├── windows/            # ⚠️ DEPRECATED - use c/ folder instead
├── dll/                # Required DLL files for Windows
└── dist/               # Distribution files

```

> ⚠️ **Deprecation Notice**: The `linux/` and `windows/` folders are deprecated and will be removed in future versions. Please use the unified `c/` folder for cross-platform C++ development.

## 🛠️ Development

### Building C++ Examples

Refer to the [Linux README](linux/README.md) for detailed compilation instructions.

### Running Python Examples

Each example directory contains its own README with specific usage instructions.

## 📝 Version

Current SDK Version: **v1.1.5**

See [CHANGELOG](CHANGELOG.md) for update history.

## 🤝 Support

For technical support and questions:
- Visit our [official documentation](https://www.brainco-hz.com/docs/revolimb-hand/index.html)
- Check the specific README files in each example directory
- Contact BrainCo technical support

## 📄 License

Copyright © BrainCo Technology. All rights reserved.

---

**Note**: This SDK is provided for development and integration purposes. Please ensure you have the necessary hardware and permissions before use.
