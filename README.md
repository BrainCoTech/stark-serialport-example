# BrainCo RevoHand SDK Examples

[![Version](https://img.shields.io/badge/version-v1.0.1-blue.svg)](VERSION)
[![License](https://img.shields.io/badge/license-Proprietary-red.svg)]()

[English](README.md) | [中文](README-CN.md)

This repository provides comprehensive SDK examples for BrainCo RevoHand devices, including both Revo1 and Revo2 series. It contains ready-to-use code samples in C++ and Python to help developers quickly integrate and control the dexterous robotic hand.

## 📖 Official Documentation

For detailed technical specifications and API references, visit: [BrainCo RevoHand Documentation](https://www.brainco-hz.com/docs/revolimb-hand/index.html)

## 🚀 Quick Start

### System Requirements

- **Python**: 3.8 ~ 3.12
- **Operating Systems**:
  - macOS 10.15 or later
  - Windows 10 build 10.0.15063 or later
  - Ubuntu 20.04 LTS or later

### Installation

1. Clone this repository:
```bash
git clone https://github.com/BrainCoTech/brainco_hand_sdk.git
cd brainco_hand_sdk
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

### C++ Examples (Linux/Ubuntu)

C++ development examples and compilation instructions for Linux environments:
- **Revo1**: RS-485, CAN
- **Revo2**: RS-485, CAN, CANFD, EtherCAT

For detailed instructions: [Ubuntu C++ Development Guide](linux/README.md)

### Windows Examples

Windows-specific examples for Revo1 and Revo2 devices.

See: [Windows Examples](windows/)

### ROS/ROS2 Integration

For ROS/ROS2 integration and examples: [ROS Development Guide](https://github.com/BrainCoTech/brainco_hand_ros2)

## 🔌 Supported Communication Protocols

| Device | RS-485 | CAN | CANFD | EtherCAT |
|--------|--------|-----|-------|----------|
| Revo1  | ✅     | ✅  | ❌    | ❌       |
| Revo2  | ✅     | ✅  | ✅    | ✅       |

## 📁 Repository Structure

```
.
├── python/              # Python examples and SDK
│   ├── revo1/          # Revo1 RS-485 examples
│   ├── revo1_can/      # Revo1 CAN examples
│   ├── revo2/          # Revo2 RS-485 examples
│   ├── revo2_canfd/    # Revo2 CANFD examples
│   └── revo2_ethercat/ # Revo2 EtherCAT examples
├── linux/              # C++ examples for Linux
│   ├── revo1/          # Revo1 examples
│   ├── revo2/          # Revo2 CAN/CANFD examples
│   └── revo2_ec/       # Revo2 EtherCAT examples
├── windows/            # Windows-specific examples
├── dll/                # Required DLL files for Windows
└── dist/               # Distribution files

```

## 🛠️ Development

### Building C++ Examples

Refer to the [Linux README](linux/README.md) for detailed compilation instructions.

### Running Python Examples

Each example directory contains its own README with specific usage instructions.

## 📝 Version

Current SDK Version: **v1.0.1**

See [VERSION](VERSION) file for update history.

## 🤝 Support

For technical support and questions:
- Visit our [official documentation](https://www.brainco-hz.com/docs/revolimb-hand/index.html)
- Check the specific README files in each example directory
- Contact BrainCo technical support

## 📄 License

Copyright © BrainCo Technology. All rights reserved.

---

**Note**: This SDK is provided for development and integration purposes. Please ensure you have the necessary hardware and permissions before use.
