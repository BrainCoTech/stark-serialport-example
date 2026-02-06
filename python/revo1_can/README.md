# Revo1 CAN Python SDK

## Requirement

- Python 3.8~3.12
- Linux: Ubuntu 20.04/22.04 LTS (x86_64/aarch64), glibc ≥ 2.31
- macOS: 10.15+ (ZQWL only)
- Windows: 10/11

## Usage

```shell
cd python

# Install dependencies
(py310) ➜  python git:(main) ✗ pip3 install -r requirements.txt

## CAN Communication Protocol
(py310) ➜  python git:(main) ✗ cd revo1_can
# ZQWL CAN device, currently only Windows dependencies are provided
(py310) ➜  revo1_can git:(main) ✗ python zqwl_can.py # Read device info, control device
# ZLG CAN device, both Windows and Linux are supported
(py310) ➜  revo1_can git:(main) ✗ python zlg_can.py # Read device info, control device

# SocketCAN (Linux)
(py310) ➜  revo1_can git:(main) ✗ STARK_SOCKETCAN_IFACE=can0 python socketcan_can.py # Read device info, control device
```
