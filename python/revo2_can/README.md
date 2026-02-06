# Revo2 CAN Python SDK

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
(py310) ➜  python git:(main) ✗ cd revo2_can

# ZLG CAN device, supports Windows and Linux
(py310) ➜  revo2_can git:(main) ✗ python zlg_can.py # Read device info, control device

# SocketCAN (Linux)
(py310) ➜  revo2_can git:(main) ✗ STARK_SOCKETCAN_IFACE=can0 python socketcan_can.py # Read device info, control device
```
