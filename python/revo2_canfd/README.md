# Revo2 CANFD Python SDK

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

## CANFD Communication Protocol
(py310) ➜  python git:(main) ✗ cd revo2_canfd

# ZLG USBCAN-FD device, supports Windows and Linux
(py310) ➜  revo2_canfd git:(main) ✗ python zlg_canfd.py # Read device info, control device
(py310) ➜  revo2_canfd git:(main) ✗ python zlg_canfd_touch_pressure.py # Pressure-sensitive tactile hand example

# SocketCAN (Linux)
(py310) ➜  revo2_canfd git:(main) ✗ STARK_SOCKETCAN_IFACE=can0 python socketcan_canfd.py # Read device info, control device
(py310) ➜  revo2_canfd git:(main) ✗ STARK_SOCKETCAN_IFACE=can0 STARK_SLAVE_ID=0x7f python socketcan_canfd.py # Select slave id
(py310) ➜  revo2_canfd git:(main) ✗ STARK_SOCKETCAN_IFACE=can0 python socketcan_canfd_touch_pressure.py # Touch pressure example
(py310) ➜  revo2_canfd git:(main) ✗ STARK_SOCKETCAN_IFACE=can0 python socketcan_canfd_dfu.py # Firmware OTA
```
