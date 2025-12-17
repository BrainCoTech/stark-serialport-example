# Revo2 Python SDK

## Requirement

- Python 3.8~3.12
- Windows 10 build 10.0.15063 or later

## Usage

```shell
cd python

# Install dependencies
(py310) ➜  python git:(main) ✗ pip3 install -r requirements.txt

## CANFD Communication Protocol
(py310) ➜  python git:(main) ✗ cd revo2_canfd

# ZLG USBCAN-FD device, supports Windows and Linux
(py310) ➜  revo2_canfd git:(main) ✗ python zlg_canfd.py # Read device info, control device
(py310) ➜  revo2_canfd git:(main) ✗ python zlg_canfd_dfu.py # Firmware OTA
(py310) ➜  revo2_canfd git:(main) ✗ python zlg_canfd_touch_pressure.py # Pressure-sensitive tactile hand example

# ZQWL CANFD device, currently only Windows dependencies are provided
(py310) ➜  revo2_canfd git:(main) ✗ python zqwl_canfd.py # Read device info, control device
(py310) ➜  revo2_canfd git:(main) ✗ python zqwl_canfd_dfu.py # Firmware OTA

# ZLG CAN device, both Windows and Linux are supported
(py310) ➜  revo2_canfd git:(main) ✗ python zlg_can.py # Read device info, control device
(py310) ➜  revo2_canfd git:(main) ✗ python zlg_can_dfu.py # Firmware OTA
(py310) ➜  revo2_canfd git:(main) ✗ python zlg_can_touch_pressure.py # Pressure-sensitive tactile hand example
```
