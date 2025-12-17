# Revo2 CAN Python SDK

## Requirement

- Python 3.8~3.12
- Windows 10 build 10.0.15063 or later

## Usage

```shell
cd python

# Install dependencies
(py310) ➜  python git:(main) ✗ pip3 install -r requirements.txt

## CAN Communication Protocol
(py310) ➜  python git:(main) ✗ cd revo2_can

# ZLG USBCAN-FD device, supports Windows and Linux
(py310) ➜  revo2_can git:(main) ✗ python zlgcan_can.py # Read device info, control device

# ZQWL CAN device, currently only Windows dependencies are provided
(py310) ➜  revo2_can git:(main) ✗ python zqwl_can.py # Read device info, control device
# ZLG CAN device, both Windows and Linux are supported
(py310) ➜  revo2_can git:(main) ✗ python zlg_can.py # Read device info, control device
```

