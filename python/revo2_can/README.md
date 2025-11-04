# Revo2 CAN Python SDK

## Requirement

- Python 3.8~3.12
- Windows 10 build 10.0.15063 or later

## Usage

```shell
cd python

# 安装依赖
(py310) ➜  python git:(main) ✗ pip3 install -r requirements.txt

## CAN通信协议
(py310) ➜  python git:(main) ✗ cd revo2_can

# ZLG USBCAN-FD设备, 支持Windows和Linux
(py310) ➜  revo2_can git:(main) ✗ python zlgcan_can.py # 读取设备信息，控制设备

# 智嵌物联厂商CAN设备, 目前仅提供了Windows依赖
# 其他厂商的CAN设备请自行实现读写回调即可，SDK本身支持Windows和Linux
(py310) ➜  revo2_can git:(main) ✗ python zqwl_can.py # 读取设备信息，控制设备
```

