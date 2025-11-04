# Revo2 Python SDK

## Requirement

- Python 3.8~3.12
- Windows 10 build 10.0.15063 or later

## Usage

```shell
cd python

# 安装依赖
(py310) ➜  python git:(main) ✗ pip3 install -r requirements.txt

## CANFD通信协议
(py310) ➜  python git:(main) ✗ cd revo2_canfd

# ZLG USBCAN-FD设备, 支持Windows和Linux
(py310) ➜  revo2_canfd git:(main) ✗ python zlg_canfd.py # 读取设备信息，控制设备
(py310) ➜  revo2_canfd git:(main) ✗ python zlg_canfd_dfu.py # 固件OTA
(py310) ➜  revo2_canfd git:(main) ✗ python zlg_canfd_touch_pressure.py # 压感式触觉手示例

# 智嵌物联厂商CANFD设备, 目前仅提供了Windows依赖
# 其他厂商的CANFD设备请自行实现读写回调即可，SDK本身支持Windows和Linux
(py310) ➜  revo2_canfd git:(main) ✗ python zqwl_canfd.py # 读取设备信息，控制设备
(py310) ➜  revo2_canfd git:(main) ✗ python zqwl_canfd_dfu.py # 固件OTA
```
