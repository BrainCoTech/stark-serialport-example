# Python

## Requirement

- Python 3.8 or later
- Mac 10.15 or later
- Windows 10 build 10.0.15063 or later
- Linux Ubuntu 20.04 LTS or later

## Usage

```shell
# 安装依赖
pip3 install -r requirements.txt

cd protobuf_example
# 读取设备信息
python3 example_get.py
# 控制设备运动
python3 example_control.py
python3 example_control_broadcast.py
# 更新设备配置
python3 example_update_cfg.py
# 升级设备
python3 example_ota.py
# 内部/工厂测试
python3 example_factory.py

cd modbus_example
# 读取设备信息
python3 example_Modbus_get.py
# 控制设备运动
python3 example_Modbus_control.py
# 更新设备配置
python3 example_Modbus_update_cfg.py
# 升级设备
python3 example_Modbus_ota.py
# 内部/工厂测试
python3 example_Modbus_factory.py
```
