# Revo2 Python SDK

## Usage

```shell
cd python

# 安装依赖
(py310) ➜  python git:(main) ✗ pip3 install -r requirements.txt

## EtherCAT通信协议
(py310) ➜  python git:(main) ✗ cd revo2_ethercat
(py310) ➜  revo2_ethercat git:(main) ✗ python ec_sdo.py # SDO读取/配置
(py310) ➜  revo2_ethercat git:(main) ✗ python ec_pdo.py # PDO读取关节状态，控制设备
(py310) ➜  revo2_ethercat git:(main) ✗ python ec_dfu.py # 固件OTA
```
