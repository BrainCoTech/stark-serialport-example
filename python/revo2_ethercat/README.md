# Revo2 EtherCAT Python SDK

## Requirement

- Python 3.8~3.12
- Linux: Ubuntu 20.04/22.04 LTS (x86_64/aarch64), glibc ≥ 2.31
- EtherCAT Master (IgH EtherCAT Master) installed and configured

## Usage

```shell
cd python

# Install dependencies
(py310) ➜  python git:(main) ✗ pip3 install -r requirements.txt

## EtherCAT Communication Protocol
(py310) ➜  python git:(main) ✗ cd revo2_ethercat
(py310) ➜  revo2_ethercat git:(main) ✗ python ec_sdo.py # SDO read/configure
(py310) ➜  revo2_ethercat git:(main) ✗ python ec_pdo.py # PDO read joint status, control device
(py310) ➜  revo2_ethercat git:(main) ✗ python ec_dfu.py # Firmware OTA
```
