# Revo2 Python SDK

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
