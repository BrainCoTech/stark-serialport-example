# Revo1 Python SDK (RS-485/Modbus)

## Requirement

- Python 3.8~3.12
- Linux: Ubuntu 20.04/22.04 LTS (x86_64/aarch64), glibc ≥ 2.31
- macOS: 10.15+
- Windows: 10/11

## Usage

```shell
cd python

# Install dependencies
pip3 install -r requirements.txt

cd revo1
# Control device - single dexterous hand
python3 revo1_ctrl.py
# Control device - multiple hands on one bus
python3 revo1_ctrl_multi.py
# Action sequences, supports 6 built-in gestures and 6 custom gestures
python3 revo1_action_seq.py
# Update configuration, modify slave_id, baud rate, Turbo mode, etc.
python3 revo1_cfg.py
# Tactile version
python3 revo1_touch.py
```
