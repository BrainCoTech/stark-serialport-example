# Revo1 Python SDK

## Usage

```shell
cd python

# Install dependencies
pip3 install -r requirements.txt

cd revo1
# Read device information
python3 revo1_get.py
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
# Upgrade device firmware
python3 dfu.py
```
