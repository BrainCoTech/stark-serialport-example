# Revo2 Python SDK (RS-485/Modbus)

## Requirement

- Python 3.8~3.12
- Linux: Ubuntu 20.04/22.04 LTS (x86_64/aarch64), glibc ≥ 2.31
- macOS: 10.15+
- Windows: 10/11

## Usage

```shell
cd python

# Install dependencies
(py310) ➜  python git:(main) ✗ pip3 install -r requirements.txt

(py310) ➜  python git:(main) ✗ cd revo2
# Control/read info - single hand
(py310) ➜  revo2 git:(main) ✗ python revo2_ctrl.py
# Control/read info - single hand (capacitive tactile hand)
(py310) ➜  revo2 git:(main) ✗ python revo2_touch.py
# Control/read info - single hand (pressure-sensitive tactile hand)
(py310) ➜  revo2 git:(main) ✗ python revo2_touch_pressure.py
# Control/read info - multiple hands
(py310) ➜  revo2 git:(main) ✗ python revo2_ctrl_multi.py
# Control dual hands
(py310) ➜  revo2 git:(main) ✗ python revo2_ctrl_dual.py
# Action sequences
(py310) ➜  revo2 git:(main) ✗ python revo2_action_seq.py
# Update configuration, modify device ID, baud rate, Turbo mode, etc.
(py310) ➜  revo2 git:(main) ✗ python revo2_cfg.py
```
