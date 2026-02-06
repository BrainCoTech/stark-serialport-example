# Python Demo Examples

Unified demo programs with auto-detect device and protocol.
Supports: Modbus (RS485), Protobuf, CAN 2.0, CANFD, SocketCAN, ZLG

## Examples

| Example | Description |
|---------|-------------|
| `hand_demo.py` | Comprehensive demo (8 modes) |
| `hand_monitor.py` | High-performance data monitor |
| `hand_dfu.py` | Firmware upgrade |
| `auto_detect.py` | Device detection |

## Quick Start

```bash
cd python/demo

# Comprehensive demo
python hand_demo.py              # Auto-detect, interactive menu
python hand_demo.py 1            # Run specific demo (1-8)
python hand_demo.py 0            # Run all demos

# Real-time monitor
python hand_monitor.py           # Auto-select mode
python hand_monitor.py motor     # Motor data (100Hz)
python hand_monitor.py touch     # Motor + touch

# Firmware upgrade
python hand_dfu.py firmware.bin

# Device detection
python auto_detect.py
```

## Initialization Options

All demos support the same initialization options:

```bash
# Auto-detect (recommended)
python hand_demo.py

# Modbus (RS485)
python hand_demo.py -m /dev/ttyUSB0 460800 127

# Protobuf (serial, 115200 baud)
python hand_demo.py -p /dev/ttyUSB0           # Default slave_id=10
python hand_demo.py -p /dev/ttyUSB0 10        # Custom slave_id

# ZQWL CAN 2.0
python hand_demo.py -c /dev/ttyUSB0 1000000 1

# ZQWL CANFD
python hand_demo.py -f /dev/ttyUSB0 1000000 5000000 127

# SocketCAN CAN 2.0 - SDK built-in (Linux, recommended)
python hand_demo.py -b can0 1

# SocketCAN CANFD - SDK built-in (Linux, recommended)
python hand_demo.py -B can0 127

# SocketCAN CAN 2.0 - external adapter (Linux)
python hand_demo.py -s can0 1

# SocketCAN CANFD - external adapter (Linux)
python hand_demo.py -S can0 127

# ZLG CAN 2.0 (Linux)
python hand_demo.py -z 2

# ZLG CANFD (Linux)
python hand_demo.py -Z 127
```


## hand_demo - Demo Modes

| Mode | Function | Device |
|------|----------|--------|
| 1 | Position control | All |
| 2 | Speed/current control | All |
| 3 | Advanced control (position+time/speed) | Revo2 |
| 4 | Action sequence | All |
| 5 | Configuration info | All |
| 6 | Touch sensor | Touch versions |
| 7 | Interactive loop | All |
| 8 | Multi-device control | All |

## hand_monitor - Monitor Modes

| Mode | Data | Frequency | Device |
|------|------|-----------|--------|
| `motor` | Position/speed/current | 100Hz | All |
| `touch` | + Capacitive touch | 100Hz + 10Hz | Touch versions |
| `summary` | + Pressure summary | 100Hz + 10Hz | Pressure versions |
| `detailed` | + Pressure per sensor | 100Hz + 10Hz | Pressure versions |

## CAN Adapter Guide

### ZQWL (Default)

SDK built-in support with auto-detection:

```bash
python hand_demo.py           # Auto-detect
python hand_demo.py -c /dev/ttyUSB0 1000000 1   # Manual CAN 2.0
python hand_demo.py -f /dev/ttyUSB0 1000000 5000000 127  # Manual CANFD
```

### SocketCAN (Linux)

Two implementations available:

**SDK Built-in (recommended)**

No external adapter needed, uses SDK's native SocketCAN support:

```bash
# Configure interface
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Auto-detect (includes SocketCAN)
python hand_demo.py

# Manual specify - SDK built-in
python hand_demo.py -b can0 1       # CAN 2.0
python hand_demo.py -B can0 127     # CANFD
```

**External Adapter**

Uses `revo2_can/socketcan_can.py` adapter:

```bash
# Manual specify - external adapter
python hand_demo.py -s can0 1       # CAN 2.0
python hand_demo.py -S can0 127     # CANFD
```

> **ZLG USB-CAN/CANFD with SocketCAN**: See [../../c/platform/linux/README.md](../../c/platform/linux/README.md) for driver installation.

### ZLG (Linux)

```bash
python hand_demo.py -z 2            # ZLG CAN 2.0
python hand_demo.py -Z 127          # ZLG CANFD
```

## Device ID Reference

| Protocol | Default IDs | Devices |
|----------|-------------|---------|
| CAN 2.0 | 1 (left), 2 (right) | Revo1 |
| CANFD | 126/0x7E (left), 127/0x7F (right) | Revo2 |
| Modbus | 1-247 (configurable) | All |
| Protobuf | 10-254 (default 10) | Revo1 Protobuf |

## Protobuf Protocol Notes

Protobuf protocol has some differences from Modbus/CAN:

- **Fixed baudrate**: 115200 (cannot be changed)
- **Slave ID range**: 10-254 (default 10)
- **Position range**: SDK automatically converts 0-1000 to internal 0-100
- **No single finger control**: Must use `set_finger_positions()` with all 6 fingers
- **Multi-device**: Same serial port can control multiple devices with different slave_ids

```python
# Protobuf initialization
ctx = await sdk.protobuf_open("/dev/ttyUSB0")  # Default slave_id=10
ctx = await sdk.protobuf_open("/dev/ttyUSB0", slave_id=11)  # Custom slave_id

# Control multiple devices on same bus
status1 = await ctx.get_motor_status(10)  # Device 1
status2 = await ctx.get_motor_status(11)  # Device 2
```

## Requirements

```bash
pip install bc_stark_sdk
```
