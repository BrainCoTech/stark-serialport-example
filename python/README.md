# BrainCo RevoHand Python SDK

[English](README.md) | [中文](README.zh.md)

Complete Python SDK and examples for BrainCo RevoHand devices (Revo1 and Revo2 series).

## 📋 Table of Contents

- [System Requirements](#-system-requirements)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [Communication Protocols](#-communication-protocols)
- [API Reference](#-api-reference)
- [Examples](#-examples)
- [Utilities](#-utilities)

## 💻 System Requirements

- **Python**: 3.8 ~ 3.12
- **Linux**: Ubuntu 20.04/22.04 LTS (x86_64/aarch64), glibc ≥ 2.31
- **macOS**: 10.15+
- **Windows**: 10/11

## 📦 Installation

```bash
cd python

# Install dependencies
pip3 install -r requirements.txt
```

### Dependencies

- `bc-stark-sdk==1.1.5` - BrainCo Stark SDK core library
- `asyncio>=3.4.3` - Asynchronous I/O support
- `colorlog>=6.9.0` - Colored logging output

## 🚀 Quick Start

### Basic Control Example (Revo1)

```python
import asyncio
from revo1_utils import open_modbus_revo1, libstark

async def main():
    # Auto-detect and connect to device
    client, slave_id = await open_modbus_revo1()
    
    # Get device information
    device_info = await client.get_device_info(slave_id)
    print(f"Device: {device_info.description}")
    
    # Control fingers - close grip
    await client.set_finger_positions(slave_id, [600, 600, 1000, 1000, 1000, 1000])
    await asyncio.sleep(1)
    
    # Open fingers
    await client.set_finger_positions(slave_id, [0] * 6)
    
    # Clean up
    libstark.modbus_close(client)

asyncio.run(main())
```

### Basic Control Example (Revo2)

```python
import asyncio
from revo2_utils import open_modbus_revo2, libstark

async def main():
    # Auto-detect and connect to device
    client, slave_id = await open_modbus_revo2()
    
    # Control fingers
    await client.set_finger_positions(slave_id, [400, 400, 1000, 1000, 1000, 1000])
    await asyncio.sleep(1)
    
    # Clean up
    libstark.modbus_close(client)

asyncio.run(main())
```

## 🔌 Communication Protocols

### Revo1 Supported Protocols

| Protocol | Description | Example Directory |
|----------|-------------|-------------------|
| RS-485 (Modbus) | Serial communication via RS-485 | [revo1/](revo1/) |
| CAN | Controller Area Network | [revo1_can/](revo1_can/) |

### Revo2 Supported Protocols

| Protocol | Description | Example Directory |
|----------|-------------|-------------------|
| RS-485 (Modbus) | Serial communication via RS-485 | [revo2/](revo2/) |
| CAN | Controller Area Network | [revo2_can/](revo2_can/) |
| CANFD | CAN with Flexible Data-Rate | [revo2_canfd/](revo2_canfd/) |
| EtherCAT | Industrial Ethernet protocol | [revo2_ethercat/](revo2_ethercat/) |

## 📚 API Reference

### Core SDK Module: `bc_stark_sdk`

Import the SDK:
```python
from bc_stark_sdk import main_mod
libstark = main_mod
```

### Connection Management

#### `auto_detect_modbus_revo1(port_name=None, quick=True)`
Auto-detect and connect to Revo1 device via Modbus.

**Parameters:**
- `port_name` (str, optional): Serial port name. `None` for auto-detection.
- `quick` (bool): Quick detection mode. `True` = faster, `False` = comprehensive.

**Returns:** `(protocol, port_name, baudrate, slave_id)`

**Example:**
```python
protocol, port, baud, slave_id = await libstark.auto_detect_modbus_revo1(None, True)
```

#### `auto_detect_modbus_revo2(port_name=None, quick=True)`
Auto-detect and connect to Revo2 device via Modbus.

**Parameters:** Same as `auto_detect_modbus_revo1`

#### `modbus_open(port_name, baudrate)`
Open Modbus connection with specified parameters.

**Parameters:**
- `port_name` (str): Serial port name (e.g., "/dev/ttyUSB0", "COM3")
- `baudrate` (int): Communication baud rate

**Returns:** `DeviceContext` - Client instance

**Example:**
```python
client = await libstark.modbus_open("/dev/ttyUSB0", 115200)
```

#### `modbus_close(client)`
Close Modbus connection and release resources.

**Parameters:**
- `client` (DeviceContext): Client instance to close

#### `auto_detect()` (New in v1.1.0)
Unified auto-detection for all protocols (Modbus, CAN, CANFD).

**Parameters:**
- `scan_all` (bool): If True, scan for all devices. Default: False
- `port` (str, optional): Specific port to scan. Default: None (scan all)
- `protocol` (str, optional): Protocol to use. Default: None (try all)

**Returns:** `list[DetectedDevice]` - List of detected devices

**Example:**
```python
devices = await libstark.auto_detect()
if devices:
    ctx = await libstark.init_from_detected(devices[0])
```

#### `init_from_detected(device)` (New in v1.1.0)
Initialize device handler from detected device info.

**Parameters:**
- `device` (DetectedDevice): Device from auto_detect()

**Returns:** `DeviceContext` - Ready-to-use device context

#### `init_device_handler(protocol_type, master_id)` (New in v1.1.0)
Initialize device handler for CAN/CANFD/EtherCAT protocols.

**Parameters:**
- `protocol_type` (StarkProtocolType): Protocol type enum
- `master_id` (int): Master ID (default: 0)

**Returns:** `DeviceContext` - Device context

**Example:**
```python
ctx = libstark.init_device_handler(libstark.StarkProtocolType.CanFd, 0)
```

#### `protobuf_open(port_name, slave_id)` (New in v1.1.2)
Open Protobuf protocol connection (Revo1 legacy serial protocol).

**Parameters:**
- `port_name` (str): Serial port name
- `slave_id` (int): Slave ID (default: 10, range: 10-254)

**Returns:** `DeviceContext` - Client instance

**Note:** Protobuf uses fixed baudrate 115200. Position range is automatically converted by SDK.

**Example:**
```python
ctx = await libstark.protobuf_open("/dev/ttyUSB0")  # Default slave_id=10
ctx = await libstark.protobuf_open("/dev/ttyUSB0", 11)  # Custom slave_id
```

#### `init_socketcan_canfd(iface)` / `init_socketcan_can(iface)` (New in v1.1.5)
Initialize SDK built-in SocketCAN support (Linux only).

**Parameters:**
- `iface` (str): CAN interface name (e.g., "can0")

**Example:**
```python
# Initialize SocketCAN CANFD
libstark.init_socketcan_canfd("can0")
ctx = libstark.init_device_handler(libstark.StarkProtocolType.CanFd, 0)

# Initialize SocketCAN CAN 2.0
libstark.init_socketcan_can("can0")
ctx = libstark.init_device_handler(libstark.StarkProtocolType.Can, 0)
```

#### `close_socketcan()` (New in v1.1.5)
Close SocketCAN connection.

#### `socketcan_scan_devices()` (New in v1.1.5)
Scan for devices on SocketCAN interface.

**Returns:** `list[DetectedDevice]` - List of detected devices

### Device Information

#### `client.get_device_info(slave_id)`
Get device information and configuration.

**Returns:** `DeviceInfo` object with properties:
- `description` (str): Device description
- `uses_revo1_motor_api()` (bool): Check if device uses Revo1 Motor API
- `uses_revo2_motor_api()` (bool): Check if device uses Revo2 Motor API
- `uses_revo1_touch_api()` (bool): Check if device uses Revo1 Touch API
- `uses_revo2_touch_api()` (bool): Check if device uses Revo2 Touch API
- `is_touch()` (bool): Check if device has touch sensors

**Example:**
```python
device_info = await client.get_device_info(slave_id)
print(device_info.description)
if device_info.uses_revo2_touch_api():
    print("Revo2 Touch-enabled device")
```

#### `client.get_voltage(slave_id)`
Get device battery voltage.

**Returns:** `float` - Voltage in millivolts (mV)

**Example:**
```python
voltage = await client.get_voltage(slave_id)
print(f"Battery: {voltage:.1f} mV")
```

#### `client.get_serialport_baudrate(slave_id)`
Get current serial port baud rate.

**Returns:** `int` - Baud rate value

### Finger Control

#### `client.set_finger_positions(slave_id, positions)`
Set target positions for all fingers.

**Parameters:**
- `slave_id` (int): Device ID
- `positions` (list[int]): Position values for 6 joints [0-1000]
  - Index 0: Thumb
  - Index 1: Thumb Auxiliary
  - Index 2: Index Finger
  - Index 3: Middle Finger
  - Index 4: Ring Finger
  - Index 5: Pinky Finger

**Position Range:** 0 (fully open) to 1000 (fully closed)

**Example:**
```python
# Close grip
await client.set_finger_positions(slave_id, [600, 600, 1000, 1000, 1000, 1000])

# Open all fingers
await client.set_finger_positions(slave_id, [0, 0, 0, 0, 0, 0])

# Custom positions
await client.set_finger_positions(slave_id, [500, 500, 800, 800, 600, 400])
```

#### `client.set_finger_position(slave_id, finger_id, position)`
Set position for a single finger.

**Parameters:**
- `slave_id` (int): Device ID
- `finger_id` (FingerId): Finger identifier enum
  - `libstark.FingerId.Thumb`
  - `libstark.FingerId.ThumbAux`
  - `libstark.FingerId.Index`
  - `libstark.FingerId.Middle`
  - `libstark.FingerId.Ring`
  - `libstark.FingerId.Pinky`
- `position` (int): Target position [0-1000]

**Example:**
```python
# Close pinky finger only
await client.set_finger_position(slave_id, libstark.FingerId.Pinky, 1000)
```

#### `client.set_finger_speeds(slave_id, speeds)`
Set movement speeds for all fingers (speed control mode).

**Parameters:**
- `slave_id` (int): Device ID
- `speeds` (list[int]): Speed values for 6 joints
  - Positive values: Close direction
  - Negative values: Open direction
  - Range: -1000 to +1000

**Example:**
```python
# Close all fingers at speed 500
await client.set_finger_speeds(slave_id, [500] * 6)

# Open all fingers at speed -500
await client.set_finger_speeds(slave_id, [-500] * 6)
```

### Motor Status

#### `client.get_motor_status(slave_id)`
Get current motor status including positions, currents, and states.

**Returns:** `MotorStatusData` object with properties:
- `positions` (list[int]): Current positions [0-1000] for 6 joints
- `currents` (list[int]): Current values for 6 motors
- `states` (list[int]): Motor state flags for 6 motors
- `description` (str): Human-readable status description
- `is_idle()` (bool): Check if motors are idle
- `is_closed()` (bool): Check if fingers are closed
- `is_opened()` (bool): Check if fingers are opened

**Example:**
```python
status = await client.get_motor_status(slave_id)
print(f"Positions: {list(status.positions)}")
print(f"Currents: {list(status.currents)}")
print(f"Is idle: {status.is_idle()}")
print(f"Is closed: {status.is_closed()}")
```

### Force Control (Revo1 Basic)

#### `client.get_force_level(slave_id)`
Get current force level setting.

**Returns:** `int` - Force level (device-specific range)

#### `client.set_force_level(slave_id, level)`
Set force level for grip strength.

**Parameters:**
- `slave_id` (int): Device ID
- `level` (int): Force level value

**Note:** Only available on non-touch devices. Touch devices use current control.

### Utility Functions

#### Port Detection

```python
from revo1.revo1_utils import get_stark_port_name
# Or when in revo1/ directory:
# from revo1_utils import get_stark_port_name

# Get first available port
port_name = get_stark_port_name()
```

#### Angle/Position Conversion (Revo1)

```python
from revo1.revo1_utils import convert_to_position, convert_to_angle

# Convert angles to position percentages
angles = [30, 45, 35, 35, 35, 35]  # degrees
positions = convert_to_position(angles)  # [0-100]

# Convert position percentages to angles
positions = [50, 50, 50, 50, 50, 50]
angles = convert_to_angle(positions)  # degrees
```

#### Current Conversion (Revo1)

```python
from revo1.revo1_utils import convert_to_mA

# Convert raw current values to milliamps
raw_currents = [100, 120, 110, 115, 105, 108]
currents_mA = convert_to_mA(raw_currents)
```

#### Shutdown Event Handler

```python
from common_utils import setup_shutdown_event

async def main():
    shutdown_event = setup_shutdown_event(logger)
    
    # Your code here...
    
    # Wait for Ctrl+C or shutdown signal
    await shutdown_event.wait()
```

### Logging

```python
from logger import getLogger
import logging

# Get logger with INFO level
logger = getLogger(logging.INFO)

# Get logger with DEBUG level
logger = getLogger(logging.DEBUG)

# Use logger
logger.info("Information message")
logger.debug("Debug message")
logger.warning("Warning message")
logger.error("Error message")
```

Logs are automatically saved to `logs/` directory with timestamps.

## 📂 Examples

### Revo1 Examples

| Example | Description | File |
|---------|-------------|------|
| Auto Control | Automatic grip/open cycle | [revo1_ctrl.py](revo1/revo1_ctrl.py) |
| Dual Hand | Control two hands simultaneously | [revo1_ctrl_dual.py](revo1/revo1_ctrl_dual.py) |
| Multi Hand | Control multiple hands | [revo1_ctrl_multi.py](revo1/revo1_ctrl_multi.py) |
| Action Sequence | Execute predefined action sequences | [revo1_action_seq.py](revo1/revo1_action_seq.py) |
| Configuration | Read/write device configuration | [revo1_cfg.py](revo1/revo1_cfg.py) |
| Touch Sensors | Touch sensor data reading | [revo1_touch.py](revo1/revo1_touch.py) |

**Detailed Guide:** [Revo1 RS-485 README](revo1/README.md)

### Revo1 CAN Examples

**Detailed Guide:** [Revo1 CAN README](revo1_can/README.md)

### Revo2 Examples

| Example | Description | File |
|---------|-------------|------|
| Basic Control | Get device info, control fingers | [revo2_ctrl.py](revo2/revo2_ctrl.py) |
| Left Hand | Control left hand | [revo2_ctrl_left.py](revo2/revo2_ctrl_left.py) |
| Right Hand | Control right hand | [revo2_ctrl_right.py](revo2/revo2_ctrl_right.py) |
| Dual Hand | Control two hands simultaneously | [revo2_ctrl_dual.py](revo2/revo2_ctrl_dual.py) |
| Multi Hand | Control multiple hands | [revo2_ctrl_multi.py](revo2/revo2_ctrl_multi.py) |
| Action Sequence | Execute predefined action sequences | [revo2_action_seq.py](revo2/revo2_action_seq.py) |
| Configuration | Read/write device configuration | [revo2_cfg.py](revo2/revo2_cfg.py) |
| Touch Sensors | Touch sensor data reading | [revo2_touch.py](revo2/revo2_touch.py) |
| Touch Pressure | Pressure sensor data | [revo2_touch_pressure.py](revo2/revo2_touch_pressure.py) |
| Touch Collector | Touch data collection | [revo2_touch_collector.py](revo2/revo2_touch_collector.py) |
| Pressure Collector | Pressure data collection | [revo2_touch_pressure_collector.py](revo2/revo2_touch_pressure_collector.py) |

**Detailed Guide:** [Revo2 RS-485 README](revo2/README.md)

### Revo2 CAN Examples

**Detailed Guide:** [Revo2 CAN README](revo2_can/README.md)

### Revo2 CANFD Examples

Supports ZLG USBCAN-FD and SocketCAN devices.

**Detailed Guide:** [Revo2 CANFD README](revo2_canfd/README.md)

### Revo2 EtherCAT Examples

| Example | Description |
|---------|-------------|
| SDO Operations | Service Data Object read/write |
| PDO Operations | Process Data Object control |
| Firmware Update | OTA via EtherCAT |

**Detailed Guide:** [Revo2 EtherCAT README](revo2_ethercat/README.md)

## 🛠️ Utilities

### Common Utilities (`common_utils.py`)

- `setup_shutdown_event(logger)` - Graceful shutdown handler for async applications

### Logger (`logger.py`)

- RFC3339 timestamp format
- Colored console output
- Automatic file logging to `logs/` directory
- Configurable log levels

### Common Modules

- **common_imports.py** - Unified SDK import, logging, and hardware type helpers
- **common_init.py** - Unified device initialization (`parse_args_and_init`, `DeviceContext`, `cleanup_context`)
- **common_utils.py** - Shutdown event handler, touch sensor print utilities
- **common_socketcan.py** - SocketCAN utilities

### Device-Specific Utilities

- **Revo1**: `revo1/revo1_utils.py` - Connection helpers, angle/position conversion, current conversion
- **Revo2**: `revo2/revo2_utils.py` - Connection helpers, position state checking

## 📖 Additional Resources

- [Official Documentation](https://www.brainco-hz.com/docs/revolimb-hand/index.html)
- [ROS/ROS2 Integration](https://github.com/BrainCoTech/brainco_hand_ros2)

## 🤝 Support

For technical support:
- Check example code in subdirectories
- Review API documentation above
- Contact BrainCo technical support

## 📝 Notes

- All async functions must be called with `await` inside an async context
- Always close connections with `libstark.modbus_close(client)` when done
- Position values range from 0 (open) to 1000 (closed) for all devices
- Touch-enabled devices use current control instead of force levels
- Use quick detection mode for faster connection in production environments

---

**Version:** Compatible with bc-stark-sdk 1.1.5
