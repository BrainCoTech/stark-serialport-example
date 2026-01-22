# BrainCo RevoHand C++ SDK for Linux/Ubuntu

[English](README.md) | [中文](README.zh.md)

Complete C++ SDK and examples for BrainCo RevoHand devices on Linux/Ubuntu platforms.

## 📋 Table of Contents

- [System Requirements](#-system-requirements)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [Communication Protocols](#-communication-protocols)
- [API Reference](#-api-reference)
- [Examples](#-examples)
- [Build System](#-build-system)

## 💻 System Requirements

- **Operating System**: Ubuntu 20.04 LTS or later
- **Compiler**: GCC with C++11 support
- **Build Tools**: make, pkg-config
- **Dependencies**: Stark SDK libraries (auto-downloaded)

## 📦 Installation

### Download SDK Libraries

First, download the required Stark SDK libraries:

```bash
# From project root directory
rm VERSION
./download-lib.sh
```

This script will download the latest SDK libraries to the `dist/` directory.

### Verify Installation

```bash
# Check if libraries are downloaded
ls dist/
# Should see: libstark-sdk.so and other library files
```

## 🚀 Quick Start

### Basic Control Example (Revo1)

```cpp
#include "stark-sdk.h"
#include "stark_common.h"
#include <unistd.h>

int main() {
    // Auto-detect and connect to device
    auto cfg = auto_detect_modbus_revo1(NULL, true);
    if (cfg == NULL) return -1;
    
    auto handle = modbus_open(cfg->port_name, cfg->baudrate);
    uint8_t slave_id = cfg->slave_id;
    free_device_config(cfg);
    
    // Control fingers - close grip
    uint16_t positions_fist[] = {500, 500, 1000, 1000, 1000, 1000};
    stark_set_finger_positions(handle, slave_id, positions_fist, 6);
    usleep(1000 * 1000); // Wait 1 second
    
    // Open fingers
    uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};
    stark_set_finger_positions(handle, slave_id, positions_open, 6);
    
    return 0;
}
```

### Basic Control Example (Revo2)

```cpp
#include "stark-sdk.h"
#include "stark_common.h"
#include <unistd.h>

int main() {
    // Initialize configuration
    init_cfg(STARK_PROTOCOL_TYPE_MODBUS, LOG_LEVEL_INFO);
    
    // Auto-detect and connect to device
    auto cfg = auto_detect_modbus_revo2("/dev/ttyUSB0", true);
    if (cfg == NULL) return -1;
    
    auto handle = modbus_open(cfg->port_name, cfg->baudrate);
    uint8_t slave_id = cfg->slave_id;
    free_device_config(cfg);
    
    // Control fingers
    uint16_t positions[] = {400, 400, 1000, 1000, 1000, 1000};
    stark_set_finger_positions(handle, slave_id, positions, 6);
    
    return 0;
}
```

## 🔌 Communication Protocols

### Revo1 Supported Protocols

| Protocol | Description | Example Directory | Hardware Required |
|----------|-------------|-------------------|-------------------|
| RS-485 (Modbus) | Serial communication | [revo1/](revo1/) | USB-to-RS485 adapter |
| CAN | Controller Area Network | [revo1/](revo1/) | ZLG USB-CAN device or SocketCAN |

### Revo2 Supported Protocols

| Protocol | Description | Example Directory | Hardware Required |
|----------|-------------|-------------------|-------------------|
| RS-485 (Modbus) | Serial communication | [revo2/](revo2/) | USB-to-RS485 adapter |
| CAN | Controller Area Network | [revo2/](revo2/) | ZLG USB-CAN device or SocketCAN |
| CANFD | CAN with Flexible Data-Rate | [revo2/](revo2/) | ZLG USB-CANFD device or SocketCAN |
| EtherCAT | Industrial Ethernet | [revo2_ec/](revo2_ec/) | EtherCAT master |

## 📚 API Reference

### Core SDK Header: `stark-sdk.h`

Include the SDK in your code:
```cpp
#include "stark-sdk.h"
```

### Initialization and Configuration

#### `init_cfg(protocol_type, log_level)`
Initialize SDK configuration (Revo2 only).

**Parameters:**
- `protocol_type` (StarkProtocolType): Protocol type
  - `STARK_PROTOCOL_TYPE_MODBUS`
  - `STARK_PROTOCOL_TYPE_CAN`
  - `STARK_PROTOCOL_TYPE_CANFD`
  - `STARK_PROTOCOL_TYPE_ETHERCAT`
- `log_level` (LogLevel): Logging level
  - `LOG_LEVEL_DEBUG`
  - `LOG_LEVEL_INFO`
  - `LOG_LEVEL_WARN`
  - `LOG_LEVEL_ERROR`

**Example:**
```cpp
init_cfg(STARK_PROTOCOL_TYPE_MODBUS, LOG_LEVEL_INFO);
```

### Connection Management

#### `auto_detect_modbus_revo1(port_name, quick)`
Auto-detect and connect to Revo1 device via Modbus.

**Parameters:**
- `port_name` (const char*): Serial port name. `NULL` for auto-detection.
- `quick` (bool): Quick detection mode. `true` = faster, `false` = comprehensive.

**Returns:** `DeviceConfig*` - Configuration structure (must be freed with `free_device_config`)

**Example:**
```cpp
auto cfg = auto_detect_modbus_revo1(NULL, true);
if (cfg != NULL) {
    auto handle = modbus_open(cfg->port_name, cfg->baudrate);
    uint8_t slave_id = cfg->slave_id;
    free_device_config(cfg);
}
```

#### `auto_detect_modbus_revo2(port_name, quick)`
Auto-detect and connect to Revo2 device via Modbus.

**Parameters:** Same as `auto_detect_modbus_revo1`

#### `modbus_open(port_name, baudrate)`
Open Modbus connection with specified parameters.

**Parameters:**
- `port_name` (const char*): Serial port name (e.g., "/dev/ttyUSB0")
- `baudrate` (int): Communication baud rate

**Returns:** `DeviceHandler*` - Device handler instance

**Example:**
```cpp
auto handle = modbus_open("/dev/ttyUSB0", 115200);
```

#### `free_device_config(config)`
Free device configuration structure.

**Parameters:**
- `config` (DeviceConfig*): Configuration to free

### Device Information

#### `stark_get_device_info(handle, slave_id)`
Get device information and configuration.

**Returns:** `DeviceInfo*` - Device information structure (must be freed with `free_device_info`)

**DeviceInfo Structure:**
- `serial_number` (char*): Device serial number
- `firmware_version` (char*): Firmware version string
- `hardware_type` (StarkHardwareType): Hardware type enum
  - `STARK_HARDWARE_TYPE_REVO1`
  - `STARK_HARDWARE_TYPE_REVO1_TOUCH`
  - `STARK_HARDWARE_TYPE_REVO2`
  - `STARK_HARDWARE_TYPE_REVO2_TOUCH`
- `sku_type` (StarkSkuType): SKU type

**Example:**
```cpp
auto info = stark_get_device_info(handle, slave_id);
if (info != NULL) {
    printf("Serial: %s, Firmware: %s\n", 
           info->serial_number, info->firmware_version);
    
    if (info->hardware_type == STARK_HARDWARE_TYPE_REVO2_TOUCH) {
        printf("Touch-enabled device\n");
    }
    free_device_info(info);
}
```

#### `stark_get_voltage(handle, slave_id)`
Get device battery voltage.

**Returns:** `uint16_t` - Voltage in millivolts (mV)

**Example:**
```cpp
uint16_t voltage = stark_get_voltage(handle, slave_id);
printf("Battery: %hu mV\n", voltage);
```

#### `stark_get_rs485_baudrate(handle, slave_id)`
Get current RS-485 baud rate.

**Returns:** `uint32_t` - Baud rate value

#### `stark_get_canfd_baudrate(handle, slave_id)`
Get current CANFD baud rate.

**Returns:** `uint32_t` - Baud rate value

### Finger Control

#### Finger ID Enumeration

```cpp
typedef enum {
    STARK_FINGER_ID_THUMB = 0,      // Thumb
    STARK_FINGER_ID_THUMB_AUX = 1,  // Thumb Auxiliary
    STARK_FINGER_ID_INDEX = 2,      // Index Finger
    STARK_FINGER_ID_MIDDLE = 3,     // Middle Finger
    STARK_FINGER_ID_RING = 4,       // Ring Finger
    STARK_FINGER_ID_PINKY = 5       // Pinky Finger
} StarkFingerId;
```

#### `stark_set_finger_positions(handle, slave_id, positions, count)`
Set target positions for all fingers.

**Parameters:**
- `handle` (DeviceHandler*): Device handler
- `slave_id` (uint8_t): Device ID
- `positions` (uint16_t[]): Position values for 6 joints [0-1000]
- `count` (size_t): Number of positions (should be 6)

**Position Range:** 0 (fully open) to 1000 (fully closed)

**Example:**
```cpp
// Close grip
uint16_t positions_fist[] = {600, 600, 1000, 1000, 1000, 1000};
stark_set_finger_positions(handle, slave_id, positions_fist, 6);

// Open all fingers
uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};
stark_set_finger_positions(handle, slave_id, positions_open, 6);
```

#### `stark_set_finger_position(handle, slave_id, finger_id, position)`
Set position for a single finger.

**Parameters:**
- `handle` (DeviceHandler*): Device handler
- `slave_id` (uint8_t): Device ID
- `finger_id` (StarkFingerId): Finger identifier
- `position` (uint16_t): Target position [0-1000]

**Example:**
```cpp
// Close pinky finger only
stark_set_finger_position(handle, slave_id, STARK_FINGER_ID_PINKY, 1000);
```

#### `stark_set_finger_speeds(handle, slave_id, speeds, count)` (Revo1)
Set movement speeds for all fingers.

**Parameters:**
- `speeds` (int16_t[]): Speed values for 6 joints
  - Positive: Close direction
  - Negative: Open direction
  - Range: -1000 to 1000

**Example:**
```cpp
int16_t speeds[] = {100, 100, 100, 100, 100, 100};
stark_set_finger_speeds(handle, slave_id, speeds, 6);
```

#### `stark_set_finger_speed(handle, slave_id, finger_id, speed)` (Revo2)
Set movement speed for a single finger.

**Parameters:**
- `speed` (int16_t): Speed value [-1000 to 1000]

**Example:**
```cpp
stark_set_finger_speed(handle, slave_id, STARK_FINGER_ID_MIDDLE, 500);
```

#### `stark_set_finger_current(handle, slave_id, finger_id, current)` (Revo2)
Set current for a single finger (current control mode).

**Parameters:**
- `current` (int16_t): Current value [-1000 to 1000]

**Example:**
```cpp
stark_set_finger_current(handle, slave_id, STARK_FINGER_ID_INDEX, -300);
```

#### `stark_set_finger_currents(handle, slave_id, currents, count)` (Revo2)
Set currents for all fingers.

**Parameters:**
- `currents` (int16_t[]): Current values for 6 joints [-1000 to 1000]

**Example:**
```cpp
int16_t currents[] = {-300, -300, -300, -300, -300, -300};
stark_set_finger_currents(handle, slave_id, currents, 6);
```

#### `stark_set_finger_pwm(handle, slave_id, finger_id, pwm)` (Revo2)
Set PWM for a single finger (PWM control mode).

**Parameters:**
- `pwm` (int16_t): PWM value [-1000 to 1000]

#### `stark_set_finger_pwms(handle, slave_id, pwms, count)` (Revo2)
Set PWMs for all fingers.

**Parameters:**
- `pwms` (int16_t[]): PWM values for 6 joints [-1000 to 1000]

#### `stark_set_finger_position_with_speed(handle, slave_id, finger_id, position, speed)` (Revo2)
Set position with specified speed for a single finger.

**Parameters:**
- `position` (uint16_t): Target position [0-1000]
- `speed` (uint16_t): Movement speed [0-1000]

**Example:**
```cpp
stark_set_finger_position_with_speed(handle, slave_id, 
                                     STARK_FINGER_ID_MIDDLE, 1000, 50);
```

#### `stark_set_finger_positions_and_speeds(handle, slave_id, positions, speeds, count)` (Revo2)
Set positions with speeds for all fingers.

**Parameters:**
- `positions` (uint16_t[]): Target positions for 6 joints
- `speeds` (uint16_t[]): Movement speeds for 6 joints

**Example:**
```cpp
uint16_t positions[] = {300, 300, 500, 500, 500, 500};
uint16_t speeds[] = {500, 500, 500, 500, 500, 500};
stark_set_finger_positions_and_speeds(handle, slave_id, positions, speeds, 6);
```

#### `stark_set_finger_position_with_millis(handle, slave_id, finger_id, position, duration)` (Revo2)
Set position with specified duration for a single finger.

**Parameters:**
- `position` (uint16_t): Target position [0-1000]
- `duration` (uint16_t): Movement duration in milliseconds

**Example:**
```cpp
stark_set_finger_position_with_millis(handle, slave_id, 
                                      STARK_FINGER_ID_THUMB, 1000, 1000);
```

#### `stark_set_finger_positions_and_durations(handle, slave_id, positions, durations, count)` (Revo2)
Set positions with durations for all fingers.

**Parameters:**
- `positions` (uint16_t[]): Target positions for 6 joints
- `durations` (uint16_t[]): Movement durations in milliseconds

### Motor Status

#### `stark_get_motor_status(handle, slave_id)`
Get current motor status including positions, speeds, currents, and states.

**Returns:** `MotorStatusData*` - Motor status structure (must be freed with `free_motor_status_data`)

**MotorStatusData Structure:**
- `positions[6]` (uint16_t): Current positions [0-1000]
- `speeds[6]` (int16_t): Current speeds
- `currents[6]` (int16_t): Current values
- `states[6]` (uint8_t): Motor state flags

**Example:**
```cpp
auto status = stark_get_motor_status(handle, slave_id);
if (status != NULL) {
    printf("Positions: %hu, %hu, %hu, %hu, %hu, %hu\n",
           status->positions[0], status->positions[1],
           status->positions[2], status->positions[3],
           status->positions[4], status->positions[5]);
    printf("Currents: %hd, %hd, %hd, %hd, %hd, %hd\n",
           status->currents[0], status->currents[1],
           status->currents[2], status->currents[3],
           status->currents[4], status->currents[5]);
    free_motor_status_data(status);
}
```

### Finger Configuration (Revo2)

#### `stark_set_finger_unit_mode(handle, slave_id, mode)`
Set unit mode for finger control parameters.

**Parameters:**
- `mode` (FingerUnitMode):
  - `FINGER_UNIT_MODE_NORMALIZED` - Normalized values [0-1000]
  - `FINGER_UNIT_MODE_PHYSICAL` - Physical units (degrees, mm/s, mA)

#### `stark_get_finger_unit_mode(handle, slave_id)`
Get current unit mode.

**Returns:** `FingerUnitMode`

#### `stark_set_finger_min_position(handle, slave_id, finger_id, min_pos)`
Set minimum position limit for a finger.

#### `stark_get_finger_min_position(handle, slave_id, finger_id)`
Get minimum position limit.

**Returns:** `uint16_t` - Minimum position value

#### `stark_set_finger_max_position(handle, slave_id, finger_id, max_pos)`
Set maximum position limit for a finger.

#### `stark_get_finger_max_position(handle, slave_id, finger_id)`
Get maximum position limit.

#### `stark_set_finger_max_speed(handle, slave_id, finger_id, max_speed)`
Set maximum speed limit for a finger.

#### `stark_get_finger_max_speed(handle, slave_id, finger_id)`
Get maximum speed limit.

#### `stark_set_finger_max_current(handle, slave_id, finger_id, max_current)`
Set maximum current limit for a finger.

#### `stark_get_finger_max_current(handle, slave_id, finger_id)`
Get maximum current limit.

#### `stark_set_finger_protected_current(handle, slave_id, finger_id, protected_current)`
Set protection current threshold for a finger.

#### `stark_get_finger_protected_current(handle, slave_id, finger_id)`
Get protection current threshold.

### Touch Sensors (Touch Devices)

#### `stark_enable_touch_sensor(handle, slave_id, mask)`
Enable touch sensors.

**Parameters:**
- `mask` (uint8_t): Bitmask for enabling sensors
  - Bit 0: Thumb
  - Bit 1: Index
  - Bit 2: Middle
  - Bit 3: Ring
  - Bit 4: Pinky
  - `0x1F`: Enable all sensors

**Example:**
```cpp
// Enable all touch sensors
stark_enable_touch_sensor(handle, slave_id, 0x1F);
```

### LED Control

#### `stark_get_led_info(handle, slave_id)`
Get LED status information.

**Returns:** `LedInfo*` - LED information structure (must be freed with `free_led_info`)

**LedInfo Structure:**
- `mode` (uint8_t): LED mode
- `color` (uint8_t): LED color

### Button Events

#### `stark_get_button_event(handle, slave_id)`
Get button event information.

**Returns:** `ButtonEvent*` - Button event structure (must be freed with `free_button_event`)

**ButtonEvent Structure:**
- `timestamp` (int32_t): Event timestamp
- `button_id` (int32_t): Button identifier
- `press_state` (uint8_t): Press state

### Utility Functions (stark_common.h)

#### `setup_signal_handlers()`
Setup signal handlers for crash debugging.

**Example:**
```cpp
setup_signal_handlers();
```

#### `verify_device_is_revo1(handle, slave_id)`
Verify device is Revo1 and print device info.

**Returns:** `bool` - `true` if device is Revo1

#### `verify_device_is_revo2(handle, slave_id)`
Verify device is Revo2 and print device info.

**Returns:** `bool` - `true` if device is Revo2

#### `get_and_print_device_info(handle, slave_id)`
Get and print device information.

**Returns:** `bool` - `true` if successful

#### `get_and_print_extended_info(handle, slave_id)`
Get and print extended device information (baudrate, voltage, LED, button).

## 📂 Examples

### Revo1 Examples

| Example | Description | File |
|---------|-------------|------|
| Basic Control | Get device info, control fingers | [revo1_ctrl.cpp](revo1/revo1_ctrl.cpp) |
| Multi Hand | Control multiple hands | [revo1_ctrl_multi.cpp](revo1/revo1_ctrl_multi.cpp) |
| CAN Control | Control via CAN protocol | [revo1_can.cpp](revo1/revo1_can.cpp) |
| Custom Modbus | Custom Modbus implementation | [revo1_customed_modbus.cpp](revo1/revo1_customed_modbus.cpp) |
| Async Modbus | Asynchronous Modbus control | [revo1_customed_modbus_async.cpp](revo1/revo1_customed_modbus_async.cpp) |
| Firmware Update | OTA firmware update | [revo1_dfu.cpp](revo1/revo1_dfu.cpp) |
| Touch Sensors | Touch sensor data reading | [revo1_touch.cpp](revo1/revo1_touch.cpp) |
| Motor Collector | Motor data collection | [revo1_basic_collector.cpp](revo1/revo1_basic_collector.cpp) |
| Touch Collector | Touch data collection | [revo1_touch_collector.cpp](revo1/revo1_touch_collector.cpp) |

**Detailed Guide:** [Revo1 README](revo1/README.md)

### Revo2 Examples

| Example | Description | File |
|---------|-------------|------|
| Basic Control | Get device info, control fingers | [revo2_ctrl.cpp](revo2/revo2_ctrl.cpp) |
| Multi Hand | Control multiple hands | [revo2_ctrl_multi.cpp](revo2/revo2_ctrl_multi.cpp) |
| CAN Control | Control via CAN protocol | [revo2_can_ctrl.cpp](revo2/revo2_can_ctrl.cpp) |
| CANFD Control | Control via CANFD protocol | [revo2_canfd.cpp](revo2/revo2_canfd.cpp) |
| CANFD Touch | Touch control via CANFD | [revo2_canfd_touch.cpp](revo2/revo2_canfd_touch.cpp) |
| Custom Modbus | Custom Modbus implementation | [revo2_customed_modbus.cpp](revo2/revo2_customed_modbus.cpp) |
| Async Modbus | Asynchronous Modbus control | [revo2_customed_modbus_async.cpp](revo2/revo2_customed_modbus_async.cpp) |
| Firmware Update | OTA firmware update | [revo2_dfu.cpp](revo2/revo2_dfu.cpp) |
| Touch Sensors | Touch sensor data reading | [revo2_touch.cpp](revo2/revo2_touch.cpp) |
| EtherCAT | EtherCAT protocol example | [revo2_ethercat.cpp](revo2/revo2_ethercat.cpp) |
| **Motor Collector** | **Motor data collection** | [revo2_basic_collector.cpp](revo2/revo2_basic_collector.cpp) |
| **Touch Collector** | **Touch data collection with trajectory** | [revo2_touch_collector.cpp](revo2/revo2_touch_collector.cpp) |
| **Pressure Collector** | **Pressure sensor data collection** | [revo2_touch_pressure_collector.cpp](revo2/revo2_touch_pressure_collector.cpp) |

**Detailed Guide:** [Revo2 README](revo2/README.md)

### Revo2 EtherCAT Examples

| Example | Description | File |
|---------|-------------|------|
| SDO Operations | Service Data Object read/write | [revo2_sdo.cpp](revo2_ec/revo2_sdo.cpp) |
| PDO Operations | Process Data Object control | [revo2_pdo.cpp](revo2_ec/revo2_pdo.cpp) |
| Multi-Hand PDO | Control multiple hands via PDO | [revo2_multi_pdo.cpp](revo2_ec/revo2_multi_pdo.cpp) |
| Touch SDO | Touch sensor via SDO | [revo2_touch_sdo.cpp](revo2_ec/revo2_touch_sdo.cpp) |
| Touch PDO | Touch sensor via PDO | [revo2_touch_pdo.cpp](revo2_ec/revo2_touch_pdo.cpp) |
| Touch Pressure | Pressure sensor data | [revo2_touch_pressure_pdo.cpp](revo2_ec/revo2_touch_pressure_pdo.cpp) |

**Detailed Guide:** [Revo2 EtherCAT README](revo2_ec/README.md)

## 🛠️ Build System

### Smart Build & Run

The Makefile provides intelligent build commands that auto-detect the correct mode:

```bash
# Smart compile + run (auto-detects MODE)
make run revo1_ctrl           # Modbus programs (default)
make run revo1_can            # Auto-detects CAN mode
make run revo2_ctrl           # Modbus programs
make run revo2_canfd          # Auto-detects CANFD mode
make run revo2_ethercat       # Auto-detects EtherCAT mode

make run revo1_basic_collector   # Run Revo1 motor collector example
make run revo1_touch_collector   # Run Revo1 touch collector example
make run revo2_basic_collector   # Run Revo2 motor collector example
make run revo2_touch_collector   # Run Revo2 touch collector example

# Show available targets
make run                      # Shows usage help
```

### Traditional Build Commands

```bash
# Clean build artifacts
make clean

# Build with specific modes
make                          # Build with default mode (Modbus)
make MODE=can                 # Build with CAN interface mode
make MODE=ethercat            # Build with EtherCAT interface mode

# Run only (must be compiled first)
make run_revo1_ctrl           # Run revo1_ctrl example
make run_revo2_ctrl           # Run revo2_ctrl example
```

### Build Modes

| Mode | Description | Required Hardware |
|------|-------------|-------------------|
| (default) | Modbus/RS-485 | USB-to-RS485 adapter |
| `MODE=can` | CAN/CANFD | ZLG USB-CAN(FD) device or SocketCAN adapter |
| `MODE=ethercat` | EtherCAT | EtherCAT master |

For ZLG USB-CAN(FD), ensure `libusbcanfd.so` is installed. If missing, run `./download-lib.sh`
to populate `dist/shared/linux` or set `ZLG_LIB_DIR=/path/to/lib`.

### Compilation Flags

The build system automatically includes:
- `-I../../dist/include` - SDK headers
- `-L../../dist/lib` - SDK libraries
- `-lstark-sdk` - Stark SDK library
- `-lusbcanfd` - USB-CANFD library (CAN mode, ZLG backend only)
- `-std=c++11` - C++11 standard

### SocketCAN Backend (Linux)

Use SocketCAN for standard Linux CAN/CANFD interfaces (e.g., `can0`, `can1`, `vcan0`).

```bash
# Build with both backends (ZLG + SocketCAN)
make MODE=can

# Build with ZLG USB-CAN(FD) backend only
make MODE=can CAN_BACKEND=zlg

# Build without ZLG USB-CANFD library
make MODE=can CAN_BACKEND=socketcan

# Build with both backends (ZLG + SocketCAN)
make MODE=can CAN_BACKEND=both

# Select backend + interface at runtime
export STARK_CAN_BACKEND=socketcan
export STARK_SOCKETCAN_IFACE=can0
```

Typical CANFD interface setup (example only):

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
```

Run examples with SocketCAN:

```bash
# CAN
STARK_CAN_BACKEND=socketcan STARK_SOCKETCAN_IFACE=can0 make run revo1_can
STARK_CAN_BACKEND=socketcan STARK_SOCKETCAN_IFACE=can0 make run revo2_can_ctrl

# CANFD
STARK_CAN_BACKEND=socketcan STARK_SOCKETCAN_IFACE=can0 make run revo2_canfd
```

### EtherCAT Build Notes

EtherCAT examples require special capabilities:

```bash
# Compile (automatically sets capabilities)
cd revo2_ec
make

# Verify capabilities
getcap revo2_pdo.exe
# Output: revo2_pdo.exe cap_net_admin,cap_net_raw,cap_sys_nice=eip

# Run without sudo
./revo2_pdo.exe
```

## 📖 Additional Resources

- [Official Documentation](https://www.brainco-hz.com/docs/revolimb-hand/index.html)
- [ROS/ROS2 Integration](https://github.com/BrainCoTech/brainco_hand_ros2)

## 🤝 Support

For technical support:
- Check example code in subdirectories
- Review API documentation above
- Contact BrainCo technical support

## 📝 Notes

- Always call `setup_signal_handlers()` at the start of your program for better debugging
- Remember to free allocated structures (`free_device_config`, `free_device_info`, `free_motor_status_data`, etc.)
- Position values range from 0 (open) to 1000 (closed) for all devices
- For Revo2, use `init_cfg()` before connecting to the device
- Touch-enabled devices require `stark_enable_touch_sensor()` before reading sensor data
- EtherCAT examples should run without `sudo` to avoid D-state issues
- Always check slave positions with `sudo ethercat slaves` before using EtherCAT commands

---

**Version:** Compatible with Stark SDK v1.0.1
