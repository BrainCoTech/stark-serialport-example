# BrainCo RevoHand SDK Examples Changelog

> This changelog is for external customers and integration developers. For SDK internal implementation details, please contact BrainCo technical support.

## v1.1.5 (2026/02/09)

### SDK Changes
- Fixed `canfd_ctx.rs` boundary check
- SocketCAN scan now iterates all interfaces instead of single interface
- Added SocketCAN Python bindings (`init_socketcan_canfd`, `close_socketcan`, `socketcan_scan_devices`)

### Example Improvements

#### Runtime CAN Backend Selection
- Changed CAN backend from compile-time to runtime selection
- Use `STARK_CAN_BACKEND` env var or CLI options to select backend
  - `-s` / `-S`: SocketCAN
  - `-z` / `-Z`: ZLG
  - `-c` / `-f`: ZQWL
- Linux builds all backends by default, use `STARK_NO_CAN` to disable CAN support
- ZLG backend now uses dlopen for dynamic loading (no compile-time dependency)

### Other Changes
- Added `zlgcan_linux_utils.py` / `zlgcan_win_utils.py`, `print_finger_touch_data()` utility
- Removed deprecated `zqwl.py`, `zqwl_can.py`, `socketcan_canfd_dfu.py`, `CAN_BACKEND` variable
- Fixed `zlg_canfd_touch_pressure.py` callback signature, added slave_id notes

---

## v1.1.4 (2026/02/09)

### New Features

#### Device Context Query
- Added `stark_get_protocol_type`, `stark_get_port_name`, `stark_get_baudrate`, `stark_get_can_arb_baudrate`, `stark_get_can_data_baudrate` query APIs
- Added `init_device_handler_can()` / `init_device_handler_can_with_hw_type()` CAN device initialization
- Added `StarkProtocolType::Auto = 0` enum value for auto-detecting all protocols

### Example Improvements
- C++: Renamed `CollectorContext` to `DeviceContext`, removed `protocol`, `port_name`, `baudrate` fields, use C API query instead
- C++: CAN initialization functions now use `init_device_handler_can()` to store baudrate
- C++: `stark_auto_detect` return type changed to `StarkProtocolType` enum for type safety
- C++: Fixed `hand_demo.cpp` compile error: use `STARK_PROTOCOL_TYPE_AUTO` instead of literal `0`
- Python: Added comprehensive type annotations

---

## v1.1.2 (2026/02/06)

### New Features

#### Protocol Support Extension
- **ZQWL CAN Built-in** - SDK built-in driver, no extra DLL needed, supports Linux/macOS/Windows
- **SocketCAN Built-in** (Linux) - No external code required
- **Protobuf Protocol** - Revo1 serial protocol, baudrate 115200, Slave ID 10-254

#### Unified Auto Detection
- `auto_detect()` supports Modbus, ZQWL CAN/CANFD multi-protocol auto detection
- `init_from_detected()` initializes device directly from detection result

#### Revo1 Touch API
- Touch sensor support for Modbus and CAN 2.0 protocols
- Support for Revo1Touch and Revo1AdvancedTouch devices

#### New Hardware Types
- `Revo1Advanced` - Gen1 Advanced (serial prefix BCMEL/BCMER)
- `Revo1AdvancedTouch` - Gen1 Advanced Touch (serial prefix BCMTL2/BCMTR2)

#### Examples and Tools
- `c/demo/` - Cross-platform C++ examples (hand_demo, hand_monitor, hand_dfu, auto_detect)
- `python/demo/` - Unified Python examples
- `python/gui/` - GUI debugging tool (for debugging only)

### Migration Guide

#### API Renaming

| Old API | New API |
|---------|---------|
| `is_revo1()` | `uses_revo1_motor_api()` |
| `is_revo1_touch()` | `uses_revo1_touch_api()` |
| `PyDeviceContext` | `DeviceContext` |

> Note: To check if using Revo2 API, use `!uses_revo1_motor_api()` / `!uses_revo1_touch_api()`

#### Initialization API Changes

**Old API:**
```python
sdk.init_config(protocol, log_level)
ctx = PyDeviceContext.init_canfd(master_id)
```

**New API:**
```python
sdk.init_logging(log_level)
ctx = sdk.init_device_handler(StarkProtocolType.CanFd, master_id)
```

#### C Struct Naming Changes

All C exported structs now have `C` prefix: `MotorStatusData` → `CMotorStatusData`, `DeviceInfo` → `CDeviceInfo`, etc.

#### Deprecation Notice
- `linux/` and `windows/` folders are deprecated, please migrate to `c/` folder

---

## v1.0.6 (2026/01/26)

### 🚀 New Features

#### New Hardware Support
- **Revo1Advanced** - Gen1 Advanced (serial prefix BCMEL/BCMER), uses Revo2 API
- **Revo1AdvancedTouch** - Gen1 Advanced Touch (serial prefix BCMTL2/BCMTR2)

#### Serial Number Auto Recognition

| Serial Prefix | Hardware Type |
|---------------|---------------|
| `BCMRL/BCMRR` | Revo1Basic |
| `BCMEL/BCMER` | Revo1Advanced |
| `BCMTL1/BCMTR1` | Revo1Touch |
| `BCMTL2/BCMTR2` | Revo1AdvancedTouch |
| `BCXTL/BCXTR` | Revo2Touch |
| `BCX*` | Revo2Basic |

#### Architecture Optimization
- Motor/touch status uses low-level multi-threaded collection, upper layer passive reading
- Physical mode control logic fix
- Revo2 RS-485 DFU baudrate auto detection
- SocketCAN backend support

#### Auto Detection Enhancement
- Multi-port auto traversal
- Revo1/Revo2/Protobuf protocol auto recognition
- Quick mode for fast detection

### 📚 New Examples
- `revo2_touch_collector.py` - Touch data collection
- `revo2_timing_test_gui.py` - Timing test GUI

### ⚠️ Note
- Revo1Advanced (BCMEL/BCMER) should use examples in `revo2` directory

---

## v1.0.4 (2026/01/23)

- Data Collector support
- Trajectory control support

---

## v1.0.1 (2025/12/23)

- EtherCAT multi-slave communication support

---

## v1.0.0 (2025/12/08)

### 🎉 Official Release
- Support for Revo1 and Revo2 devices
- Support for RS-485, CAN, CANFD, EtherCAT protocols
- Python and C++ example code provided

---

## v0.9.9 (2025/11/19)

- Revo1 Advanced device support
- Unified control parameter range: position 0~1000, speed/current/PWM -1000~+1000

> ⚠️ Revo1 Advanced devices require SDK v0.9.9+

---

## v0.9.8 (2025/11/04)

### 🚀 New Features

#### CAN/CANFD Support
- Revo2 CAN2.0/CANFD protocol stack
- ZLG CAN/CANFD driver wrapper
- CANFD chunked read/write (supports more than 29 registers)

#### EtherCAT Support
- Touch sensor data collection (PDO/SDO)
- Touch pressure sensor support

#### General Features
- ProtectedCurrent read/write API
- `run_action_sequence` action sequence execution
- Serial number device type detection

#### Performance Optimization
- C/C++ `set` commands execute asynchronously
- High-frequency APIs disable retry to avoid command queue buildup

### 🐛 Bug Fixes
- TurboConfig byte order issue
- Modbus C API async call
- DFU upgrade process optimization

---

For questions, visit [Official Documentation](https://www.brainco-hz.com/docs/revolimb-hand/index.html) or contact BrainCo technical support.
