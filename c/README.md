# Stark SDK C/C++ Examples

Cross-platform examples for Revo1 and Revo2 devices.

## Quick Start

```bash
cd c
make
cd demo
./hand_demo.exe       # Auto-detect device and protocol
```

## Directory Structure

```
c/
├── demo/                # ⭐ Main entry - auto-detect device and protocol
│   ├── hand_demo.cpp        # Comprehensive demo (8 modes)
│   ├── hand_monitor.cpp     # Real-time data monitor
│   ├── hand_dfu.cpp         # Firmware upgrade
│   └── auto_detect.cpp      # Device detection
├── common/              # Shared code library
│   ├── stark_common.cpp/h   # Common device operations
│   ├── dfu_common.cpp/h     # DFU operations
│   ├── can_common.cpp/h     # CAN support (ZLG/SocketCAN)
│   ├── platform_compat.h    # Cross-platform compatibility
│   └── zqwl_common.h        # ZQWL definitions
└── platform/linux/      # Linux-only (EtherCAT)
```

## Supported Protocols and Adapters

### SDK Built-in Support (Auto-detect)

| Protocol | Adapter | Device | Auto-detect |
|----------|---------|--------|-------------|
| Modbus (RS485) | USB-RS485 | Revo1, Revo2 | ✅ |
| Protobuf | USB-Serial | Revo1 | ✅ |
| CAN 2.0 | ZQWL USB-CAN | Revo1, Revo2 | ✅ |
| CANFD | ZQWL USB-CANFD | Revo2 | ✅ |
| SocketCAN | Linux CAN | Revo1, Revo2 | ✅ |

### Requires Manual Configuration

| Protocol | Adapter | Platform | Notes |
|----------|---------|----------|-------|
| CAN 2.0 / CANFD | SocketCAN | Linux | SDK built-in (`-b/-B`) or can_common.cpp (`-s/-S`) |
| CAN 2.0 / CANFD | ZLG | Linux/Win | Requires external DLL + callback |
| EtherCAT | NIC | Linux | Requires IgH driver |

## Platform Support

| Feature | Linux | macOS | Windows |
|---------|-------|-------|---------|
| Modbus (Serial) | ✅ | ✅ | ✅ |
| ZQWL USB-CAN/CANFD | ✅ | ✅ | ✅ |
| ZLG USB-CAN/CANFD | ✅ | ❌ | ✅ |
| SocketCAN | ✅ | ❌ | ❌ |
| EtherCAT | ✅ | ❌ | ❌ |

## Build

```bash
cd c
make                        # Build (all CAN backends on Linux)
make clean                  # Clean
make TARGET=win             # Cross-compile for Windows
make STARK_NO_CAN=1         # Disable CAN support
```

### Build Options

| Option | Description |
|--------|-------------|
| `TARGET=win` | Cross-compile for Windows |
| `STARK_NO_CAN=1` | Disable CAN support entirely |

### CAN Backend (Runtime Selection)

All CAN backends are compiled by default on Linux. Select at runtime:

| Backend | CLI Options | Environment Variable |
|---------|-------------|---------------------|
| SocketCAN (default) | `-s`/`-S` | `STARK_CAN_BACKEND=socketcan` |
| ZLG | `-z`/`-Z` | `STARK_CAN_BACKEND=zlg` |
| ZQWL (SDK built-in) | `-c`/`-f` | - |

Note: ZLG requires `libusbcanfd.so` at runtime (dynamic loading).

## Running Examples

### hand_demo - Comprehensive Demo

```bash
./hand_demo.exe          # Auto-detect, run all demos
./hand_demo.exe 1        # Run specific demo (1-8)
./hand_demo.exe -h       # Show help
```

**Initialization options:**
```bash
./hand_demo.exe                                          # Auto-detect (recommended)
./hand_demo.exe -m /dev/ttyUSB0 460800 127               # Modbus
./hand_demo.exe -p /dev/ttyUSB0                          # Protobuf (115200, default id=10)
./hand_demo.exe -p /dev/ttyUSB0 10                       # Protobuf with custom slave_id
./hand_demo.exe -c /dev/cu.usbmodem14201 1000000 1       # CAN 2.0 (ZQWL)
./hand_demo.exe -f /dev/cu.usbmodem14201 1000000 5000000 127  # CANFD (ZQWL)
./hand_demo.exe -s can0 1                                # SocketCAN CAN 2.0 (can_common.cpp)
./hand_demo.exe -S can0 127                              # SocketCAN CANFD (can_common.cpp)
./hand_demo.exe -b can0 1                                # SocketCAN CAN 2.0 (SDK built-in)
./hand_demo.exe -B can0 127                              # SocketCAN CANFD (SDK built-in)
```

**Demo modes:**

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

### hand_monitor - Real-time Monitor

```bash
./hand_monitor.exe           # Auto-detect
./hand_monitor.exe motor     # Motor data
./hand_monitor.exe touch     # Motor + capacitive touch
./hand_monitor.exe summary   # Motor + pressure summary
./hand_monitor.exe detailed  # Motor + pressure detailed
```

### hand_dfu - Firmware Upgrade

```bash
./hand_dfu.exe firmware.bin
```

### auto_detect - Device Detection

```bash
./auto_detect.exe
```

## CAN Adapter Configuration

### ZQWL (SDK Built-in)

Auto-detect works out of the box, no manual configuration needed.

### SocketCAN (Linux)

Two implementations available:

**Option 1: SDK Built-in (Recommended)**

No additional compile flags needed:

```bash
# Configure interface
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Run with SDK built-in SocketCAN
./hand_demo.exe -b can0 1       # CAN 2.0
./hand_demo.exe -B can0 127     # CANFD
```

**Option 2: Example's can_common.cpp**

All backends compiled by default on Linux:

```bash
# Configure interface
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Run (SocketCAN is default on Linux)
./hand_demo.exe -s can0 1       # CAN 2.0
./hand_demo.exe -S can0 127     # CANFD
```

> **ZLG USB-CAN/CANFD with SocketCAN**: If using ZLG adapter with SocketCAN driver, see [platform/linux/README.md](platform/linux/README.md) for driver installation.

### ZLG (dynamic loading at runtime)

Requires `libusbcanfd.so` (Linux) or `libusbcanfd.dll` (Windows) at runtime.

```bash
# Run with ZLG backend
STARK_CAN_BACKEND=zlg ./hand_demo.exe -z 1      # CAN 2.0
STARK_CAN_BACKEND=zlg ./hand_demo.exe -Z 127    # CANFD
```

## Device Types

| Device Type | Motor | Touch | Protocol |
|-------------|-------|-------|----------|
| Revo1 Basic | 6DOF | - | Modbus, Protobuf, CAN |
| Revo1 Touch | 6DOF | Capacitive | Modbus, Protobuf, CAN |
| Revo2 Basic | 6DOF | - | Modbus, CAN, CANFD |
| Revo2 Touch | 6DOF | Capacitive | Modbus, CAN, CANFD |
| Revo2 Touch Pressure | 6DOF | Pressure | Modbus, CAN, CANFD |

## Protocol Parameters

| Protocol | Baudrate | Slave ID |
|----------|----------|----------|
| Modbus | 460800 | Revo1: 1-10, Revo2: 126-127 |
| Protobuf | 115200 (fixed) | 10-254 (default: 10) |
| CAN 2.0 | 1000000 | Revo1: 1-10, Revo2: 126-127 |
| CANFD | 1M/5M | 126-127 |
