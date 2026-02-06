# Linux-specific C++ Examples

This directory contains Linux-only examples that require libraries not available on macOS/Windows.

## Directory Structure

```
platform/linux/
├── revo2/           # EtherCAT example using Stark SDK (requires libethercat)
└── revo2_ec/        # Standalone EtherCAT library (no SDK dependency)
```

## Linux-only Features

### ZLG USB-CAN/CANFD SocketCAN Driver

ZLG USB-CAN/CANFD adapters can be used via the Linux SocketCAN subsystem.

Official documentation: https://manual.zlg.cn/web/#/42/13456

#### 1. Download Driver

Download the SocketCAN driver from ZLG official website: https://manual.zlg.cn/web/#/42/13456

#### 2. Extract Driver

```bash
unzip usbcanfd200_400u_2.10.zip
cd usbcanfd200_400u_2.10
```

#### 3. Load Driver

```bash
# Load dependency modules
sudo modprobe can-dev

# If you encounter "Unknown symbol in module" error, load sja1000 first
sudo modprobe sja1000

# Load ZLG SocketCAN driver
sudo insmod usbcanfd.ko

# To unload driver
sudo rmmod usbcanfd
```

> **Note**: SocketCAN driver and ZLG official DLL are two independent methods:
> - **SocketCAN driver**: Uses Linux kernel CAN subsystem, with standard tools like `ip link` and `candump`
> - **ZLG official DLL**: Accesses USB device directly via `libusbcanfd.so`
>
> They cannot be used simultaneously. If SocketCAN driver is loaded, ZLG DLL cannot open the device; vice versa.
> Unload SocketCAN driver before using DLL: `sudo rmmod usbcanfd`

#### 4. Verify Device

```bash
# Check if USB device is recognized
lsusb | grep -E "04cc:1240|3068:0009"

# Check if CAN interface is created
ls /sys/class/net | grep can
```

#### 5. Configure CAN Interface

```bash
# CAN 2.0 mode
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# CANFD mode (recommended)
sudo ip link set can0 type can fd on bitrate 1000000 dbitrate 5000000 sample-point 0.75 dsample-point 0.75
sudo ip link set can0 up
```

#### 6. Test with candump

```bash
# Monitor CAN traffic
candump can0 -t A
```

### EtherCAT

```bash
cd revo2/
make
make run
```

Requires:
- libethercat.so installed
- EtherCAT master configured
- Appropriate permissions

## EtherCAT Library (revo2_ec/)

Standalone EtherCAT library and examples. **Does not depend on Stark SDK** - uses IgH EtherCAT Master directly.

| Example | Description |
|---------|-------------|
| revo2_sdo.cpp | SDO read/write operations |
| revo2_pdo.cpp | PDO cyclic control |
| revo2_multi_pdo.cpp | Multi-slave PDO control |
| revo2_touch_sdo.cpp | Touch sensor via SDO |
| revo2_touch_pdo.cpp | Touch sensor via PDO |
| revo2_touch_pressure_pdo.cpp | Pressure sensor data |

See [revo2_ec/README.md](revo2_ec/README.md) for details.
