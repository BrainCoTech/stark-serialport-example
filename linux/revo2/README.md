# Revo2 Example Programs

## Smart Build & Run Commands

The new `make run` command automatically detects the program type and uses the correct compilation mode:

```shell
# Smart compile + run (automatically detects MODE)
make run revo2_ctrl           # Modbus programs (default mode)
make run revo2_touch          # Modbus touch-hand example
make run revo2_canfd          # Auto-detects CAN mode
make run revo2_canfd_touch    # Auto-detects CAN mode
make run revo2_can_ctrl       # Auto-detects CAN mode

# Show usage help
make run                      # Shows available targets and examples
```

## Traditional Build & Run Commands

```shell
make clean # Clean old build artifacts

# Build with specific modes
make                          # Build with default mode (Modbus)
make MODE=can                 # Build with CAN/CANFD (all backends compiled)
make STARK_NO_CAN=1           # Disable CAN support entirely

# Run only (must be compiled first)
make run_revo2_ctrl           # Run Modbus control example
make run_revo2_ctrl_multi     # Run Modbus multi-hand control example
make run_revo2_touch          # Run Modbus touch-hand example
make run_revo2_can_ctrl       # Run CAN control example
make run_revo2_canfd          # Run CANFD control example
make run_revo2_canfd_touch    # Run CANFD touch example
```

## EtherCAT Examples

EtherCAT examples are located in a separate directory:

```shell
# See c/platform/linux/revo2_ec/ for standalone EtherCAT examples
cd ../../c/platform/linux/revo2_ec/
make
./revo2_pdo.exe
```

## CAN Backend Selection (Runtime)

All CAN backends are compiled by default. Select at runtime via environment variables:

```shell
# SocketCAN (Linux default, no 3rd party deps)
export STARK_CAN_BACKEND=socketcan
export STARK_SOCKETCAN_IFACE=can0

# ZLG USB-CANFD (dynamic loading)
export STARK_CAN_BACKEND=zlg
export STARK_ZLG_LIB_PATH=/path/to/libusbcanfd.so  # Optional custom path
```

Run examples:

```shell
# SocketCAN (default on Linux)
STARK_SOCKETCAN_IFACE=can0 make run revo2_can_ctrl
STARK_SOCKETCAN_IFACE=can0 make run revo2_canfd
STARK_SOCKETCAN_IFACE=can0 make run revo2_canfd_touch

# ZLG backend
STARK_CAN_BACKEND=zlg make run revo2_canfd
```

