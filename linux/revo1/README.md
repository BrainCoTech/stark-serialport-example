# Revo1 Example Programs

## Smart Build & Run Commands

The new `make run` command automatically detects the program type and uses the correct compilation mode:

```shell
# Smart compile + run (automatically detects MODE)
make run revo1_ctrl           # Modbus programs (default mode)
make run revo1_touch          # Modbus touch-hand example
make run revo1_can            # Auto-detects CAN mode

# Show usage help
make run                      # Shows available targets and examples
```

## Traditional Build & Run Commands

```shell
make clean # Clean old build artifacts

# Build with specific modes
make                          # Build with default mode (Modbus)
make MODE=can                 # Build with CAN (all backends compiled)
make STARK_NO_CAN=1           # Disable CAN support entirely

# Run only (must be compiled first)
make run_revo1_ctrl           # Run revo1_modbus_ctrl example
make run_revo1_ctrl_multi     # Run revo1_modbus_ctrl_multi example
make run_revo1_touch          # Run revo1_modbus_touch example
make run_revo1_can            # Run revo1_can_ctrl example
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
STARK_SOCKETCAN_IFACE=can0 make run revo1_can

# ZLG backend
STARK_CAN_BACKEND=zlg make run revo1_can
```
