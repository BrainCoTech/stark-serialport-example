# Revo2 Example Programs

## Smart Build & Run Commands

The new `make run` command automatically detects the program type and uses the correct compilation mode:

```shell
# Smart compile + run (automatically detects MODE)
make run revo2_ctrl           # Modbus programs (default mode)
make run revo2_touch          # Modbus touch-hand example
make run revo2_canfd          # Auto-detects CAN mode, includes -lusbcanfd
make run revo2_can_ctrl       # Auto-detects CAN mode
make run revo2_ethercat       # Auto-detects EtherCAT mode

# Show usage help
make run                      # Shows available targets and examples
```

## Traditional Build & Run Commands

```shell
make clean # Clean old build artifacts

# Build with specific modes
make                          # Build with default mode (Modbus)
make MODE=can                 # Build with CAN/CANFD interface mode  
make MODE=ethercat            # Build with EtherCAT interface mode

# Run only (must be compiled first)
make run_revo2_ctrl           # Run Modbus control example
make run_revo2_ctrl_multi     # Run Modbus multi-hand control example
make run_revo2_dfu            # Run Modbus firmware upgrade example
make run_revo2_touch          # Run Modbus touch-hand example
make run_revo2_can_ctrl       # Run CAN control example
make run_revo2_can_dfu        # Run CAN firmware upgrade example
make run_revo2_canfd          # Run CANFD control example
make run_revo2_canfd_dfu      # Run CANFD firmware upgrade example
make run_revo2_ethercat       # Run EtherCAT example
```
