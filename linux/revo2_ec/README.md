# Revo 2 - EtherCAT Examples

```shell
ip link show                   # Check network interfaces
ethercat version               # Check EtherCAT version
sudo systemctl enable ethercat # Enable EtherCAT master
sudo systemctl restart ethercat # Restart EtherCAT master
sudo systemctl status ethercat # Check EtherCAT master status

# /usr/local/etc/ethercat.conf
journalctl -u ethercat.service -b # View EtherCAT service logs
# Check kernel logs to verify driver / NIC binding
dmesg | grep EtherCAT
dmesg | grep ec_
lsmod | grep ec_

sudo ethercat slaves  # List EtherCAT slaves
sudo ethercat sdos    # Inspect SDO information of slaves
sudo ethercat pdos    # Inspect PDO information of slaves

## Running Programs

### No sudo needed!

Programs are automatically granted necessary capabilities during compilation:
```shell
# Compile (automatically sets capabilities)
make

# Run directly without sudo
./revo2_pdo.exe
./revo2_sdo.exe

# Or use make commands (smart compile + run)
make run revo2_pdo      # Auto-compile if needed, then run
make run revo2_sdo      # Auto-compile if needed, then run
make run revo1          # Alias for revo2_pdo
make run revo2          # Alias for revo2_pdo

# Run only (no compilation)
make run_revo2_pdo      # Run directly (must be compiled first)
make run_revo2_sdo      # Run directly (must be compiled first)

# Verify capabilities
getcap revo2_pdo.exe
# Output: revo2_pdo.exe cap_net_admin,cap_net_raw,cap_sys_nice=eip

# Read firmware versions
ethercat upload -t string -p 0 0x8000 0x11 # FW version
ethercat upload -t string -p 0 0x8000 0x12 # SN
ethercat upload -t string -p 0 0x8000 0x13 # Wrist FW version
ethercat upload -t string -p 0 0x8000 0x14 # Wrist SN

ethercat upload -t uint8  -p 0 0x8010 0x06 # Touch Sensor Vendor ID
ethercat upload -t string -p 0 0x8010 0x0D # Thumb Touch Sensor FW
ethercat upload -t string -p 0 0x8010 0x0E # Index Touch Sensor FW
ethercat upload -t string -p 0 0x8010 0x0F # Middle Touch Sensor FW
ethercat upload -t string -p 0 0x8010 0x10 # Ring Touch Sensor FW
ethercat upload -t string -p 0 0x8010 0x11 # Pinky Touch Sensor FW 

# Read joint information in OP mode
# echo "=== Read all joint data ==="
# echo "Position data (6x UINT16):"
ethercat upload -t raw -p 0 0x6000 0x01
ethercat upload -t raw -p 0 0x6000 0x01 | xxd -r -p | od -An -t u2 --endian=little -w2 | awk '{printf "Joint Position %d: %d\n", NR, $1}'

# Read touch information in OP mode
ethercat upload -t raw -p 0 0x6010 0x01
ethercat upload -t raw -p 0 0x6010 0x01 | xxd -r -p | od -An -t u2 --endian=little -w2 | awk '{printf "Normal Force %d: %d\n", NR, $1}'

make clean                 # Clean old build artifacts
make                       # Build
make run_revo2_sdo         # Run revo2_sdo example
make run_revo2_pdo         # Run revo2_pdo example
make run_revo2_touch_sdo   # Run revo2_sdo example (touch hand)
make run_revo2_touch_pdo   # Run revo2_pdo example (touch hand)
```

### Why not sudo?

Using `sudo` increases the risk of processes entering D state (uninterruptible sleep).
See `WHY_NO_SUDO.md` for details.

## Troubleshooting

### Kill stuck EtherCAT processes

```shell
# Find processes using EtherCAT device
sudo fuser -v /dev/EtherCAT0

# Kill specific process
sudo kill -9 <PID>

# Or use the helper script
./kill_ethercat_processes.sh

# If process is in D state (uninterruptible sleep), restart EtherCAT service
sudo systemctl restart ethercat
```

### Common Issues

**Error: Module ec_generic is in use**
- Cause: A program is still using the EtherCAT device
- Solution: 
  1. Find and kill the process: `sudo fuser -k /dev/EtherCAT0`
  2. If that fails: `sudo systemctl restart ethercat`
  3. Last resort: Reboot system

**Process stuck in D state**
- Cause: Process blocked in kernel waiting for I/O
- Solution: Cannot kill with `kill -9`, must restart EtherCAT service
  `sudo systemctl restart ethercat`
