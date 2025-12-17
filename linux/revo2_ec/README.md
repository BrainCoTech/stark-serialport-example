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

sudo ethercat slaves  # List EtherCAT slaves (check position numbers!)
sudo ethercat sdos    # Inspect SDO information of slaves
sudo ethercat pdos    # Inspect PDO information of slaves

# IMPORTANT: Check slave position first!
# Example output of 'ethercat slaves':
#   0: 0  PREOP  +  BrainCo-Revo2Slave
#   1: 0  PREOP  +  STARK REVO2 Hand
# The first number (0, 1) is the slave position to use with -p option

## Running Programs

### No sudo needed!

Programs are automatically granted necessary capabilities during compilation:
```shell
# Compile (automatically sets capabilities)
make

# Run directly without sudo
./revo2_pdo.exe [slave_pos]  # Default: slave_pos=1 for PDO, slave_pos=1 for SDO
./revo2_sdo.exe [slave_pos]  # Default: slave_pos=1

# Examples:
./revo2_pdo.exe 0    # Use slave position 0
./revo2_pdo.exe 1    # Use slave position 1 (default)
./revo2_sdo.exe 0    # Use slave position 0
./revo2_sdo.exe 1    # Use slave position 1 (default)

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

# Read firmware versions (adjust -p <pos> based on your slave position)
# STEP 1: ALWAYS check your actual slave position first!
#   sudo ethercat slaves
#   Look at the first number in the output (0, 1, 2, etc.)
# STEP 2: Use that position number with -p option

# Example commands (replace <POS> with your actual slave position):
# If your slave is at position 0:
sudo ethercat upload -t string -p 0 0x8000 0x11 # FW version
sudo ethercat upload -t string -p 0 0x8000 0x12 # SN
sudo ethercat upload -t string -p 0 0x8000 0x13 # Wrist FW version
sudo ethercat upload -t string -p 0 0x8000 0x14 # Wrist SN
sudo ethercat upload -t uint8  -p 0 0x8000 0x1  # Hand Type
sudo ethercat upload -t uint16 -p 0 0x8000 0xa  # thumb_flex_pro_cur
sudo ethercat upload -t uint16 -p 0 0x8000 0xb  # thumb_aux_pro_cur

# If your slave is at position 1, use -p 1 instead:
# sudo ethercat upload -t uint16 -p 1 0x8000 0xb

# Touch sensor information (replace <POS> with your actual slave position):
sudo ethercat upload -t uint8  -p 0 0x8010 0x06 # Touch Sensor Vendor ID
sudo ethercat upload -t string -p 0 0x8010 0x0D # Thumb Touch Sensor FW
sudo ethercat upload -t string -p 0 0x8010 0x0E # Index Touch Sensor FW
sudo ethercat upload -t string -p 0 0x8010 0x0F # Middle Touch Sensor FW
sudo ethercat upload -t string -p 0 0x8010 0x10 # Ring Touch Sensor FW
sudo ethercat upload -t string -p 0 0x8010 0x11 # Pinky Touch Sensor FW 

# Read joint information in OP mode (replace <POS> with your actual slave position)
# echo "=== Read all joint data ==="
# echo "Position data (6x UINT16):"
sudo ethercat upload -t raw -p 0 0x6000 0x01
sudo ethercat upload -t raw -p 0 0x6000 0x01 | xxd -r -p | od -An -t u2 --endian=little -w2 | awk '{printf "Joint Position %d: %d\n", NR, $1}'

# Read touch information in OP mode
sudo ethercat upload -t raw -p 0 0x6010 0x01
sudo ethercat upload -t raw -p 0 0x6010 0x01 | xxd -r -p | od -An -t u2 --endian=little -w2 | awk '{printf "Normal Force %d: %d\n", NR, $1}'

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

**Error: "The slave selection matches 0 slaves"**
- Cause: The specified slave position doesn't exist
- Solution:
  1. **First, check actual slave positions:**
     ```shell
     sudo ethercat slaves
     # Look at the first column (position number)
     # Example output:
     #   0: 0  PREOP  +  BrainCo-Revo2Slave
     #   1: 0  PREOP  +  STARK REVO2 Hand
     ```
  2. **If no slaves found:**
     - Check physical connection
     - Verify EtherCAT master is running: `sudo systemctl status ethercat`
     - Check if device is bound: `dmesg | grep EtherCAT`
  3. **Use the correct position number:**
     ```shell
     # If 'ethercat slaves' shows position 1, use -p 1:
     sudo ethercat upload -t uint16 -p 1 0x8000 0xb
     # Not -p 0!
     ```

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

**Master AL state stuck at 0x0C (not reaching 0x08 OPERATIONAL)**
- Cause: Some slaves are in SAFEOP state instead of OPERATIONAL
- EtherCAT AL State Codes:
  - `0x01`: INIT - Initialization state
  - `0x02`: PREOP - Pre-operational state (SDO operations allowed)
  - `0x04`: SAFEOP - Safe operational state (PDO read-only)
  - `0x08`: OP - Operational state (full PDO read/write)
  - `0x0C`: Combination of SAFEOP (0x04) and OP (0x08) - indicates some slaves are in SAFEOP
- Solution:
  1. Check individual slave states: `sudo ethercat slaves`
  2. Verify PDO configuration matches slave capabilities
  3. Check for PDO mapping errors in slave configuration
  4. Ensure all slaves support the configured PDOs
  5. Check slave error registers: `sudo ethercat sdos -p <pos>`
