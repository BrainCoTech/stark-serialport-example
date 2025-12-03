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

sudo ethercat slaves  # List EtherCAT slaves
sudo ethercat sdos    # Inspect SDO information of slaves
sudo ethercat pdos    # Inspect PDO information of slaves

# Read firmware versions
ethercat upload -t string -p 0 0x8000 0x11 # FW version
ethercat upload -t string -p 0 0x8000 0x12 # SN
ethercat upload -t string -p 0 0x8000 0x13 # Wrist FW version
ethercat upload -t string -p 0 0x8000 0x14 # Wrist SN

ethercat upload -t uint8 -p 0 0x8010 0x06 # Touch Sensor Vendor ID
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

# Read tactile information in OP mode
ethercat upload -t raw -p 0 0x6010 0x01
ethercat upload -t raw -p 0 0x6010 0x01 | xxd -r -p | od -An -t u2 --endian=little -w2 | awk '{printf "Normal Force %d: %d\n", NR, $1}'

make clean                 # Clean old build artifacts
make                       # Build
make run_revo2_sdo         # Run revo2_sdo example
make run_revo2_pdo         # Run revo2_pdo example
make run_revo2_touch_sdo   # Run revo2_sdo example (tactile version)
make run_revo2_touch_pdo   # Run revo2_pdo example (tactile version)
```
