#!/bin/bash
# Script to kill all processes using EtherCAT device

echo "=== Killing EtherCAT Processes ==="

# Find all processes using EtherCAT device
PIDS=$(sudo fuser /dev/EtherCAT0 2>/dev/null)

if [ -z "$PIDS" ]; then
    echo "[OK] No processes using EtherCAT device"
    exit 0
fi

echo "Found processes using EtherCAT: $PIDS"

# Try graceful kill first
for PID in $PIDS; do
    echo "Trying to kill process $PID gracefully..."
    sudo kill $PID 2>/dev/null
done

sleep 2

# Check if still running
PIDS=$(sudo fuser /dev/EtherCAT0 2>/dev/null)
if [ -z "$PIDS" ]; then
    echo "[OK] All processes terminated successfully"
    exit 0
fi

# Force kill if still running
echo "Some processes still running, forcing kill..."
for PID in $PIDS; do
    echo "Force killing process $PID..."
    sudo kill -9 $PID 2>/dev/null
done

sleep 1

# Final check
PIDS=$(sudo fuser /dev/EtherCAT0 2>/dev/null)
if [ -z "$PIDS" ]; then
    echo "[OK] All processes terminated"
else
    echo "[WARN] Some processes in uninterruptible sleep state (D state)"
    echo "   PIDs: $PIDS"
    echo "   These processes are blocked in kernel and cannot be killed."
    echo "   Solution: Restart EtherCAT service or reboot system"
    echo ""
    echo "   Quick fix:"
    echo "   sudo systemctl restart ethercat"
    exit 1
fi
