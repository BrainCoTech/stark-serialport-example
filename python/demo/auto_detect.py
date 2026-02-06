#!/usr/bin/env python3
"""
Stark Auto-Detect - Device Detection

Scans for all connected Stark devices across all protocols.

Run:
    python auto_detect.py
"""

import asyncio
import sys
import os

# Setup path and imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common_imports import sdk, check_sdk, get_hw_type_name, get_protocol_display_name, logger

check_sdk()


async def main():
    print("=== Stark Auto-Detect ===\n")
    logger.info("Scanning for devices...")
    
    # Scan all protocols
    devices = await sdk.auto_detect(scan_all=True)
    
    if not devices:
        logger.info("No devices found")
        print("\nTroubleshooting:")
        print("  - Check USB connection")
        print("  - Verify device is powered on")
        print("  - Check driver installation")
        return
    
    logger.info(f"Found {len(devices)} device(s)")
    print("-" * 70)
    
    for i, device in enumerate(devices):
        hw_name = get_hw_type_name(device.hardware_type) if device.hardware_type else "Unknown"
        proto_name = get_protocol_display_name(device.protocol_type)
        
        print(f"\n[Device {i + 1}] {hw_name}")
        print(f"  Protocol:     {proto_name}")
        print(f"  Port:         {device.port_name}")
        print(f"  Slave ID:     0x{device.slave_id:02X} ({device.slave_id})")
        print(f"  Baudrate:     {device.baudrate}")
        
        if device.serial_number:
            print(f"  Serial:       {device.serial_number}")
        if device.firmware_version:
            print(f"  Firmware:     {device.firmware_version}")
        if device.sku_type:
            print(f"  SKU:          {device.sku_type}")
    
    print("\n" + "-" * 70)
    
    # Summary by protocol
    protocols = {}
    for d in devices:
        proto = get_protocol_display_name(d.protocol_type)
        protocols[proto] = protocols.get(proto, 0) + 1
    
    summary_parts = [f"{proto}: {count}" for proto, count in protocols.items()]
    print(f"\nTotal: {len(devices)} device(s) ({', '.join(summary_parts)})")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nUser interrupted")
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
