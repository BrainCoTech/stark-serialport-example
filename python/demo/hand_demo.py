#!/usr/bin/env python3
"""
Stark Hand Demo - Unified Python Example

Auto-detects device and protocol, provides comprehensive demo modes.
Supports: Modbus (RS485), Protobuf, CAN 2.0, CANFD, SocketCAN, ZLG

Run:
    python hand_demo.py              # Auto-detect, interactive menu
    python hand_demo.py 1            # Run specific demo (1-8)
    python hand_demo.py -p /dev/ttyUSB0  # Protobuf, default slave_id=10
    python hand_demo.py -s can0 1    # SocketCAN, slave_id=1
    python hand_demo.py -z 2         # ZLG CAN, slave_id=2
    python hand_demo.py -h           # Show help

Demo modes:
    1. Position control
    2. Speed/current control
    3. Advanced control (position+time/speed) - Revo2 only
    4. Action sequence
    5. Configuration info
    6. Touch sensor
    7. Interactive loop
    8. Multi-device control
"""

import asyncio
import sys
import os
import argparse

# Setup path and imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common_imports import (
    sdk, check_sdk, get_hw_type_name, logger,
    uses_revo1_motor_api, uses_revo2_motor_api, has_touch, 
    uses_revo1_touch_api, uses_revo2_touch_api
)
from common_init import (
    DeviceContext, parse_args_and_init, cleanup_context, 
    print_init_usage, auto_detect_and_init
)

check_sdk()


class HandDemo:
    """Unified hand demo class"""
    
    def __init__(self, device_ctx: DeviceContext):
        self.device_ctx = device_ctx
        self.ctx = device_ctx.ctx
        self.slave_id = device_ctx.slave_id
        self.hw_type = device_ctx.hw_type
        self.protocol_type = device_ctx.protocol_type
    
    async def demo_position_control(self):
        """Demo 1: Position control"""
        print("\n=== Demo 1: Position Control ===")
        
        finger_names = ["Thumb", "ThumbAux", "Index", "Middle", "Ring", "Pinky"]
        
        print("Opening all fingers...")
        await self.ctx.set_finger_positions(self.slave_id, [0, 0, 0, 0, 0, 0])
        await asyncio.sleep(1.0)
        
        for i, name in enumerate(finger_names):
            print(f"Closing {name}...")
            positions = [0] * 6
            positions[i] = 1000
            await self.ctx.set_finger_positions(self.slave_id, positions)
            await asyncio.sleep(0.5)
        
        print("Making fist...")
        await self.ctx.set_finger_positions(self.slave_id, [1000, 1000, 1000, 1000, 1000, 1000])
        await asyncio.sleep(1.0)
        
        print("Opening all fingers...")
        await self.ctx.set_finger_positions(self.slave_id, [0, 0, 0, 0, 0, 0])
        await asyncio.sleep(0.5)
        
        status = await self.ctx.get_motor_status(self.slave_id)
        print(f"Final positions: {list(status.positions)}")
    
    async def demo_speed_current_control(self):
        """Demo 2: Speed and current control"""
        print("\n=== Demo 2: Speed/Current Control ===")
        
        print("Opening all fingers...")
        await self.ctx.set_finger_positions(self.slave_id, [0, 0, 0, 0, 0, 0])
        await asyncio.sleep(1.0)
        
        print("Speed control - closing at speed 500...")
        await self.ctx.set_finger_speeds(self.slave_id, [500, 500, 500, 500, 500, 500])
        await asyncio.sleep(1.5)
        await self.ctx.set_finger_speeds(self.slave_id, [0, 0, 0, 0, 0, 0])
        
        print("Opening...")
        await self.ctx.set_finger_positions(self.slave_id, [0, 0, 0, 0, 0, 0])
        await asyncio.sleep(1.0)
        
        print("Current control - applying 500mA...")
        await self.ctx.set_finger_currents(self.slave_id, [500, 500, 500, 500, 500, 500])
        await asyncio.sleep(1.5)
        await self.ctx.set_finger_currents(self.slave_id, [0, 0, 0, 0, 0, 0])
        
        print("Opening all fingers...")
        await self.ctx.set_finger_positions(self.slave_id, [0, 0, 0, 0, 0, 0])
        await asyncio.sleep(0.5)
        
        status = await self.ctx.get_motor_status(self.slave_id)
        print(f"Final positions: {list(status.positions)}")

    
    async def demo_advanced_control(self):
        """Demo 3: Advanced control (Revo2 only)"""
        print("\n=== Demo 3: Advanced Control (Position + Time/Speed) ===")
        
        if not uses_revo2_motor_api(self.hw_type):
            print("[SKIP] This demo requires Revo2 or Revo1 Advanced device")
            return
        
        print("Position with duration (1000ms)...")
        positions = [500, 500, 1000, 1000, 1000, 1000]
        durations = [1000, 1000, 1000, 1000, 1000, 1000]
        await self.ctx.set_finger_positions_and_durations(self.slave_id, positions, durations)
        await asyncio.sleep(1.5)
        
        if self.protocol_type in [sdk.StarkProtocolType.Modbus, sdk.StarkProtocolType.CanFd]:
            print("Position with speed...")
            positions = [0, 0, 0, 0, 0, 0]
            speeds = [500, 500, 500, 500, 500, 500]
            await self.ctx.set_finger_positions_and_speeds(self.slave_id, positions, speeds)
            await asyncio.sleep(1.5)
        else:
            print("[SKIP] Position+speed requires Modbus or CANFD")
            positions = [0, 0, 0, 0, 0, 0]
            durations = [1000, 1000, 1000, 1000, 1000, 1000]
            await self.ctx.set_finger_positions_and_durations(self.slave_id, positions, durations)
            await asyncio.sleep(1.5)
        
        status = await self.ctx.get_motor_status(self.slave_id)
        print(f"Final positions: {list(status.positions)}")
    
    async def demo_action_sequence(self):
        """Demo 4: Action sequence"""
        print("\n=== Demo 4: Action Sequence ===")
        
        actions = [
            ("Fist", [1000, 1000, 1000, 1000, 1000, 0]),
            ("Open", [0, 0, 0, 0, 0, 0]),
            ("Point", [0, 0, 1000, 1000, 1000, 0]),
            ("OK", [500, 500, 0, 0, 0, 0]),
            ("Open", [0, 0, 0, 0, 0, 0]),
        ]
        
        for name, positions in actions:
            print(f"Action: {name}")
            await self.ctx.set_finger_positions(self.slave_id, positions)
            await asyncio.sleep(1.5)
        
        print("Action sequence completed")
    
    async def demo_config_info(self):
        """Demo 5: Configuration info"""
        print("\n=== Demo 5: Configuration Info ===")
        
        try:
            info = await self.ctx.get_device_info(self.slave_id)
            print(f"Hardware Type: {get_hw_type_name(info.hardware_type)}")
            print(f"Serial Number: {info.serial_number}")
            print(f"Firmware: {info.firmware_version}")
            print(f"Protocol: {self.protocol_type}")
            print(f"Slave ID: 0x{self.slave_id:02X} ({self.slave_id})")
        except Exception as e:
            print(f"Device info error: {e}")
        
        try:
            status = await self.ctx.get_motor_status(self.slave_id)
            print(f"\nMotor Status:")
            print(f"  Positions: {list(status.positions)}")
            print(f"  Speeds: {list(status.speeds)}")
            print(f"  Currents: {list(status.currents)}")
        except Exception as e:
            print(f"Motor status error: {e}")

    
    async def demo_touch_sensor(self):
        """Demo 6: Touch sensor"""
        print("\n=== Demo 6: Touch Sensor ===")
        
        if not has_touch(self.hw_type):
            print("[SKIP] This device does not have touch sensor")
            return
        
        if uses_revo1_touch_api(self.hw_type):
            print("Enabling touch sensors...")
            await self.ctx.touch_sensor_setup(self.slave_id, 0xFF)
            await asyncio.sleep(0.5)
        
        print("Reading touch sensor data (5 samples)...")
        finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
        
        for i in range(5):
            try:
                touch_data = await self.ctx.get_touch_sensor_status(self.slave_id)
                print(f"\n[{i+1}] Touch Data:")
                for idx, finger in enumerate(touch_data):
                    if idx < len(finger_names):
                        print(f"  {finger_names[idx]}: Normal={finger.normal_force1:4d}, "
                              f"Tangential={finger.tangential_force1:4d}, "
                              f"Status={finger.status}")
            except Exception as e:
                print(f"[{i+1}] Touch read error: {e}")
            await asyncio.sleep(0.2)
    
    async def demo_interactive_loop(self):
        """Demo 7: Interactive loop"""
        print("\n=== Demo 7: Interactive Loop ===")
        print("Commands: o=open, c=close, 1-6=toggle finger, q=quit")
        
        positions = [0, 0, 0, 0, 0, 0]
        finger_names = ["Thumb", "ThumbAux", "Index", "Middle", "Ring", "Pinky"]
        
        while True:
            try:
                cmd = input("\nCommand: ").strip().lower()
                if cmd == 'q':
                    break
                elif cmd == 'o':
                    positions = [0, 0, 0, 0, 0, 0]
                    await self.ctx.set_finger_positions(self.slave_id, positions)
                    print("Opened all fingers")
                elif cmd == 'c':
                    positions = [1000, 1000, 1000, 1000, 1000, 1000]
                    await self.ctx.set_finger_positions(self.slave_id, positions)
                    print("Closed all fingers")
                elif cmd in ['1', '2', '3', '4', '5', '6']:
                    idx = int(cmd) - 1
                    positions[idx] = 0 if positions[idx] > 500 else 1000
                    await self.ctx.set_finger_positions(self.slave_id, positions)
                    state = "closed" if positions[idx] > 500 else "open"
                    print(f"{finger_names[idx]}: {state}")
                else:
                    print("Unknown command")
            except EOFError:
                break
            except Exception as e:
                print(f"Error: {e}")
        
        print("Interactive loop ended")

    
    async def demo_multi_device(self):
        """Demo 8: Multi-device control"""
        print("\n=== Demo 8: Multi-Device Control ===")
        
        print("Scanning for all devices...")
        devices = await sdk.auto_detect(scan_all=True)
        
        if not devices:
            print("[WARN] No devices found via auto-detect")
            return
        
        print(f"\nFound {len(devices)} device(s):")
        for i, dev in enumerate(devices):
            hw_name = get_hw_type_name(dev.hardware_type) if dev.hardware_type else "Unknown"
            print(f"  [{i+1}] {hw_name} - {dev.protocol_type}, slave_id={dev.slave_id}")
        
        if len(devices) < 2:
            print("\n[WARN] Multi-device demo requires at least 2 devices")
            print("       Skipping demo.")
            return
        
        # Initialize handlers for all devices
        print(f"\nInitializing handlers for {len(devices)} devices...")
        handlers = []
        for dev in devices[:8]:  # Max 8 devices
            try:
                ctx = await sdk.init_from_detected(dev)
                handlers.append((ctx, dev.slave_id, dev.protocol_type))
                print(f"  Initialized device: slave_id={dev.slave_id}")
            except Exception as e:
                print(f"  [WARN] Failed to init slave_id={dev.slave_id}: {e}")
        
        if len(handlers) < 2:
            print("\n[WARN] Could not initialize at least 2 devices")
            return
        
        print(f"\nControlling {len(handlers)} devices simultaneously...")
        
        positions_fist = [500, 500, 1000, 1000, 1000, 1000]
        positions_open = [0, 0, 0, 0, 0, 0]
        
        # Synchronized fist gesture
        print("\nAll devices: Fist gesture...")
        for ctx, slave_id, _ in handlers:
            await ctx.set_finger_positions(slave_id, positions_fist)
        await asyncio.sleep(1.0)
        
        # Synchronized open hand
        print("All devices: Open hand...")
        for ctx, slave_id, _ in handlers:
            await ctx.set_finger_positions(slave_id, positions_open)
        await asyncio.sleep(1.0)
        
        # Alternating gestures (mirror mode)
        print("Alternating gestures (mirror mode)...")
        for cycle in range(3):
            # Device 1 fist, Device 2 open
            await handlers[0][0].set_finger_positions(handlers[0][1], positions_fist)
            await handlers[1][0].set_finger_positions(handlers[1][1], positions_open)
            await asyncio.sleep(0.8)
            
            # Device 1 open, Device 2 fist
            await handlers[0][0].set_finger_positions(handlers[0][1], positions_open)
            await handlers[1][0].set_finger_positions(handlers[1][1], positions_fist)
            await asyncio.sleep(0.8)
        
        # Final open
        print("All devices: Open hand (reset)...")
        for ctx, slave_id, _ in handlers:
            await ctx.set_finger_positions(slave_id, positions_open)
        await asyncio.sleep(0.5)
        
        print("Multi-device demo completed")

    
    async def run_demo(self, demo_num: int):
        """Run specific demo"""
        demos = {
            1: self.demo_position_control,
            2: self.demo_speed_current_control,
            3: self.demo_advanced_control,
            4: self.demo_action_sequence,
            5: self.demo_config_info,
            6: self.demo_touch_sensor,
            7: self.demo_interactive_loop,
            8: self.demo_multi_device,
        }
        
        if demo_num in demos:
            await demos[demo_num]()
        else:
            logger.error(f"Invalid demo number: {demo_num}")
    
    async def run_all_demos(self):
        """Run all demos (except interactive and multi-device)"""
        for i in range(1, 7):
            await self.run_demo(i)
            await asyncio.sleep(0.5)
    
    def show_menu(self) -> int:
        """Show demo menu and get selection"""
        print("\n=== Demo Menu ===")
        print("1. Position control")
        print("2. Speed/current control")
        print("3. Advanced control (Revo2)")
        print("4. Action sequence")
        print("5. Configuration info")
        print("6. Touch sensor")
        print("7. Interactive loop")
        print("8. Multi-device control")
        print("0. Run all (1-6)")
        print("q. Quit")
        
        while True:
            try:
                choice = input("\nSelect demo: ").strip().lower()
                if not choice:
                    continue
                if choice == 'q':
                    return -1
                num = int(choice)
                if 0 <= num <= 8:
                    return num
                print("Please enter 0-8 or q")
            except ValueError:
                print("Please enter 0-8 or q")


def print_usage():
    """Print usage information"""
    print("Usage: python hand_demo.py [options] [demo_number]")
    print("\nDemo numbers:")
    print("  1 - Position control")
    print("  2 - Speed/current control")
    print("  3 - Advanced control (Revo2)")
    print("  4 - Action sequence")
    print("  5 - Configuration info")
    print("  6 - Touch sensor")
    print("  7 - Interactive loop")
    print("  8 - Multi-device control")
    print("  0 - Run all demos (1-6)")
    print_init_usage("python hand_demo.py")


async def main():
    """Main entry point"""
    # Check for help
    if '-h' in sys.argv or '--help' in sys.argv:
        print_usage()
        return
    
    # Create extra parser for demo-specific args
    extra_parser = argparse.ArgumentParser(add_help=False)
    extra_parser.add_argument('demo_num', nargs='?', type=int, default=None,
                             help='Demo number to run (1-8, 0=all)')
    
    print("=== Stark Hand Demo ===\n")
    
    # Parse args and initialize
    device_ctx, extra_args, remaining = await parse_args_and_init(sys.argv, extra_parser)
    if device_ctx is None:
        return
    
    demo = HandDemo(device_ctx)
    demo_num = extra_args.demo_num if extra_args else None
    
    try:
        if demo_num is not None:
            if demo_num == 0:
                await demo.run_all_demos()
            else:
                await demo.run_demo(demo_num)
        else:
            # Interactive menu
            while True:
                choice = demo.show_menu()
                if choice == -1:
                    break
                elif choice == 0:
                    await demo.run_all_demos()
                else:
                    await demo.run_demo(choice)
    finally:
        await cleanup_context(device_ctx)
    
    print("\n=== Demo completed ===")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nUser interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
