#!/usr/bin/env python3
"""
Stark Hand Monitor - Real-time High-Performance Data Monitor

Auto-detects device and displays real-time motor/touch data using DataCollector.
Supports: Modbus (RS485), Protobuf, ZQWL CAN/CANFD, ZLG CAN, SocketCAN

Run:
    python hand_monitor.py                      # Auto-detect, auto-select mode
    python hand_monitor.py motor                # Motor data only
    python hand_monitor.py touch                # Motor + touch
    python hand_monitor.py summary              # Motor + pressure summary
    python hand_monitor.py -p /dev/ttyUSB0 motor  # Protobuf, motor mode
    python hand_monitor.py -s can0 1 motor      # SocketCAN, slave_id=1, motor mode
    python hand_monitor.py -z 2 touch           # ZLG CAN, slave_id=2, touch mode

Options:
    --duration N        Collection duration in seconds (default: 30)
"""

import asyncio
import sys
import os
import platform
import argparse

# Setup path and imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common_imports import (
    sdk, check_sdk, get_hw_type_name, logger,
    has_touch, has_pressure_touch, uses_revo1_touch_api
)
from common_init import (
    DeviceContext, parse_args_and_init, cleanup_context, print_init_usage
)
from common_utils import setup_shutdown_event

check_sdk()


async def monitor_motor(ctx, slave_id: int, shutdown_event, duration: int = 30):
    """Monitor motor data with DataCollector"""
    print("\n[Motor Monitor] Press Ctrl+C to stop\n")

    is_linux = platform.system() == "Linux"
    motor_freq = 100 if is_linux else 50
    print(f"Mode: {motor_freq}Hz motor data collection")
    print(f"Duration: {duration} seconds\n")

    try:
        motor_buffer = sdk.MotorStatusBuffer(max_size=10000)
        collector = sdk.DataCollector.new_basic(
            ctx=ctx,
            motor_buffer=motor_buffer,
            slave_id=slave_id,
            motor_frequency=motor_freq,
            enable_stats=True
        )

        collector.start()
        logger.info(f"DataCollector started ({motor_freq}Hz)")

        print("Finger:    Thumb  ThumbAux  Index  Middle  Ring  Pinky")
        print("-" * 60)

        for i in range(duration):
            await asyncio.sleep(1)
            if shutdown_event.is_set():
                break

            motor_data = motor_buffer.pop_all()
            if motor_data:
                latest = motor_data[-1]
                pos = list(latest.positions)
                print(f"[{i+1:2d}s] {len(motor_data):4d} samples | "
                      f"Pos: {pos[0]:5} {pos[1]:5} {pos[2]:5} {pos[3]:5} {pos[4]:5} {pos[5]:5}")
            else:
                print(f"[{i+1:2d}s] No data")

        collector.stop()
        collector.wait()
        logger.info("DataCollector stopped")

    except Exception as e:
        logger.error(f"Monitor error: {e}")
        import traceback
        traceback.print_exc()


def print_revo1_touch_finger(name: str, finger, finger_idx: int):
    """Print Revo1 Touch API finger data"""
    status_str = {0: "OK", 1: "DataErr", 2: "CommErr"}.get(finger.status, "Unknown")

    if finger_idx == 0:
        print(f"  {name:6s}: F1({finger.normal_force1:4d},{finger.tangential_force1:4d}) "
              f"F2({finger.normal_force2:4d},{finger.tangential_force2:4d}) "
              f"Prox={finger.self_proximity1:4d} [{status_str}]")
    elif finger_idx in [1, 2, 3]:
        print(f"  {name:6s}: F1({finger.normal_force1:4d},{finger.tangential_force1:4d}) "
              f"F2({finger.normal_force2:4d},{finger.tangential_force2:4d}) "
              f"F3({finger.normal_force3:4d},{finger.tangential_force3:4d}) "
              f"Prox=({finger.self_proximity1:4d},{finger.self_proximity2:4d},{finger.mutual_proximity:4d}) [{status_str}]")
    else:
        print(f"  {name:6s}: F1({finger.normal_force1:4d},{finger.tangential_force1:4d}) "
              f"F2({finger.normal_force2:4d},{finger.tangential_force2:4d}) "
              f"Prox=({finger.self_proximity1:4d},{finger.mutual_proximity:4d}) [{status_str}]")


async def monitor_touch(ctx, slave_id: int, hw_type, shutdown_event, duration: int = 30, protocol_type=None):
    """Monitor motor + touch data with DataCollector"""
    print("\n[Touch Monitor] Press Ctrl+C to stop\n")

    is_can = protocol_type == sdk.StarkProtocolType.Can if protocol_type else False
    is_linux = platform.system() == "Linux"

    if is_can:
        motor_freq, touch_freq = 20, 5
    elif is_linux:
        motor_freq, touch_freq = 100, 10
    else:
        motor_freq, touch_freq = 50, 10

    print(f"Mode: {motor_freq}Hz motor + {touch_freq}Hz touch")
    print(f"Duration: {duration} seconds\n")

    try:
        motor_buffer = sdk.MotorStatusBuffer(max_size=10000)
        touch_buffer = sdk.TouchStatusBuffer(max_size=1000)

        collector = sdk.DataCollector.new_capacitive(
            ctx=ctx,
            motor_buffer=motor_buffer,
            touch_buffer=touch_buffer,
            slave_id=slave_id,
            motor_frequency=motor_freq,
            touch_frequency=touch_freq,
            enable_stats=True
        )

        collector.start()
        logger.info(f"DataCollector started ({motor_freq}Hz motor + {touch_freq}Hz touch)")

        finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
        uses_revo1_touch = uses_revo1_touch_api(hw_type) if hw_type else False

        for i in range(duration):
            await asyncio.sleep(1)
            if shutdown_event.is_set():
                break

            motor_data = motor_buffer.pop_all()
            touch_data_all = touch_buffer.pop_all()
            touch_counts = [len(data) for data in touch_data_all]

            print(f"[{i+1:2d}s] Motor: {len(motor_data):4d} samples | Touch: {touch_counts}")

            for finger_idx, finger_data in enumerate(touch_data_all):
                if len(finger_data) > 0:
                    latest = finger_data[-1]
                    if uses_revo1_touch:
                        print_revo1_touch_finger(finger_names[finger_idx], latest, finger_idx)
                    else:
                        print(f"  {finger_names[finger_idx]}: "
                              f"Normal={latest.normal_force1:4d}, "
                              f"Tangential={latest.tangential_force1:4d}, "
                              f"Status={latest.status}")

        collector.stop()
        collector.wait()
        logger.info("DataCollector stopped")

    except Exception as e:
        logger.error(f"Touch monitor error: {e}")
        import traceback
        traceback.print_exc()


async def monitor_pressure_summary(ctx, slave_id: int, shutdown_event, duration: int = 30):
    """Monitor motor + pressure touch summary"""
    print("\n[Pressure Summary Monitor] Press Ctrl+C to stop\n")
    print("Mode: 100Hz motor + 10Hz pressure summary")
    print(f"Duration: {duration} seconds\n")
    print("Finger:  Thumb  Index  Middle  Ring  Pinky  Palm")
    print("-" * 50)

    try:
        motor_buffer = sdk.MotorStatusBuffer(max_size=10000)
        pressure_buffer = sdk.PressureSummaryBuffer(max_size=1000)

        collector = sdk.DataCollector.new_pressure_summary(
            ctx=ctx,
            motor_buffer=motor_buffer,
            pressure_summary_buffer=pressure_buffer,
            slave_id=slave_id,
            motor_frequency=100,
            touch_frequency=10,
            enable_stats=True
        )

        collector.start()
        logger.info("DataCollector started (100Hz motor + 10Hz pressure)")

        for i in range(duration):
            await asyncio.sleep(1)
            if shutdown_event.is_set():
                break

            motor_data = motor_buffer.pop_all()
            pressure_data = pressure_buffer.pop_all()

            if pressure_data:
                latest = [data[-1] if data else 0 for data in pressure_data]
                print(f"[{i+1:2d}s] {sum(len(d) for d in pressure_data):3d} samples | "
                      f"Pressure: {latest[0]:5} {latest[1]:5} {latest[2]:5} "
                      f"{latest[3]:5} {latest[4]:5} {latest[5]:5}")
            else:
                print(f"[{i+1:2d}s] No data")

        collector.stop()
        collector.wait()
        logger.info("DataCollector stopped")

    except Exception as e:
        logger.error(f"Pressure monitor error: {e}")
        import traceback
        traceback.print_exc()


async def monitor_pressure_detailed(ctx, slave_id: int, shutdown_event, duration: int = 30):
    """Monitor motor + pressure touch detailed"""
    print("\n[Pressure Detailed Monitor] Press Ctrl+C to stop\n")
    print("Mode: 100Hz motor + 10Hz pressure detailed")
    print(f"Duration: {duration} seconds\n")

    try:
        motor_buffer = sdk.MotorStatusBuffer(max_size=10000)
        pressure_buffer = sdk.PressureDetailedBuffer(max_size=1000)

        collector = sdk.DataCollector.new_pressure_detailed(
            ctx=ctx,
            motor_buffer=motor_buffer,
            pressure_detailed_buffer=pressure_buffer,
            slave_id=slave_id,
            motor_frequency=100,
            touch_frequency=10,
            enable_stats=True
        )

        collector.start()
        logger.info("DataCollector started (100Hz motor + 10Hz pressure detailed)")

        for i in range(duration):
            await asyncio.sleep(1)
            if shutdown_event.is_set():
                break

            motor_data = motor_buffer.pop_all()
            pressure_data = pressure_buffer.pop_all()

            if pressure_data:
                thumb_data = pressure_data[0] if len(pressure_data) > 0 else []
                index_data = pressure_data[1] if len(pressure_data) > 1 else []
                print(f"[{i+1:2d}s] {sum(len(d) for d in pressure_data):3d} samples | "
                      f"Thumb: {thumb_data[-1].sensors[:3] if thumb_data else 'N/A'} | "
                      f"Index: {index_data[-1].sensors[:3] if index_data else 'N/A'}")
            else:
                print(f"[{i+1:2d}s] No data")

        collector.stop()
        collector.wait()
        logger.info("DataCollector stopped")

    except Exception as e:
        logger.error(f"Pressure detailed monitor error: {e}")
        import traceback
        traceback.print_exc()


def print_usage():
    print("Usage: python hand_monitor.py [options] [mode]")
    print("\nModes:")
    print("  motor     - Motor data only (100Hz)")
    print("  touch     - Motor + capacitive touch (100Hz + 10Hz)")
    print("  summary   - Motor + pressure summary (100Hz + 10Hz)")
    print("  detailed  - Motor + pressure detailed (100Hz + 10Hz)")
    print("\nOptions:")
    print("  --duration N        Collection duration in seconds (default: 30)")
    print_init_usage("python hand_monitor.py")
    print("\nExamples:")
    print("  python hand_monitor.py motor")
    print("  python hand_monitor.py touch --duration 60")
    print("  python hand_monitor.py -s can0 1 motor")
    print("  python hand_monitor.py -z 2 touch")


async def main():
    # Check for help
    if '-h' in sys.argv or '--help' in sys.argv:
        print_usage()
        return

    # Create extra parser for monitor-specific args
    extra_parser = argparse.ArgumentParser(add_help=False)
    extra_parser.add_argument('mode', nargs='?', default=None,
                             choices=['motor', 'touch', 'summary', 'detailed'],
                             help='Monitor mode')
    extra_parser.add_argument('--duration', type=int, default=30,
                             help='Collection duration (seconds)')

    print("=== Stark Hand Monitor ===\n")

    # Parse args and initialize
    device_ctx, extra_args, remaining = await parse_args_and_init(sys.argv, extra_parser)
    if device_ctx is None:
        return

    mode = extra_args.mode if extra_args else None
    duration = extra_args.duration if extra_args else 30

    # Auto-select mode if not specified
    if mode is None:
        if has_pressure_touch(device_ctx.hw_type):
            mode = "summary"
        elif has_touch(device_ctx.hw_type):
            mode = "touch"
        else:
            mode = "motor"
        logger.info(f"Auto-selected mode: {mode}")

    # Setup shutdown
    shutdown_event = setup_shutdown_event()

    try:
        if mode == "motor":
            await monitor_motor(device_ctx.ctx, device_ctx.slave_id, shutdown_event, duration)
        elif mode == "touch":
            await monitor_touch(device_ctx.ctx, device_ctx.slave_id, device_ctx.hw_type, 
                              shutdown_event, duration, device_ctx.protocol_type)
        elif mode == "summary":
            await monitor_pressure_summary(device_ctx.ctx, device_ctx.slave_id, shutdown_event, duration)
        elif mode == "detailed":
            await monitor_pressure_detailed(device_ctx.ctx, device_ctx.slave_id, shutdown_event, duration)
    finally:
        await cleanup_context(device_ctx)
        logger.info("Monitor stopped")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nUser interrupted")
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
