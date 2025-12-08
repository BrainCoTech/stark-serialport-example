"""
Revo2 action sequence example

This example demonstrates how to use the action sequence feature of Revo2 dexterous hand, including:
- Creating and configuring custom action sequences
- Using multiple control modes
- Uploading and storing action sequences
- Executing built-in action sequences
- Executing and managing custom action sequences
- Reading and verifying action sequence data

Notes:
- Revo2 supports multiple control modes: position time control, position speed control, current control, speed control
- When the position value is 65535 (0xFFFF), it means the finger maintains the original angle
- Action sequence parameters use physical quantities (angle, speed, current)
- The device supports up to 6 custom action sequences
"""

import asyncio
import sys
from utils import setup_shutdown_event
from revo2_utils import *

# Revo2 action sequence configuration
# Revo2 supports more complex action sequences, including multiple control modes and detailed parameter configurations
sample_action_sequences = [
    {
        "index": 0,                          # Action sequence index, used to identify the position of this action sequence in the queue
        "duration_ms": 1000,                 # Action sequence execution time (milliseconds)
        "mode": 1,                           # Control mode: 1=position time control, 2=position speed control, 3=current control, 4=speed control
        "positions": [24, 0, 0, 0, 0, 0],    # Finger positions (°): [thumb, index, middle, ring, little, wrist]
        "durations": [500, 100, 100, 100, 100, 100],  # Time for each finger to reach the target position (milliseconds)
        "speeds": [0, 0, 0, 0, 0, 0],        # Finger speed (°/s): only effective in position speed control mode
        "currents": [0, 0, 0, 0, 0, 0],      # Finger current (mA): only effective in current control mode
    },
    {
        "index": 1,
        "duration_ms": 2000,
        "mode": 1,                           # Position time control
        "positions": [45, 44, 84, 84, 84, 84],  # Execute fist gesture: each finger closes to the specified angle
        "durations": [2000, 2000, 100, 100, 100, 100],  # Thumb and index finger slowly close, other fingers close quickly
        "speeds": [0, 0, 0, 0, 0, 0],
        "currents": [0, 0, 0, 0, 0, 0],
    },
    {
        "index": 2,
        "duration_ms": 1000,
        "mode": 1,
        "positions": [24, 0, 0, 0, 0, 0],    # Thumb partially closed, other fingers fully opened
        "durations": [500, 100, 100, 100, 100, 100],
        "speeds": [0, 0, 0, 0, 0, 0],
        "currents": [0, 0, 0, 0, 0, 0],
    },
    {
        "index": 3,
        "duration_ms": 2000,
        "mode": 1,
        "positions": [45, 44, 84, 84, 84, 84],  # Repeat fist gesture
        "durations": [2000, 2000, 100, 100, 100, 100],
        "speeds": [0, 0, 0, 0, 0, 0],
        "currents": [0, 0, 0, 0, 0, 0],
    },
]


def action_sequence_info_to_list(action):
    """
    Convert action sequence dictionary to list format

    Convert the configuration information of the action sequence to the list format required by the SDK, for uploading to the device.
    The action sequence format of Revo2 is more complex than Revo1, including more control parameters.
    Args:
        action (dict): Action sequence configuration dictionary, containing index, duration_ms, mode, positions, durations, speeds, currents

    Returns:
        list: Converted list format [index, duration_ms, mode, positions..., durations..., speeds..., currents...]
    """
    return (
        [action["index"], action["duration_ms"], action["mode"]]
        + action["positions"]
        + action["durations"]
        + action["speeds"]
        + action["currents"]
    )


async def main():
    """
    Main function: initialize the dexterous hand and execute action sequence control
    """
    # Set shutdown event listener
    shutdown_event = setup_shutdown_event(logger)

    # Connect Revo2 device
    (client, slave_id) = await open_modbus_revo2(port_name=None) # Replace with actual port name, None will try to automatically detect

    # Convert action sequence format
    mapped_sequences = map(lambda action: action_sequence_info_to_list(action), sample_action_sequences)
    action_sequences = list(mapped_sequences)

    # Upload custom action sequence to device
    # The device supports up to 6 custom action sequences (CustomGesture1 ~ CustomGesture6)
    custom_action_id = libstark.ActionSequenceId.CustomGesture1
    await client.transfer_action_sequence(slave_id, custom_action_id, action_sequences)
    logger.info(f"Custom action sequence uploaded to {custom_action_id}")

    # Read and verify the uploaded action sequence
    # The device supports up to 6 built-in action sequences and up to 24 custom action sequences
    action_result: libstark.ActionSequenceItem = await client.get_action_sequence(slave_id, custom_action_id)
    logger.info(f"Action sequence: {action_result.description}")

    # Execute built-in action sequence example (optional)
    # logger.info("Executing built-in fist gesture...")
    # await client.run_action_sequence(slave_id, libstark.ActionSequenceId.DefaultGestureFist)
    # await asyncio.sleep(3)  # Wait for the built-in action sequence to execute

    # Execute custom action sequence
    logger.info("Executing custom action sequence...")
    await client.run_action_sequence(slave_id, custom_action_id)

    # logger.info("Clearing custom action sequence...")
    # await client.clear_action_sequence(slave_id, custom_action_id)
    # await client.run_action_sequence(slave_id, custom_action_id)

    # Wait for shutdown event (Ctrl+C or other shutdown signal)
    await shutdown_event.wait()

    # Clean up resources
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
