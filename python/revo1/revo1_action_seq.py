"""
Revo1 Dexterous Hand Action Sequence Control Example

This example demonstrates how to use the action sequence functionality of Revo1 dexterous hand, including:
- Creating and configuring custom action sequences
- Uploading and storing action sequences
- Executing built-in action sequences
- Executing and managing custom action sequences
- Reading and verifying action sequence data
"""

import asyncio
import sys
from utils import setup_shutdown_event
from revo1_utils import *

# Revo1 dexterous hand action sequence configuration
# Note: In current version, only positions parameter is actually effective, speeds and forces parameters are reserved fields
sample_action_sequences = [
    {
        "index": 0,                          # Action sequence index
        "duration_ms": 2000,                 # Action duration (milliseconds)
        "positions": [0, 0, 100, 100, 100, 100],  # Finger positions: [Thumb, Thumb Aux, Index, Middle, Ring, Pinky]
        "speeds": [10, 20, 30, 40, 50, 60],      # Finger speeds (reserved parameter)
        "forces": [5, 10, 15, 20, 25, 30],       # Finger forces (reserved parameter)
    },
    {
        "index": 1,
        "duration_ms": 2000,
        "positions": [100, 100, 0, 0, 0, 0],     # Thumb closed, other fingers open
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
    {
        "index": 2,
        "duration_ms": 2000,
        "positions": [0, 0, 100, 100, 100, 100], # Thumb open, other fingers closed
        "speeds": [10, 20, 30, 40, 50, 60],
        "forces": [5, 10, 15, 20, 25, 30],
    },
    {
        "index": 3,
        "duration_ms": 2000,
        "positions": [100, 100, 0, 0, 0, 0],
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
    {
        "index": 4,
        "duration_ms": 2000,
        "positions": [0, 0, 0, 0, 0, 0],         # All fingers fully open
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
    {
        "index": 5,
        "duration_ms": 2000,
        "positions": [0, 0, 0, 0, 0, 0],         # Maintain open state
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
    {
        "index": 6,
        "duration_ms": 2000,
        "positions": [0, 0, 0, 0, 0, 0],
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
    {
        "index": 7,
        "duration_ms": 2000,
        "positions": [0, 0, 0, 0, 0, 0],
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
    {
        "index": 8,
        "duration_ms": 2000,
        "positions": [0, 0, 0, 0, 0, 0],
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
    {
        "index": 9,
        "duration_ms": 2000,
        "positions": [0, 0, 0, 0, 0, 0],
        "speeds": [15, 25, 35, 45, 55, 65],
        "forces": [6, 11, 16, 21, 26, 31],
    },
]


def action_sequence_info_to_list(action):
    """
    Convert action sequence dictionary to list format

    Convert action sequence configuration information to list format required by SDK for uploading to device.

    Args:
        action (dict): Action sequence configuration dictionary, containing index, duration_ms, positions, speeds, forces

    Returns:
        list: Converted list format [index, duration_ms, positions..., speeds..., forces...]
    """
    return (
        [action["index"]]
        + [action["duration_ms"]]
        + action["positions"]
        + action["speeds"]
        + action["forces"]
    )


async def main():
    """
    Main function: Initialize dexterous hand and execute action sequence control
    """
    # Set up shutdown event listener
    shutdown_event = setup_shutdown_event(logger)

    # Detect baud rate and device ID of dexterous hand, initialize client
    (client, slave_id) = await open_modbus_revo1(port_name=None)  # Replace with actual serial port name, passing None will attempt auto-detection

    # Convert action sequence format
    # sample_action_sequences = random_action_sequences()  # Optional: Use random action sequences
    mapped_sequences = map(lambda action: action_sequence_info_to_list(action), sample_action_sequences)
    action_sequences = list(mapped_sequences)

    # Upload custom action sequence to device
    # Device supports up to 6 custom action sequences (CustomGesture1 ~ CustomGesture6)
    custom_action_id = libstark.ActionSequenceId.CustomGesture1
    await client.transfer_action_sequence(slave_id, custom_action_id, action_sequences)
    logger.info(f"Custom action sequence uploaded to {custom_action_id}")

    # Read and verify uploaded action sequence
    action_result: libstark.ActionSequenceItem = await client.get_action_sequence(slave_id, custom_action_id)
    logger.info(f"Action sequence: {action_result.description}")

    # Execute built-in action sequence: fist gesture
    logger.info("Executing built-in fist gesture...")
    await client.run_action_sequence(slave_id, libstark.ActionSequenceId.DefaultGestureFist)
    await asyncio.sleep(3)  # Wait for built-in action sequence to complete

    # Execute custom action sequence
    logger.info("Executing custom action sequence...")
    await client.run_action_sequence(slave_id, custom_action_id)

    # Wait for shutdown event (Ctrl+C or other shutdown signals)
    await shutdown_event.wait()

    # Clean up resources
    libstark.modbus_close(client)
    logger.info("Modbus client closed")
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
