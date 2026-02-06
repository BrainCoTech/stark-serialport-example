import asyncio
import os
import platform
import sys

from can_utils import *

if platform.system() != "Linux":
    raise NotImplementedError("SocketCAN is only supported on Linux.")

from socketcan_linux_utils import *


class Revo1CanController:
    """Revo1 CAN communication controller (SocketCAN)"""

    def __init__(self, master_id: int = 1, slave_id: int = 1):
        self.master_id = master_id
        self.slave_id = slave_id
        self.client = libstark.init_device_handler(libstark.StarkProtocolType.Can, master_id)

    async def initialize(self):
        try:
            socketcan_open()
            libstark.set_can_tx_callback(self._can_send)
            libstark.set_can_rx_callback(self._can_read)
            logger.info(
                f"CAN connection initialized successfully - Master ID: {self.master_id}, Slave ID: {self.slave_id}"
            )
            return True
        except Exception as e:
            logger.error(f"CAN connection initialization failed: {e}")
            return False

    def _can_send(self, _slave_id: int, can_id: int, data: list) -> bool:
        try:
            if not socketcan_send_message(can_id, bytes(data)):
                logger.error("CAN message sending failed")
                return False
            return True
        except Exception as e:
            logger.error(f"CAN message sending failed: {e}")
            return False

    def _can_read(self, _slave_id: int, expected_can_id: int, expected_frames: int) -> tuple:
        """
        CAN message receiving with multi-frame support
        
        SDK tells us how many frames to expect via expected_frames parameter.
        We need to collect all frames and return concatenated data.
        
        Args:
            _slave_id: Slave ID (not used)
            expected_can_id: Expected CAN ID
            expected_frames: Expected frame count (0 = single frame, >0 = multi-frame)
            
        Returns:
            tuple: (can_id, data)
        """
        try:
            all_data = []
            received_count = 0
            target_frames = expected_frames if expected_frames > 0 else 1
            max_attempts = 30 if expected_frames > 1 else 10
            
            for attempt in range(max_attempts):
                recv_msg = socketcan_receive_message()
                if recv_msg is None:
                    import time
                    wait_ms = 0.005 if attempt < 5 else 0.010
                    time.sleep(wait_ms)
                    continue
                
                can_id, data = recv_msg
                
                # socketcan_receive_message may return multiple frames concatenated
                # Each CAN 2.0 frame is 8 bytes max
                frame_size = 8
                num_frames_in_batch = (len(data) + frame_size - 1) // frame_size
                
                all_data.extend(data)
                received_count += num_frames_in_batch
                
                # Check if we have enough frames
                if received_count >= target_frames:
                    return expected_can_id, bytes(all_data)
                
                # For single frame request, return immediately
                if expected_frames <= 1:
                    return can_id, bytes(data)
            
            # Timeout - return whatever we have
            if all_data:
                return expected_can_id, bytes(all_data)
            
            return 0, bytes([])
        except Exception as e:
            logger.error(f"CAN message receiving failed: {e}")
            return 0, bytes([])

    async def get_device_info(self):
        try:
            device_info = await self.client.get_device_info(self.slave_id)
            logger.info(f"Device information: {device_info.description}")
            return device_info
        except Exception as e:
            logger.error(f"Failed to get device information: {e}")
            return None

    async def change_slave_id(self, new_slave_id: int):
        try:
            await self.client.set_slave_id(self.slave_id, new_slave_id)
            logger.info(f"Slave ID modified to {new_slave_id}, device will restart...")
            return True
        except Exception as e:
            logger.error(f"Failed to modify slave ID: {e}")
            return False

    async def configure_device(self):
        try:
            return True
        except Exception as e:
            logger.error(f"Failed to configure device: {e}")
            return False

    async def configure_turbo_mode(self):
        try:
            await self.client.set_turbo_mode_enabled(self.slave_id, True)
            turbo_interval = 200
            turbo_duration = 300
            turbo_conf = libstark.TurboConfig(turbo_interval, turbo_duration)
            await self.client.set_turbo_config(self.slave_id, turbo_conf)
            turbo_mode_enabled = await self.client.get_turbo_mode_enabled(self.slave_id)
            turbo_config = await self.client.get_turbo_config(self.slave_id)
            logger.info(f"Turbo mode: {turbo_mode_enabled}")
            logger.info(
                f"Turbo configuration - interval: {turbo_config.interval}ms, duration: {turbo_config.duration}ms"
            )
        except Exception as e:
            logger.error(f"Failed to configure Turbo mode: {e}")

    async def finger_position_examples(self):
        logger.info("=== Finger position control example ===")
        positions = [
            [200, 0, 0, 0, 0, 0],
            [200, 300, 0, 0, 0, 0],
            [200, 300, 500, 0, 0, 0],
            [200, 300, 500, 700, 0, 0],
            [200, 300, 500, 700, 800, 0],
            [200, 300, 500, 700, 800, 900],
        ]

        for i, pos in enumerate(positions):
            logger.info(f"Step {i+1}: {pos}")
            await self.client.set_finger_positions(self.slave_id, pos)
            await asyncio.sleep(0.8)

    async def control_examples(self):
        try:
            await self.finger_position_examples()
        except Exception as e:
            logger.error(f"Control example execution failed: {e}")

    async def run(self):
        if not await self.initialize():
            return
        if await self.get_device_info() is None:
            return
        await self.configure_device()
        await self.control_examples()

    def cleanup(self):
        socketcan_close()


async def main():
    try:
        master_id = int(os.getenv("STARK_MASTER_ID", "1"), 0)
        slave_id = int(os.getenv("STARK_SLAVE_ID", "1"), 0)
        controller = Revo1CanController(master_id, slave_id)
        await controller.run()

        shutdown_event = setup_shutdown_event(logger)
        await shutdown_event.wait()
        logger.info("Received shutdown signal, stopping...")
    except Exception as e:
        logger.error(f"Program execution exception: {e}")
    finally:
        controller.cleanup()
        sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
