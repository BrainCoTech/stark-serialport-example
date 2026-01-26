"""
CAN Adapter Abstraction Layer
Provides unified interface for ZLG and ZQWL CAN adapters
"""

import platform
from abc import ABC, abstractmethod
from typing import Optional, Tuple
from can_utils import logger


class CANAdapter(ABC):
    """Abstract base class for CAN adapters"""

    @abstractmethod
    def open(self) -> bool:
        """Open CAN device"""
        pass

    @abstractmethod
    def close(self):
        """Close CAN device"""
        pass

    @abstractmethod
    def send_message(self, can_id: int, data: bytes) -> bool:
        """Send CAN message"""
        pass

    @abstractmethod
    def receive_message(
        self, quick_retries: int = 2, dely_retries: int = 0
    ) -> Optional[Tuple[int, list]]:
        """Receive CAN message"""
        pass


class ZLGAdapter(CANAdapter):
    """ZLG CAN adapter implementation"""

    def __init__(self):
        self.system = platform.system()
        if self.system == "Windows":
            from zlg_win import (
                zlgcan_open,
                zlgcan_close,
                zlgcan_send_message,
                zlgcan_receive_message,
            )
        elif self.system == "Linux":
            from zlg_linux import (
                zlgcan_open,
                zlgcan_close,
                zlgcan_send_message,
                zlgcan_receive_message,
            )
        else:
            raise NotImplementedError(f"Unsupported OS: {self.system}")

        self._open = zlgcan_open
        self._close = zlgcan_close
        self._send = zlgcan_send_message
        self._receive = zlgcan_receive_message

    def open(self) -> bool:
        """Open ZLG CAN device"""
        try:
            self._open()
            logger.info(f"ZLG CAN adapter opened successfully on {self.system}")
            return True
        except Exception as e:
            logger.error(f"Failed to open ZLG adapter: {e}")
            return False

    def close(self):
        """Close ZLG CAN device"""
        try:
            self._close()
            logger.info("ZLG CAN adapter closed")
        except Exception as e:
            logger.error(f"Error closing ZLG adapter: {e}")

    def send_message(self, can_id: int, data: bytes) -> bool:
        """Send CAN message via ZLG"""
        return self._send(can_id, data)

    def receive_message(
        self, quick_retries: int = 2, dely_retries: int = 0
    ) -> Optional[Tuple[int, list]]:
        """Receive CAN message via ZLG"""
        return self._receive(quick_retries, dely_retries)


class ZQWLAdapter(CANAdapter):
    """ZQWL CAN adapter implementation"""

    def __init__(self, device_type: int = 4, channel: int = 0, baudrate: int = 1000000):
        """
        Initialize ZQWL adapter

        Args:
            device_type: Device type (default 4 for USBCAN-2E-U)
            channel: Channel number (default 0)
            baudrate: Baud rate in bps (default 1000000 = 1Mbps)
        """
        self.device_type = device_type
        self.channel = channel
        self.baudrate = baudrate

        if platform.system() != "Windows":
            raise NotImplementedError("ZQWL adapter only supports Windows")

        from zqwl_win import zcan_open, zcan_close, zcan_send_message, zqwl_can_receive_message

        self._open = zcan_open
        self._close = zcan_close
        self._send = zcan_send_message
        self._receive = zqwl_can_receive_message

    def open(self) -> bool:
        """Open ZQWL CAN device"""
        try:
            self._open(self.device_type, self.channel, self.baudrate)
            logger.info(
                f"ZQWL CAN adapter opened successfully (device_type={self.device_type}, "
                f"channel={self.channel}, baudrate={self.baudrate})"
            )
            return True
        except Exception as e:
            logger.error(f"Failed to open ZQWL adapter: {e}")
            return False

    def close(self):
        """Close ZQWL CAN device"""
        try:
            self._close()
            logger.info("ZQWL CAN adapter closed")
        except Exception as e:
            logger.error(f"Error closing ZQWL adapter: {e}")

    def send_message(self, can_id: int, data: bytes) -> bool:
        """Send CAN message via ZQWL"""
        # ZQWL send_message expects slave_id as first parameter (not used)
        return self._send(0, can_id, data)

    def receive_message(
        self, quick_retries: int = 2, dely_retries: int = 0
    ) -> Optional[Tuple[int, list]]:
        """Receive CAN message via ZQWL"""
        return self._receive(quick_retries, dely_retries)


def create_adapter(adapter_type: str = "zlg", **kwargs) -> CANAdapter:
    """
    Factory function to create CAN adapter

    Args:
        adapter_type: Type of adapter ("zlg" or "zqwl")
        **kwargs: Additional arguments for adapter initialization
            For ZQWL:
                - device_type: int (default 4)
                - channel: int (default 0)
                - baudrate: int (default 1000000)

    Returns:
        CANAdapter instance

    Example:
        # Create ZLG adapter
        adapter = create_adapter("zlg")

        # Create ZQWL adapter with custom settings
        adapter = create_adapter("zqwl", device_type=4, channel=0, baudrate=1000000)
    """
    adapter_type = adapter_type.lower()

    if adapter_type == "zlg":
        return ZLGAdapter()
    elif adapter_type == "zqwl":
        return ZQWLAdapter(**kwargs)
    else:
        raise ValueError(f"Unknown adapter type: {adapter_type}. Use 'zlg' or 'zqwl'")
