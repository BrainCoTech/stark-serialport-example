"""Shared Data Manager - Centralized DataCollector for all panels

This module provides a singleton DataCollector that can be shared across
multiple GUI panels. This avoids resource conflicts and ensures data consistency.

Usage:
    from .shared_data import SharedDataManager
    
    # In main_window, create and start:
    data_manager = SharedDataManager()
    data_manager.set_device(device, slave_id, device_info)
    data_manager.start()
    
    # In panels, read from buffers:
    motor_data = data_manager.get_latest_motor()
    touch_data = data_manager.get_latest_touch()
"""

import sys
import platform
from pathlib import Path
from typing import Optional, List
from PySide6.QtCore import QObject, Signal, QTimer

# Add parent directory to path for SDK import
sys.path.insert(0, str(Path(__file__).parent.parent))
from common_imports import sdk

from .constants import MOTOR_BUFFER_SIZE, TOUCH_BUFFER_SIZE, MOTOR_COUNT, TOUCH_COUNT


class SharedDataManager(QObject):
    """Centralized data collection manager
    
    Provides a single DataCollector instance that multiple panels can read from.
    This avoids resource conflicts when multiple panels need motor/touch data.
    
    Also serves as the central source of truth for device state (device, slave_id, device_info).
    Panels should access these via shared_data instead of maintaining their own copies.
    """
    
    # Signals for data updates (panels can connect to these)
    motor_updated = Signal(object)  # MotorStatusData
    touch_updated = Signal(list)    # List of TouchFingerItem (capacitive)
    pressure_touch_updated = Signal(list, list)  # (summary_data, detailed_data) for pressure touch
    connection_lost = Signal()      # Emitted when device connection is lost
    slave_id_updated = Signal(int)  # Emitted when slave_id changes
    
    def __init__(self):
        super().__init__()
        
        # Device state (central source of truth)
        self._device = None
        self._slave_id = 1
        self._device_info = None
        
        # SDK components
        self.motor_buffer = None
        self.touch_buffer = None  # Capacitive touch
        self.pressure_summary_buffer = None  # Pressure touch summary
        self.pressure_detailed_buffer = None  # Pressure touch detailed
        self.data_collector = None
        
        # Touch type flags
        self._is_pressure_touch = False
        
        # State
        self.is_running = False
        self._consecutive_no_new_data = 0
        self._max_no_new_data = 40  # Trigger disconnect after 40 cycles with no new data (2s at 50ms interval)
        self._last_buffer_len = 0
        self._enable_disconnect_detection = False  # Disabled for now, can be enabled later
        
        # Timer for emitting updates to connected panels
        self._update_timer = QTimer()
        self._update_timer.timeout.connect(self._emit_updates)
        self._update_timer.setInterval(50)  # 20Hz UI updates
    
    # Properties for device state access
    @property
    def device(self):
        return self._device
    
    @property
    def slave_id(self) -> int:
        return self._slave_id
    
    @property
    def device_info(self):
        return self._device_info
    
    @property
    def hw_type(self):
        """Get hardware type from device_info"""
        if self._device_info:
            return getattr(self._device_info, 'hardware_type', None)
        return None
    
    def set_device(self, device, slave_id: int, device_info):
        """Set device context"""
        # Stop existing collector if running
        if self.is_running:
            self.stop()
        
        self._device = device
        self._slave_id = slave_id
        self._device_info = device_info
        
        # Determine touch type using ctx.uses_pressure_touch_api (reads from saved hw_type)
        self._is_pressure_touch = False
        if device and device_info and device_info.is_touch():
            # Use ctx method which checks hw_type == Revo2TouchPressure
            self._is_pressure_touch = device.uses_pressure_touch_api(slave_id)
        
        # Create buffers
        if sdk and device:
            self.motor_buffer = sdk.MotorStatusBuffer(MOTOR_BUFFER_SIZE)
            
            # Create touch buffer based on device type
            if device_info and device_info.is_touch():
                if self._is_pressure_touch:
                    # Pressure touch: create summary and detailed buffers
                    self.pressure_summary_buffer = sdk.PressureSummaryBuffer(TOUCH_BUFFER_SIZE)
                    self.pressure_detailed_buffer = sdk.PressureDetailedBuffer(TOUCH_BUFFER_SIZE)
                    self.touch_buffer = None
                else:
                    # Capacitive touch
                    self.touch_buffer = sdk.TouchStatusBuffer(TOUCH_BUFFER_SIZE)
                    self.pressure_summary_buffer = None
                    self.pressure_detailed_buffer = None
            else:
                self.touch_buffer = None
                self.pressure_summary_buffer = None
                self.pressure_detailed_buffer = None
    
    def clear_device(self):
        """Clear device and stop collection"""
        self.stop()
        self._device = None
        self._slave_id = 1
        self._device_info = None
        self._is_pressure_touch = False
        self.motor_buffer = None
        self.touch_buffer = None
        self.pressure_summary_buffer = None
        self.pressure_detailed_buffer = None
    
    def update_slave_id(self, new_id: int):
        """Update slave_id and restart data collector
        
        This is needed when slave_id is changed at runtime (e.g., via config panel).
        The DataCollector needs to be restarted with the new slave_id.
        """
        if new_id == self._slave_id:
            return
        
        print(f"[SharedDataManager] Updating slave_id: {self._slave_id} -> {new_id}")
        old_running = self.is_running
        
        # Stop current collector
        if self.is_running:
            self.stop()
        
        self._slave_id = new_id
        
        # Notify listeners
        self.slave_id_updated.emit(new_id)
        
        # Restart collector with new slave_id after a short delay
        # This gives the device time to fully switch to the new ID
        if old_running and self._device:
            from PySide6.QtCore import QTimer
            QTimer.singleShot(500, self.start)  # 500ms delay
    
    def start(self):
        """Start data collection"""
        if not self._device or not sdk or self.is_running:
            return False
        
        # Determine frequency based on platform
        is_linux = platform.system() == "Linux"
        motor_freq = 200 if is_linux else 60
        touch_freq = 10
        
        # Create DataCollector based on touch type
        if self._is_pressure_touch and self.pressure_summary_buffer and self.motor_buffer:
            # Pressure touch: use hybrid mode (summary + detailed)
            if not self.pressure_detailed_buffer:
                print("[SharedDataManager] Missing pressure_detailed_buffer")
                return False
            self.data_collector = sdk.DataCollector.new_pressure_hybrid(
                self._device,
                self.motor_buffer,
                self.pressure_summary_buffer,
                self.pressure_detailed_buffer,
                slave_id=self._slave_id,
                motor_frequency=motor_freq,
                summary_frequency=20,  # 20Hz for summary
                detailed_frequency=5,  # 5Hz for detailed (slower, more data)
                enable_stats=False
            )
            print(f"[SharedDataManager] Started: {motor_freq}Hz motor, 20Hz pressure summary, 5Hz pressure detailed")
        elif self.touch_buffer and self.motor_buffer:
            # Capacitive touch
            self.data_collector = sdk.DataCollector.new_capacitive(
                self._device,
                self.motor_buffer,
                self.touch_buffer,
                slave_id=self._slave_id,
                motor_frequency=motor_freq,
                touch_frequency=touch_freq,
                enable_stats=False
            )
            print(f"[SharedDataManager] Started: {motor_freq}Hz motor, {touch_freq}Hz capacitive touch")
        elif self.motor_buffer:
            # Basic (no touch)
            self.data_collector = sdk.DataCollector.new_basic(
                self._device,
                self.motor_buffer,
                slave_id=self._slave_id,
                motor_frequency=motor_freq,
                enable_stats=False
            )
            print(f"[SharedDataManager] Started: {motor_freq}Hz motor (no touch)")
        else:
            print("[SharedDataManager] Missing motor_buffer")
            return False
        
        self.data_collector.start()
        self.is_running = True
        self._consecutive_no_new_data = 0
        self._last_buffer_len = 0
        self._update_timer.start()
        
        return True
    
    def stop(self):
        """Stop data collection"""
        if not self.is_running:
            return
        
        self._update_timer.stop()
        
        if self.data_collector:
            self.data_collector.stop()
            self.data_collector.wait()
            self.data_collector = None
        
        self.is_running = False
        print("[SharedDataManager] Stopped")
    
    def _emit_updates(self):
        """Emit update signals for connected panels"""
        if not self.is_running:
            return
        
        # Check if new data is coming in by monitoring buffer length
        # (Disconnect detection - currently disabled)
        if self._enable_disconnect_detection:
            current_len = self.get_motor_buffer_len()
            if current_len != self._last_buffer_len:
                # New data arrived
                self._consecutive_no_new_data = 0
                self._last_buffer_len = current_len
            else:
                # No new data
                self._consecutive_no_new_data += 1
                if self._consecutive_no_new_data >= self._max_no_new_data:
                    print(f"[SharedDataManager] Connection lost (no new data for {self._max_no_new_data} cycles)")
                    self.connection_lost.emit()
                    self._consecutive_no_new_data = 0  # Reset to avoid repeated signals
                    return
        
        # Get latest motor data
        motor = self.get_latest_motor()
        if motor:
            self.motor_updated.emit(motor)
        
        # Get latest touch data based on type
        if self._is_pressure_touch:
            # Pressure touch: emit summary and detailed data
            summary = self.get_latest_pressure_summary()
            detailed = self.get_latest_pressure_detailed()
            if summary or detailed:
                self.pressure_touch_updated.emit(summary or [], detailed or [])
        elif self.touch_buffer:
            # Capacitive touch
            touch = self.get_latest_touch()
            if touch:
                self.touch_updated.emit(touch)
    
    # ==========================================================================
    # Data Access Methods (for panels to use)
    # ==========================================================================
    
    def get_latest_motor(self) -> Optional[object]:
        """Get latest motor status (non-blocking)
        
        Returns:
            MotorStatusData or None
        """
        if not self.motor_buffer:
            return None
        return self.motor_buffer.peek_latest()
    
    def get_all_motor(self) -> List:
        """Get all buffered motor data and clear buffer
        
        Returns:
            List of MotorStatusData
        """
        if not self.motor_buffer:
            return []
        return self.motor_buffer.pop_all()
    
    def get_latest_touch(self) -> Optional[List]:
        """Get latest touch data for all fingers (non-blocking)
        
        Returns:
            List of TouchFingerItem (one per finger) or None
        """
        if not self.touch_buffer:
            return None
        return self.touch_buffer.pop_latest_all()
    
    def get_all_touch(self) -> List:
        """Get all buffered touch data and clear buffer
        
        Returns:
            List of List of TouchFingerItem (per finger)
        """
        if not self.touch_buffer:
            return []
        return self.touch_buffer.pop_all()
    
    def get_motor_buffer_len(self) -> int:
        """Get number of items in motor buffer"""
        if not self.motor_buffer:
            return 0
        return self.motor_buffer.len()
    
    def has_touch(self) -> bool:
        """Check if touch data is available"""
        return self.touch_buffer is not None
    
    def has_pressure_touch(self) -> bool:
        """Check if pressure touch data is available"""
        return self._is_pressure_touch and self.pressure_summary_buffer is not None
    
    def get_latest_pressure_summary(self) -> Optional[List]:
        """Get latest pressure touch summary data (6 values: 5 fingers + palm)
        
        Returns:
            List of int (pressure values in mN) or None
        """
        if not self.pressure_summary_buffer:
            return None
        return self.pressure_summary_buffer.pop_latest_all()
    
    def get_latest_pressure_detailed(self) -> Optional[List]:
        """Get latest pressure touch detailed data (91 sampling points)
        
        Returns:
            List of PressureDetailedItem or None
        """
        if not self.pressure_detailed_buffer:
            return None
        return self.pressure_detailed_buffer.pop_latest_all()
    
    # ==========================================================================
    # Convenience Methods
    # ==========================================================================
    
    def get_positions(self) -> List[int]:
        """Get latest finger positions"""
        motor = self.get_latest_motor()
        if motor and motor.positions:
            return list(motor.positions)
        return [0] * MOTOR_COUNT
    
    def get_speeds(self) -> List[int]:
        """Get latest finger speeds"""
        motor = self.get_latest_motor()
        if motor and motor.speeds:
            return list(motor.speeds)
        return [0] * MOTOR_COUNT
    
    def get_currents(self) -> List[int]:
        """Get latest finger currents"""
        motor = self.get_latest_motor()
        if motor and motor.currents:
            return list(motor.currents)
        return [0] * MOTOR_COUNT
    
    def get_states(self) -> List:
        """Get latest motor states"""
        motor = self.get_latest_motor()
        if motor and motor.states:
            return list(motor.states)
        return [0] * MOTOR_COUNT
