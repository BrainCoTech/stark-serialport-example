# Stark SDK GUI

> ⚠️ **Note**: This GUI tool is intended for **debugging and development purposes only**. For stable operation and better user experience in production environments, please use the official [BrainCo RevoHand Software](https://www.brainco-hz.com/docs/revolimb-hand/index.html).

Unified GUI tool for controlling and monitoring Stark dexterous hand devices.

## Features

### Supported Protocols
- ✅ Modbus/RS485
- ✅ CAN 2.0 (ZQWL)
- ✅ CANFD (ZQWL)
- ✅ SocketCAN (Linux, via Auto Detect)
- 🚧 EtherCAT (Linux only)

### Supported Devices
- Revo1 Basic / Touch / Advanced / AdvancedTouch
- Revo2 Basic / Touch / TouchPressure

### Language Support
- English (Default)
- 中文 (Chinese)

Switch language via: Menu → Language → Select language

### Main Features

#### 1. Connection Management
- Multi-protocol support (Modbus, CAN, CANFD, SocketCAN)
- Auto device detection (includes SocketCAN on Linux)
- CAN bus scanning (IDs: 1,2 for CAN / 0x7E,0x7F for CANFD)
- Connection status monitoring

#### 2. Motor Control
- Position control mode
- Speed control mode
- Torque control mode
- Individual finger control
- Global control (Open All / Close All / Stop All)
- Real-time status display

#### 3. Touch Sensor
- Enable/Disable touch sensors (per finger)
- Calibration
- Reset
- Real-time touch data display
  - Normal force
  - Tangential force
  - Proximity
  - Sensor status
- Charts with 5-finger curves

#### 4. Data Collection
- Configurable frequency (1-1000 Hz)
- Configurable duration
- Data type selection
  - Motor status (position, speed, current)
  - Touch data (force, proximity)
- CSV export
- Real-time collection status

#### 5. Action Sequence
- Preset actions (Fist, Open, Pinch, Point, Wave)
- Custom action sequences
- Import/Export JSON
- Upload to device slots
- Run/Stop control

#### 6. Realtime Monitor
- Waveform charts (Position, Speed, Current, Touch)
- Hand visualization
- Configurable time window (5-60s)
- Configurable update rate (10-100Hz)
- Statistics (frequency, latency, packets, errors)

#### 7. DFU Firmware Upgrade (NEW)
- Firmware file selection
- Device type auto-detection
- Firmware compatibility check
- Progress display
- Upgrade log

#### 8. System Configuration
- Device info display
- Slave ID settings
- Device reboot
- Factory reset

## Device ID Reference

### CAN 2.0 Protocol
- Default IDs: 1 (left hand), 2 (right hand)
- Scan range: [1, 2]

### CANFD Protocol
- Default IDs: 0x7E/126 (left hand), 0x7F/127 (right hand)
- Scan range: [0x7E, 0x7F]

### SocketCAN (Linux)
- Uses same IDs as CAN 2.0 or CANFD depending on device
- Auto-detected via "Auto Detect" button
- Configure interface before use:
  ```bash
  sudo ip link set can0 type can bitrate 1000000
  sudo ip link set can0 up
  ```

### Modbus Protocol
- Default ID: 1
- Configurable via device settings

## Installation

### Dependencies

```bash
pip install PySide6 bc_stark_sdk
```

Or use requirements.txt:

```bash
pip install -r requirements.txt
```

## Usage

### Start GUI

```bash
python main.py
```

Or from parent directory:

```bash
python -m gui.main
```

### Basic Workflow

1. **Connect Device**
   - Select protocol (Modbus/RS485)
   - Configure connection parameters (port, baudrate, slave ID)
   - Click "Connect" button

2. **Control Motors**
   - Switch to "Motor Control" tab
   - Select control mode
   - Use sliders to control finger positions
   - Or use global control buttons

3. **View Touch Data**
   - Switch to "Touch Sensor" tab
   - Click "Enable Touch"
   - View real-time touch data

4. **Collect Data**
   - Switch to "Data Collection" tab
   - Set collection parameters (frequency, duration, save path)
   - Select data types to collect
   - Click "Start Collection"

5. **System Configuration**
   - Switch to "System Config" tab
   - View device info
   - Modify system parameters

## Development

### Project Structure

```
gui/
├── __init__.py                 # Package init
├── main.py                     # Main entry
├── main_window.py              # Main window
├── i18n.py                     # Internationalization
├── styles.py                   # UI styles
├── connection_panel.py         # Connection panel (auto-detect, CAN scan)
├── motor_control_panel.py      # Motor control panel
├── touch_sensor_panel.py       # Touch sensor panel (charts)
├── data_collector_panel.py     # Data collection panel
├── action_sequence_panel.py    # Action sequence panel
├── timing_test_panel.py        # Timing test panel
├── realtime_monitor_panel.py   # Realtime monitor panel (waveforms)
├── hand_visualization.py       # Hand visualization widget
├── dfu_panel.py                # DFU firmware upgrade panel (NEW)
├── system_config_panel.py      # System config panel
├── README.md                   # This file
└── requirements.txt            # Dependencies
```

### Adding New Languages

Edit `i18n.py` to add new translations:

```python
TRANSLATIONS_XX = {
    "app_title": "Your Translation",
    # ... add all keys
}
```

Then register in `I18n.__init__`:

```python
self._translations = {
    "en": TRANSLATIONS_EN,
    "zh": TRANSLATIONS_ZH,
    "xx": TRANSLATIONS_XX  # Add new language
}
```

### Extending Functionality

To add a new panel:

1. Create new panel class inheriting from `QWidget`
2. Implement `set_device(device, slave_id, device_info)` method
3. Implement `update_texts()` method for i18n support
4. Add tab in `main_window.py`

## Troubleshooting

### Cannot Connect Device

1. Check serial port permissions:
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```

2. Check if port is in use:
   ```bash
   lsof /dev/ttyUSB0
   ```

3. Verify bc_stark_sdk installation:
   ```bash
   python -c "import bc_stark_sdk; print(bc_stark_sdk.__version__)"
   ```

### GUI Won't Start

1. Check PySide6 installation:
   ```bash
   python -c "import PySide6; print(PySide6.__version__)"
   ```

2. Check Python version (requires 3.8+):
   ```bash
   python --version
   ```

## License

© 2026 BrainCo. All rights reserved.
