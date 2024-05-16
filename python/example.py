#!/usr/bin/env python
import time
import sys
import glob
import serial
from stark_logger import SRLOG
from stark_sdk import *

if platform.system() == "Windows":
    serial_port_name = "COM3"  
elif sys.platform.startswith('darwin'):
    serial_port_name = "/dev/tty.usbserial-14220"
else:
    # serial_port_name = "/dev/ttyCH341USB0"
    serial_port_name = "/dev/ttyUSB0"    

SRLOG.LOG_INFO(f"Serial port name: {serial_port_name}")
StarkSDK.set_log_level(level=LogLevel.info)

_target_device: StarkDevice = None

device_baudrate=19200

# open serial port 8N1
port_set = None

def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def serial_open(port=None):
    port_status = True
    global port_set
    try:
        s = serial.Serial(port, baudrate=device_baudrate, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0.1)
        port_set = s
    except OSError:
        port_status = False
    return port_status

def serial_close():
    global port_set
    if port_set is not None:
        port_set.close()
        port_set = None

def serial_write_data(value, print_data=False):
    if print_data:
        print(f"serial_write_data: {len(value)} bytes")
        print(f"serial_write_data: {value.hex()}")
        print(f"serial_write_data: {','.join(['0x{:02X}'.format(byte) for byte in value])}")

    if port_set is None:
        print("port_set is None")
        return -1
    
    ret = port_set.write(value)
    port_set.flush()
    print(f"serial_write_data: {value}, ret: {ret}")
    return 0

def serial_read_value(print_data=False):
    if port_set is None:
        print("port_set is None")
        return -1
    
    # every 50ms check the data receive is ready
    byte_number_1 = 0
    byte_number_2 = 1
    while byte_number_1 != byte_number_2:
        byte_number_1 = port_set.in_waiting
        time.sleep(0.05)
        byte_number_2 = port_set.in_waiting
    receive_frame = port_set.read_all()

    if print_data:
        print(f"receive_frame, len: {len(receive_frame)}, data: {','.join(['0x{:02X}'.format(byte) for byte in receive_frame])}")
    _target_device.did_receive_data(receive_frame)
    return 0  

def on_error_response(error):
    SRLOG.LOG_ERROR(f"Error: {error.message}")

class DeviceListener(StarkDeviceListener):
    def on_error(_, error):
        SRLOG.LOG_ERROR(f"Error: {error.message}")

def on_device_error_response(device, error):
    SRLOG.LOG_ERROR(f"device: {device.name}, Error: {error.message}")

def on_serialport_cfg_response(serialport_cfg):
    SRLOG.LOG_INFO(f"Serialport cfg, baudrate: {serialport_cfg.baudrate}, serial_device_id: {serialport_cfg.serial_device_id}")

def on_hand_type_response(hand_type):
    SRLOG.LOG_INFO(f"Hand type: {hand_type}")   

def on_force_level_response(force_level):
    SRLOG.LOG_INFO(f"Force level: {force_level}")      

def on_motorboard_info_response(motorboard_info):
    SRLOG.LOG_INFO(f"Motorboard info, fw_version: {motorboard_info.fw_version}, sn: {motorboard_info.sn}, hand_type: {motorboard_info.hand_type}")

def on_limit_current_response(limit_current):
    SRLOG.LOG_INFO(f"Limit current: {limit_current}")

def on_voltage_response(voltage):
    SRLOG.LOG_INFO(f"Voltage: {voltage}")

def on_finger_status_response(finger_status):
    SRLOG.LOG_INFO(f"Finger status: {finger_status}")

def on_finger_movement_status_response(finger_movement_status):
    SRLOG.LOG_INFO(f"Finger movement status: {finger_movement_status}")

def on_button_event_response(button_event):
    SRLOG.LOG_INFO(f"Button event: {button_event}")    

if __name__ == '__main__':
    SRLOG.LOG_INFO(f"serial_ports: {serial_ports()}")
    SRLOG.LOG_INFO(f"serial_open: {serial_open(serial_port_name)}")
    StarkSDK.set_write_data_callback(serial_write_data)
    StarkSDK.set_error_callback(on_error_response)

    # broadcast
    StarkSDK.set_finger_speeds([10, 20, 30, 40, 50, 60])
    StarkSDK.set_finger_positions([10, 20, 30, 40, 50, 60])

    serial_device_id = 254 # broadcast id
    serial_device_id = 10 # 10 ~ 253
    device = StarkDevice.create_serial_device(serial_device_id, f"{serial_port_name}_{serial_device_id}")
    device.set_error_callback(on_device_error_response)
    device.set_listener(DeviceListener())
    _target_device = device

    # getters
    device.get_serialport_cfg(on_serialport_cfg_response)
    serial_read_value()
    device.get_hand_type(on_hand_type_response)
    serial_read_value()
    device.get_force_level(on_force_level_response)
    serial_read_value()
    device.get_motorboard_info(on_motorboard_info_response)
    serial_read_value()
    device.get_max_current(on_limit_current_response)
    serial_read_value()
    device.get_voltage(on_voltage_response)
    serial_read_value()
    device.get_finger_status(on_finger_status_response)
    serial_read_value()
    device.get_finger_movement_status(on_finger_movement_status_response)
    serial_read_value()
    device.get_button_event(on_button_event_response)
    serial_read_value()

    # setters
    device.factory_set_device_sn("stark-key", "stark-sn")
    device.factory_set_hand_type("stark-key", hand_type=HandType.medium_left)
    device.set_force_level(force_level=ForceLevel.full)
    # device.set_serial_device_id(1000) # invalid device_id
    device.set_serial_device_id(10)
    device.set_serialport_cfg(baudrate=device_baudrate)
    device.set_max_current(3000)
    device.reset_finger_positions()
    time.sleep(2)
    device.set_finger_position(50)
    time.sleep(2)
    device.set_finger_positions([10, 20, 30, 40, 50, 60])
    time.sleep(2)
    device.set_finger_positions([100, 100, 100, 100, 100, 100])
    time.sleep(2)
    device.set_finger_positions([0, 0, 0, 0, 0, 0])
    time.sleep(2)
    device.set_finger_speed(-50)
    time.sleep(2)
    device.set_finger_speeds([-10, 20, 30, 40, 50, 60])
    time.sleep(2)
    device.set_led_info(mode=LedMode.blink, color=LedColor.g)

    serial_close()
    exit(0)

