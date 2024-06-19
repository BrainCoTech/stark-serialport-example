import glob
import platform
import time
import serial
import sys
import pathlib

from functools import partial
current_dir = pathlib.Path(__file__).resolve()
parent_dir = current_dir.parent.parent
sys.path.append(str(parent_dir))
from lib import *

if platform.system() == "Windows":
    serial_port_name = "COM3"  
elif sys.platform.startswith('darwin'):
    serial_port_name = "/dev/tty.usbserial-14210"
else:
    serial_port_name = "/dev/ttyUSB0" 

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

def serial_open(portName: str, baudrate: int, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0.1):
    try:
        return serial.Serial(portName, baudrate, parity=parity, stopbits=stopbits, bytesize=bytesize, timeout=timeout)
    except OSError:
        pass
    return None

def serial_close(serial_port):
    if serial_port is not None:
        serial_port.close()

def serial_write_data(serial_port, value):
    SKLog.debug(f"serial_write_data: {len(value)} bytes, {hexlify_packets_without_0x(value)}")
    if (len(value) > 0 and len(value) < 30):
        SKLog.debug(hexlify_packets(value))

    if serial_port is None:
        SKLog.warning("serial_port is None")
        return -1
    
    ret = serial_port.write(value)
    if ret == 0:
        SKLog.warning("serial_write_data failed")
        return -1
    serial_port.flush()
    SKLog.debug(f"serial_write_data: {len(value)} bytes done")
    return 0

def _in_waiting(serial_port: serial.Serial):
    in_waiting = ("in_waiting" if hasattr(
        serial_port, "in_waiting") else "inWaiting")

    if in_waiting == "in_waiting":
        waitingbytes = getattr(serial_port, in_waiting)
    else:
        waitingbytes = getattr(serial_port, in_waiting)()
    return waitingbytes

def _wait_for_data(serial_port: serial.Serial, timeout: int=None):
        size = 0
        more_data = False
        if timeout is not None and timeout != 0:
            condition = partial(lambda start: (time.time() - start) <= timeout)
        else:
            condition = partial(lambda start: True)
        start = time.time()
        while condition(start):
            avaialble = _in_waiting(serial_port)
            # avaialble = serial_port.in_waiting
            if (more_data and not avaialble) or (more_data and avaialble == size):
                break
            if avaialble and avaialble != size:
                more_data = True
                size = avaialble
            time.sleep(0.01)
        return size

def serial_read_data(serial_port: serial.Serial, size=None, timeout=3):
    if serial_port is None:
        SKLog.warning("serial_port is None")
        return None
    
    if size is None:
        size = _wait_for_data(serial_port, timeout)

    data = serial_port.read(size)
    if len(data) != size:
        SKLog.warning(f"serial_read_data, expect size: {size} bytes, actually read: {len(data)} bytes")
    else:
        SKLog.debug(f"serial_read_data: {size} bytes")     
    
    if len(data) > 0 and len(data) < 100:
        SKLog.debug(hexlify_packets(data))
    return data      

