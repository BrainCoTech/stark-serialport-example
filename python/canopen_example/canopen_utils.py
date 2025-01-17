import sys
import pathlib
import time
import canopen
# from canlib import canlib

current_dir = pathlib.Path(__file__).resolve()
parent_dir = current_dir.parent.parent
sys.path.append(str(parent_dir))
sys.path.append(parent_dir.joinpath("canopen_example"))
from lib import *

# logging.basicConfig(level=logging.NOTSET)
logging.basicConfig(format='%(asctime)s %(levelname)s: %(message)s', level=logging.DEBUG)

timestamp_sync_ms = 0

def connect_to_canopen_node(node_id: int, interface: str = None, channel: int | str = None, bitrate: int = 0) -> tuple[canopen.Network, canopen.RemoteNode]:
    network = canopen.Network()

    # Connect to the CAN bus
    if interface is not None and channel is not None:
        network.connect(interface=interface, channel=channel, bitrate=bitrate)
        # network.connect(interface='socketcan', channel='can0')
        # network.connect(interface='kvaser', channel=0, bitrate=1000000)
        # network.connect(interface='pcan', channel='PCAN_USBBUS1', bitrate=250000)
        # network.connect(interface='ixxat', channel=0, bitrate=250000)
        # network.connect(interface='vector', app_name='CANalyzer', channel=0, bitrate=250000)
        # network.connect(interface='nican', channel='CAN0', bitrate=250000)
    else:
        network.connect()

    # node = canopen.LocalNode(node_id, 'BC_hand_250117.eds')
    node = canopen.RemoteNode(node_id, 'BC_hand_250117.eds')
    network.add_node(node)

    # network.scanner.search(limit=127)
    # print("Searching for nodes complete!")

    # # We may need to wait a short while here to allow all nodes to respond
    # time.sleep(0.1)
    # print(f"Found {len(network.scanner.nodes)} nodes!")
    # for node_id in network.scanner.nodes:
    #     print(f"Found node {node_id}!")    

    # Read a variable using SDO
    # read_device_info(node)

    # Write a variable using SDO
    # node.sdo['Producer heartbeat time'].raw = 1000

    # Read PDO configuration from node
    # node.tpdo.read()
    # node.rpdo.read() 

    # # Re-map TPDO[1]
    node.tpdo[1].clear()
    node.tpdo[1].add_variable('ActualPosition', 1)
    node.tpdo[1].add_variable('ActualPosition', 2)
    node.tpdo[1].add_variable('ActualPosition', 3)
    node.tpdo[1].add_variable('ActualPosition', 4)
    node.tpdo[1].trans_type = 254  # 设置传输类型为周期性传输
    node.tpdo[1].event_timer = 50  # 设置事件计时器为10毫秒
    node.tpdo[1].enabled = True
    node.tpdo[1].save() # Save new PDO configuration to node

    # Re-map TPDO[2]
    node.tpdo[2].clear()
    node.tpdo[2].add_variable('ActualPosition', 5)
    node.tpdo[2].add_variable('ActualPosition', 6)
    node.tpdo[2].trans_type = 254  # 设置传输类型为周期性传输
    node.tpdo[2].event_timer = 50  # 设置事件计时器为10毫秒
    node.tpdo[2].enabled = True
    node.tpdo[2].save() # Save new PDO configuration to node

    # # Re-map RPDO[1] currently not supported
    # node.rpdo[1].clear()
    # node.rpdo[1].add_variable('TargetPosition', 1)
    # node.rpdo[1].add_variable('TargetPosition', 2)
    # node.rpdo[1].add_variable('TargetPosition', 3)
    # node.rpdo[1].add_variable('TargetPosition', 4)
    # node.rpdo[1].trans_type  = 0 # 设置传输类型为同步传输
    # node.rpdo[1].enabled = True
    # node.rpdo[1].save() # Save new PDO configuration to node

    # # # Re-map RPDO[2]
    # node.rpdo[2].clear()
    # node.rpdo[2].add_variable('TargetPosition', 5)
    # node.rpdo[2].add_variable('TargetPosition', 6)
    # node.rpdo[2].trans_type = 0 # 设置传输类型为同步传输
    # node.rpdo[2].enabled = True
    # node.rpdo[2].save() # Save new PDO configuration to node

    # Transmit SYNC every timestamp_sync_ms milliseconds
    if timestamp_sync_ms > 0:
        network.sync.start(timestamp_sync_ms / 1000)

    # Change state to operational (NMT start)
    node.nmt.state = 'OPERATIONAL'

    return network, node

def close_canopen(network: canopen.Network):
    # Disconnect from CAN bus
    if timestamp_sync_ms > 0:
        network.sync.stop()
        
    network.disconnect()

# 读取设备信息
def read_device_info(node: canopen.RemoteNode):
    vendor_id = node.sdo[0x1018][1].raw
    product_code = node.sdo[0x1018][2].raw
    revision_number = node.sdo[0x1018][3].raw
    serial_number = node.sdo[0x1018][4].raw
    SKLog.info(f"vendor_id: {vendor_id}, product_code: {product_code}, revision_number: {revision_number}, serial_number: {serial_number}")

    # device_name = node.sdo['Manufacturer device name'].raw
    # hardware_version = node.sdo['Manufacturer hardware version'].raw
    # software_version = node.sdo['Manufacturer software version'].raw
    # SKLog.info(f"device_name {device_name}, hardware_version: {hardware_version}, software_version: {software_version}")

# 读取节点信息
def read_node_info(node: canopen.RemoteNode):
    node_id = node.sdo[0x1018][5].raw
    # bitrate = node.sdo[0x1018][6].raw
    SKLog.info(f"node_id: {node_id}")
    return node_id

# 设置节点ID, 1-127，重启后生效
def set_node_id(node: canopen.RemoteNode, node_id: int):
    node.sdo[0x1018][5].raw = node_id

# 读取单个手指的位置
def get_finger_position(node: canopen.RemoteNode, id: FingerId, wait_for_reception: bool = False):
    actual_position = 0

    idx = id.value
    if idx <= 4:
        if wait_for_reception:
            node.tpdo[1].wait_for_reception()
        actual_position = node.tpdo[1]['ActualPosition.ActualPosition ' + str(idx)].raw
    else:
        if wait_for_reception:
            node.tpdo[2].wait_for_reception()
        actual_position = node.tpdo[2]['ActualPosition.ActualPosition ' + str(idx)].raw
    return actual_position

# 读取所有手指的位置
def get_finger_positions(node: canopen.RemoteNode, wait_for_reception: bool = False):
    if wait_for_reception:
        node.tpdo[1].wait_for_reception()
        node.tpdo[2].wait_for_reception()

    actual_positions = [
        node.tpdo[1]['ActualPosition.ActualPosition 1'].raw,
        node.tpdo[1]['ActualPosition.ActualPosition 2'].raw,
        node.tpdo[1]['ActualPosition.ActualPosition 3'].raw,
        node.tpdo[1]['ActualPosition.ActualPosition 4'].raw,
        node.tpdo[2]['ActualPosition.ActualPosition 5'].raw,
        node.tpdo[2]['ActualPosition.ActualPosition 6'].raw
    ]
    return actual_positions

# 设置单个手指的位置
def set_finger_position(node: canopen.RemoteNode, id: FingerId, position: int):
    idx = id.value
    if idx <= 4:
        node.sdo['TargetPosition.TargetPosition ' + str(idx)].raw = position
        # node.rpdo[1]['TargetPosition.TargetPosition ' + str(idx)].raw = position
    else:
        node.sdo['TargetPosition.TargetPosition ' + str(idx)].raw = position
        # node.rpdo[2]['TargetPosition.TargetPosition ' + str(idx)].raw = position
        
# 设置所有手指的位置
def set_finger_positions(node: canopen.RemoteNode, positions: list[int]):
    for idx, position in enumerate(positions):
        if idx <= 4:
            node.sdo['TargetPosition.TargetPosition ' + str(idx+1)].raw = position
            # node.rpdo[1]['TargetPosition.TargetPosition ' + str(idx+1)].raw = position
        else:
            node.sdo['TargetPosition.TargetPosition ' + str(idx+1)].raw = position
            # node.rpdo[2]['TargetPosition.TargetPosition ' + str(idx+1)].raw = position   

# 读取触觉数据
def get_touch_data(node: canopen.RemoteNode):
    touch_sensor_num = 13
    normal_forces = []
    for i in range(0, touch_sensor_num):
        value = node.sdo['NormalForce'][i+1].raw
        normal_forces.append(value)

    tangential_forces = []
    for i in range(0, touch_sensor_num):
        value = node.sdo['TangentialForce'][i+1].raw
        tangential_forces.append(value)    

    target_directions = []
    for i in range(0, touch_sensor_num):
        value = node.sdo['TargetDirection'][i+1].raw
        target_directions.append(value) 

    from model import TouchStatusData
    touch_status = TouchStatusData(normal_forces, tangential_forces, target_directions)
    return touch_status    

def is_finger_opened(finger_positions):
    required_positions = [0] * 6
    # fmt: off
    for finger_position, required_position in zip(finger_positions, required_positions):
        if finger_position > required_position: return False
    return True

def is_finger_closed(finger_positions):
    # fmt: off
    required_positions = [55, 55, 99, 99, 99, 85] # 考虑到惯性及结构误差，这里的值可以根据实际情况调整

    for finger_position, required_position in zip(finger_positions, required_positions):
        if finger_position < required_position: return False
    return True