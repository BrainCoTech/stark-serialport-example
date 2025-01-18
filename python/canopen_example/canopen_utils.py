import sys
import pathlib
import time
import canopen

current_dir = pathlib.Path(__file__).resolve()
parent_dir = current_dir.parent.parent
sys.path.append(str(parent_dir))
sys.path.append(parent_dir.joinpath("canopen_example"))
from lib import *

timestamp_sync_ms = 0
rpdo_supported = False

# 定义每个 TPDO 的变量和配置
tpdo_configurations = [
    [('ActualPosition', i) for i in range(1, 5)],
    [('ActualPosition', i) for i in range(5, 7)],
    [
        ('ThumbForce.NormalForce', 1),
        ('ThumbForce.NormalForce', 2),
        ('ThumbForce.TangentialForce', 1),
        ('ThumbForce.TangentialForce', 2)
    ],
    [
        ('IndexForce.TangentDirection', 1),
        ('IndexForce.TangentDirection', 2)
    ],
    [
        ('IndexForce.NormalForce', 1),
        ('IndexForce.NormalForce', 2),
        ('IndexForce.NormalForce', 3),
        ('IndexForce.TangentialForce', 1)
    ],
    [
        ('IndexForce.TangentialForce', 2),
        ('IndexForce.TangentialForce', 3),
        ('IndexForce.TangentDirection', 1),
        ('IndexForce.TangentDirection', 2)
    ],
    [
        ('IndexForce.TangentDirection', 3)
    ],
    [
        ('MiddleForce.NormalForce', 1),
        ('MiddleForce.NormalForce', 2),
        ('MiddleForce.NormalForce', 3),
        ('MiddleForce.TangentialForce', 1)
    ],
    [
        ('MiddleForce.TangentialForce', 2),
        ('MiddleForce.TangentialForce', 3),
        ('MiddleForce.TangentDirection', 1),
        ('MiddleForce.TangentDirection', 2)
    ],
    [
        ('MiddleForce.TangentDirection', 3)
    ],
    [
        ('RingForce.NormalForce', 1),
        ('RingForce.NormalForce', 2),
        ('RingForce.NormalForce', 3),
        ('RingForce.TangentialForce', 1)
    ],
    [
        ('RingForce.TangentialForce', 2),
        ('RingForce.TangentialForce', 3),
        ('RingForce.TangentDirection', 1),
        ('RingForce.TangentDirection', 2)
    ],
    [
        ('RingForce.TangentDirection', 3)
    ],
    [
        ('PinkForce.NormalForce', 1),
        ('PinkForce.NormalForce', 2),
        ('PinkForce.TangentialForce', 1),
        ('PinkForce.TangentialForce', 2)
    ],
    [
        ('PinkForce.TangentDirection', 1),
        ('PinkForce.TangentDirection', 2)
    ],
    [
        ('SelfProximity.Thumb', 0),
        ('SelfProximity.Index1', 0)
    ],
    [
        ('SelfProximity.Index2', 0),
        ('SelfProximity.Middle1', 0)
    ],
    [
        ('SelfProximity.Middle2', 0),
        ('SelfProximity.Ring1', 0)
    ],
    [
        ('SelfProximity.Ring2', 0),
        ('SelfProximity.Pink', 0)
    ],
    [
        ('MutualProximity.Index', 0),
        ('MutualProximity.Middle', 0)
    ],
    [
        ('MutualProximity.Ring', 0)
    ]
]

def search_for_nodes(network: canopen.Network, limit: int = 0):
    # pass 
    # TBD：TPDO 数量超过 4 时, COB-ID如何设置, 可能与node-id发生冲突
    if limit > 0 and limit <= 127:
        SKLog.info(f"search for nodes, limit: {limit}")
        network.scanner.search(limit=limit)
        time.sleep(0.1)
        SKLog.critical(f"found node id: {network.scanner.nodes}")

def canopen_connect(interface: str = None, channel: int | str = None, bitrate: int = 0, search_limit: int = 0) -> canopen.Network:
    network = canopen.Network()

    # Connect to the CAN bus, currently only support bitrate=1000000
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

    if search_limit > 0:
        search_for_nodes(network, search_limit)

    return network

def add_node_to_network(network: canopen.Network, node_id: int) -> canopen.RemoteNode:
    node = canopen.RemoteNode(node_id, 'BC_hand_thumb_250118.eds')
    network.add_node(node)

    # Read Device Information, e.g. Vendor ID, Product Code, Revision Number, Serial Number
    # read_device_info(node)

    # Write a variable using SDO
    # node.sdo['Producer heartbeat time'].raw = 1000

    # Read PDO configuration from node
    # node.tpdo.read()
    # node.rpdo.read() 

    # 重新映射 TPDO
    for i, variables in enumerate(tpdo_configurations, start=1):
        if i > 2:
            break
        node.tpdo[i].clear()
        for var, index in variables:
            if i == 1 or i == 2:
                SKLog.info(f"TPDO[{i}] add_variable: {var} {index}")
                node.tpdo[i].add_variable(var, index)
            elif index == 0:
                node.tpdo[i].add_variable(var)
            else:    
                node.tpdo[i].add_variable(f'{var} {index}')
        node.tpdo[i].cob_id = 0x180 + node_id + (i - 1) # TBD: # confict with NodeId
        node.tpdo[i].cob_id = 0x180 + node_id + (i-1) * 0x100
        SKLog.info(f"TPDO[{i}] cob_id: {hex(node.tpdo[i].cob_id)}")
        node.tpdo[i].trans_type = 254  # 设置传输类型为周期性传输
        node.tpdo[i].event_timer = 50  # 设置事件计时器为50毫秒
        node.tpdo[i].enabled = True
        node.tpdo[i].save()  # 保存新的 PDO 配置到节点

    # Re-map RPDO
    if rpdo_supported:
        node.rpdo[1].clear()
        node.rpdo[1].add_variable('TargetPosition', 1)
        node.rpdo[1].add_variable('TargetPosition', 2)
        node.rpdo[1].add_variable('TargetPosition', 3)
        node.rpdo[1].add_variable('TargetPosition', 4)
        node.rpdo[1].cob_id = 0x200 + node_id
        node.rpdo[1].trans_type  = 0 # 设置传输类型为同步传输
        node.rpdo[1].enabled = True
        node.rpdo[1].save() # Save new PDO configuration to node

        node.rpdo[2].clear()
        node.rpdo[2].add_variable('TargetPosition', 5)
        node.rpdo[2].add_variable('TargetPosition', 6)
        node.rpdo[2].cob_id = 0x300 + node_id
        node.rpdo[2].trans_type = 0 # 设置传输类型为同步传输
        node.rpdo[2].enabled = True
        node.rpdo[2].save() # Save new PDO configuration to node

    # set timestamp_sync_ms to 0 to disable timestamp synchronization
    if timestamp_sync_ms > 0:
        network.sync.start(timestamp_sync_ms / 1000)

    # Change state to operational (NMT start)
    node.nmt.state = 'OPERATIONAL'

    return node

def close_canopen(network: canopen.Network):
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

    device_name = node.sdo['Manufacturer device name'].raw
    hardware_version = node.sdo['Manufacturer hardware version'].raw
    software_version = node.sdo['Manufacturer software version'].raw
    SKLog.info(f"device_name {device_name}, hardware_version: {hardware_version}, software_version: {software_version}")

# 读取节点信息
def get_node_cfg(node: canopen.RemoteNode) -> tuple[int, int]:
    node_id = node.sdo['NodeId'].raw
    baudrate = node.sdo['BaudRate'].raw
    SKLog.info(f"node_id: {node_id}, baudrate: {baudrate}")
    return node_id, baudrate

def get_node_id(node: canopen.RemoteNode) -> int:
    return node.sdo['NodeId'].raw

# TODO: TEST
# 设置节点ID, 1-127，重启后生效
def set_node_id(node: canopen.RemoteNode, node_id: int):
    if node_id < 1 or node_id > 127:
        SKLog.error("Invalid node_id")
        return
    node.sdo['NodeId'].raw = node_id

# TODO: TEST
# 设置波特率, 1000000, 500000, 250000，重启后生效
def set_baudrate(node: canopen.RemoteNode, baudrate: int):
    if baudrate not in [1000000, 500000, 250000]:
        SKLog.error("Invalid baudrate")
        return
    node.sdo['BaudRate'].raw = baudrate 

def __get_finger_position(node: canopen.RemoteNode, idx: int):
    tpdo_index = 1 if idx <= 4 else 2
    idx_mapping = ['Thumb', 'Thumb Aux', 'Index', 'Middle', 'Ring', 'Pink']
    actual_position = node.tpdo[tpdo_index][f'ActualPosition.{idx_mapping[idx-1]}'].raw
    return actual_position

# 读取单个手指的位置
def get_finger_position(node: canopen.RemoteNode, id: FingerId, wait_for_reception: bool = False):
    idx = id.value

    tpdo_index = 1 if idx <= 4 else 2
    if wait_for_reception:
        node.tpdo[tpdo_index].wait_for_reception()

    actual_position = __get_finger_position(node, idx)
    return actual_position

# 读取所有手指的位置
def get_finger_positions(node: canopen.RemoteNode, wait_for_reception: bool = False):
    if wait_for_reception:
        node.tpdo[1].wait_for_reception()
        node.tpdo[2].wait_for_reception()

    actual_positions = []
    for i in range(1, 7):
        actual_positions.append(__get_finger_position(node, i))

    return actual_positions

def __set_finger_position(node: canopen.RemoteNode, idx: int, position: int):
    key = 'TargetPosition' # .TargetPosition ' + str(idx)
    if idx <= 4:
        if rpdo_supported:
            node.rpdo[1][key][idx].raw = position
        else:
            node.sdo[key][idx].raw = position
    else:
        if rpdo_supported:
            node.rpdo[2][key][idx].raw = position
        else:
            node.sdo[key][idx].raw = position

# 设置单个手指的位置
def set_finger_position(node: canopen.RemoteNode, id: FingerId, position: int):
    idx = id.value
    __set_finger_position(node, idx, position)
        
# 设置所有手指的位置
def set_finger_positions(node: canopen.RemoteNode, positions: list[int]):
    for idx, position in enumerate(positions):
        __set_finger_position(node, idx + 1, position)

# 重置触觉传感器
def reset_touch_sensor(node: canopen.RemoteNode):        
    node.sdo['ForceSensorReset'].raw = 1

# 读取触觉数据
def get_touch_data(node: canopen.RemoteNode):
    # 传感器分布
    sensor_mapping = {
        'ThumbForce': 2,
        'IndexForce': 3,
        'MiddleForce': 3,
        'RingForce': 3,
        'PinkForce': 2
    }

    normal_forces = []
    tangential_forces = []
    target_directions = []

    # TODO: 读取触觉数据 via PDO
    for finger, count in sensor_mapping.items():
        for i in range(1, count + 1):
            normal_forces.append(node.sdo[f'{finger}.NormalForce {i}'].raw)
            tangential_forces.append(node.sdo[f'{finger}.TangentialForce {i}'].raw)
            target_directions.append(node.sdo[f'{finger}.TangentDirection {i}'].raw)

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
    required_positions = [55, 55, 99, 99, 99, 70] # 考虑到惯性及结构误差，这里的值可以根据实际情况调整

    for finger_position, required_position in zip(finger_positions, required_positions):
        if finger_position < required_position: return False
    return True