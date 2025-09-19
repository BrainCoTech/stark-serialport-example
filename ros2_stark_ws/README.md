# ros2 humble stark controller

## RS-485 或 CAN 版本

### 下载依赖

```shell
# First download libs, in project root directory
rm VERSION
./download-lib.sh

# 下载完成后目录包含如下
ros2_stark_ws/src/ros2_stark_controller/lib
└── libbc_stark_sdk.so

ros2_stark_ws/src/ros2_stark_controller/include
└── ros2_stark_controller
    ├── stark-sdk.h
    └── stark_node.hpp
```

### 启动控制器节点

```shell
cd ros2_stark_ws
chmod +x stark_serial_manager.sh # 给脚本执行权限
./stark_serial_manager.sh launch # 启动控制器节点
# ./stark_serial_manager.sh launch build # 如果需要编译
# ./stark_serial_manager.sh launch build clean # 如果需要编译并清理
```

### 重启开启另一个终端

```shell
cd ros2_stark_ws
./stark_serial_manager.sh # 测试发送位置控制命令
./stark_serial_manager.sh monitor  # 监控电机状态
./stark_serial_manager.sh monitor_touch  # 监控触觉状态
ros2 topic list # 查看话题列表
ros2 service list # 查看服务列表
ros2 param list # 查看参数列表
ros2 param get /stark_node port # 查看串口参数
ros2 topic info /set_motor_single_1 # 查看单个手指控制话题信息
# 发送单个手指位置控制命令
ros2 topic pub --once /set_motor_single_1 ros2_stark_msgs/msg/SetMotorSingle "{
  slave_id: 1,
  mode: 1,
  motor_id: 5,
  position: 100,
  speed: 0,
  current: 0,
  pwm: 0,
  duration: 0
}"
```

## ROS2 EtherCAT 版本

### 启动控制器节点 - EtherCAT 版本

```shell
ip link show # 检查网络接口
ethercat version # 检查 EtherCAT 版本
sudo systemctl enable ethercat # 启用 EtherCAT 主站
sudo systemctl restart ethercat # 重启 EtherCAT 主站
sudo systemctl status ethercat # 检查 EtherCAT 主站是否运行
sudo ethercat slaves  # 检查是否有 EtherCAT 设备连接
sudo ethercat sdos # 检查 EtherCAT 设备的 SDO 信息
sudo ethercat pdos # 检查 EtherCAT 设备的 PDO 信息

# 读取固件版本
ethercat upload -t string -p 0 0x8000 0x11 # FW version
ethercat upload -t string -p 0 0x8000 0x12 # SN
ethercat upload -t string -p 0 0x8000 0x13 # Wrist FW version

# 在OP模式下，读取关节信息
# echo "=== 读取所有关节数据 ==="
# echo "位置数据 (6x UINT16):"
ethercat upload -t raw -p 0 0x6000 0x01
ethercat upload -t raw -p 0 0x6000 0x01 | xxd -r -p | od -An -t u2 --endian=little -w2 | awk '{printf "关节位置%d: %d\n", NR, $1}'

# 在OP模式下，读取触觉信息
ethercat upload -t raw -p 0 0x6010 0x01
ethercat upload -t raw -p 0 0x6010 0x01 | xxd -r -p | od -An -t u2 --endian=little -w2 | awk '{printf "法向力%d: %d\n", NR, $1}'

cd ros2_stark_ws
chmod +x stark_ethercat_manager.sh # 给脚本执行权限
./stark_ethercat_manager.sh launch # 启动控制器节点
#./stark_ethercat_manager.sh launch build # 如果需要编译
#./stark_ethercat_manager.sh launch build clean # 如果需要编译并清理
```

### 重启开启另一个终端 - EtherCAT 版本

```shell
cd ros2_stark_ws
./stark_ethercat_manager.sh # 测试发送位置控制命令
./stark_ethercat_manager.sh monitor  # 监控左电机状态
./stark_ethercat_manager.sh monitor_right  # 监控右电机状态
```
