# ros2 humble

```shell
# First download libs
rm VERSION
./download-lib.sh

# 下载完成后目录包含如下
ros2_stark_ws/src/ros2_stark_controller/lib
└── libbc_stark_sdk.so

ros2_stark_ws/src/ros2_stark_controller/include
└── ros2_stark_controller
    ├── stark-sdk.h
    └── stark_node.hpp

# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
# source /opt/ros/humble/setup.zsh

cd ros2_stark_ws
rm -rf build install log

# 编译
colcon build --symlink-install
# colcon build --packages-select ros2_stark_interfaces 
# colcon build --packages-select ros2_stark_controller

# 设置本地环境
source install/setup.bash
# source install/setup.zsh

# FIXME: 需要设置动态链接库路径
# export LD_LIBRARY_PATH=/home/hailong/projects/stark-serialport-example/ros2_stark_ws/install/ros2_stark_controller/lib/ros2_stark_controller:$LD_LIBRARY_PATH

# 使用 launch 文件运行Stak节点
ros2 launch ros2_stark_controller stark_launch.py 

# 测试位置控制
ros2 run ros2_stark_controller stark_position_control.py --positions 0.5 0.5 0.5 0.5 0.5 0.5
ros2 run ros2_stark_controller stark_position_control.py --positions 0.0 1.0 0.0 1.0 0.0 1.0 --rate 10

# services
ros2 interface show ros2_stark_interfaces/srv/SetMotorPositions

# 检查节点
ros2 node list
# 查看话题
ros2 topic list

# 
# ros2 topic echo /joint_states
ros2 topic echo /motor_status
ros2 topic echo /touch_status

# 查看参数
ros2 param list /stark_node
# 动态修改参数
ros2 param set /stark_node baudrate 115200
ros2 param set /stark_node baudrate 57600

```
