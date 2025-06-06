# ros2 humble or galactic

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

# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
# Ubuntu 22.04
source /opt/ros/humble/setup.bash # bash
source /opt/ros/humble/setup.zsh # zsh

# Ubuntu 20.04
source /opt/ros/galactic/setup.bash # bash
source /opt/ros/galactic/setup.zsh # zsh

cd ros2_stark_ws
rm -rf build install log

# 编译
colcon build --symlink-install
# colcon build --packages-select ros2_stark_interfaces 
# colcon build --packages-select ros2_stark_controller

# pip install catkin_pkg numpy lark empy==3.3.4
# export PYTHON_EXECUTABLE=/home/nvidia/miniconda3/envs/py310/bin/python
# colcon build --symlink-install --cmake-args -DPYTHON_EXECUTABLE=/home/nvidia/miniconda3/envs/py310/bin/python

# 设置本地环境
source install/setup.bash # bash
source install/setup.zsh # zsh

# FIXME: 需要设置动态链接库路径, 更新为实际路径
export LD_LIBRARY_PATH=/home/nvidia/projects/stark-serialport-example/ros2_stark_ws/install/ros2_stark_controller/lib/ros2_stark_controller:$LD_LIBRARY_PATH

# 使用 launch 文件运行Stak节点
ros2 launch ros2_stark_controller stark_launch.py 
# or
ros2 run ros2_stark_controller stark_node --ros-args --params-file /home/nvidia/projects/stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/config/params_v2.yaml

# 检查节点
ros2 node list
# 查看话题
ros2 topic list
# 查看服务
ros2 service list

# 测试位置控制
ros2 run ros2_stark_controller stark_node_client.py 1 # 使用从机ID (1)
ros2 run ros2_stark_controller stark_node_client.py 2 # 使用从机ID (2)

# ros2 topic echo /joint_states
ros2 topic echo /motor_status
ros2 topic echo /touch_status

# 查看参数
ros2 param list /stark_node

# 动态修改参数
ros2 param set /stark_node baudrate 115200
ros2 param set /stark_node baudrate 57600
```
