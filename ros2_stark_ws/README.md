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
# export PYTHON_EXECUTABLE=~/miniconda3/envs/py310/bin/python
# colcon build --symlink-install --cmake-args -DPYTHON_EXECUTABLE=~/miniconda3/envs/py310/bin/python

# 设置本地环境
source install/setup.bash # bash
source install/setup.zsh # zsh

# FIXME: 需要设置动态链接库路径, 更新为实际路径
export LD_LIBRARY_PATH=~/projects/stark-serialport-example/ros2_stark_ws/install/ros2_stark_controller/lib/ros2_stark_controller:$LD_LIBRARY_PATH

# Ubuntu 22.04 humble + zsh
source /opt/ros/humble/setup.zsh
colcon build --symlink-install
source install/setup.zsh

# 使用 launch 文件运行Stak节点
ros2 launch ros2_stark_controller stark_launch.py 

# 使用配置文件运行stark_node
ros2 run ros2_stark_controller stark_node --ros-args --params-file ~/projects/stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/config/params_revo1.yaml 
ros2 run ros2_stark_controller stark_node --ros-args --params-file ~/projects/stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/config/params_revo1_touch.yaml
ros2 run ros2_stark_controller stark_node --ros-args --params-file ~/projects/stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/config/params_revo1_can.yaml

ros2 run ros2_stark_controller stark_node --ros-args --params-file ~/projects/stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/config/params_revo2.yaml
ros2 run ros2_stark_controller stark_node --ros-args --params-file ~/projects/stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/config/params_revo2_canfd.yaml

# 检查节点
ros2 node list
# 查看话题
ros2 topic list
# 查看服务
ros2 service list

# 测试位置控制
ros2 run ros2_stark_controller stark_node_client.py 1    # 一代手ID默认为1
ros2 run ros2_stark_controller stark_node_client.py 0x7e # 二代手左手ID默认为0x7e
ros2 run ros2_stark_controller stark_node_client.py 0x7f # 二代手右手ID默认为0x7f

# ros2 topic echo /joint_states
ros2 topic echo /motor_status
ros2 topic echo /touch_status

# 查看参数
ros2 param list /stark_node

# 动态修改参数
ros2 param set /stark_node baudrate 115200
ros2 param set /stark_node baudrate 57600
```
