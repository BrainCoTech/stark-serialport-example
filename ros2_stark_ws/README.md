# ros2 humble

```shell
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash

cd ros2_stark_ws
rm -rf build install log

# 编译
colcon build --packages-select --allow-overriding ros2_stark_interfaces 
colcon build --packages-select ros2_stark_controller

# 设置本地环境
source install/setup.bash

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
