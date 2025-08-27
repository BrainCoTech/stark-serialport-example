#!/bin/bash

# 不在当前目录时，切换到脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" # || exit 1

if [ "$1" == "launch" ]; then
    echo "=== Launch Stark Node ==="
fi    

# 设置环境变量
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu

# 清理 conda 相关变量
for var in $(env | grep -i conda | cut -d= -f1 2>/dev/null); do
    unset $var 2>/dev/null
done

# 设置 ROS 2 环境
source /opt/ros/humble/setup.bash

if [ "$3" == "clean" ]; then
    rm -rf build install log
fi    

# 是否编译, 根据传入的参数
if [ "$2" == "build" ]; then
    echo "Building workspace..."
    colcon build --packages-select ros2_stark_interfaces # 编译自定义消息包
    colcon build --packages-select ros2_stark_controller # 编译控制器节点 RS-485或CAN/CAN FD 版本
    if [ $? -ne 0 ]; then
        echo "Build failed. Exiting."
        exit 1
    fi
fi

# 设置工作空间
source install/setup.bash

# 运行
if [ "$1" == "launch" ]; then
    # 使用 launch 文件运行Stak节点
    # ros2 launch ros2_stark_controller stark_launch.py 
    # 使用配置文件运行stark_node
    # ros2 run ros2_stark_controller stark_node --ros-args --params-file ~/projects/stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/config/params_revo1.yaml 
    # ros2 run ros2_stark_controller stark_node --ros-args --params-file ~/projects/stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/config/params_revo1_touch.yaml
    # ros2 run ros2_stark_controller stark_node --ros-args --params-file ~/projects/stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/config/params_revo1_can.yaml
    ros2 run ros2_stark_controller stark_node --ros-args --params-file ~/projects/stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/config/params_revo2.yaml
    # ros2 run ros2_stark_controller stark_node --ros-args --params-file ~/projects/stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/config/params_revo2_canfd.yaml

elif [ "$1" == "monitor" ]; then
    # 测试监控关节信息
    ros2 topic echo /motor_status

elif [ "$1" == "monitor_touch" ]; then
    # 测试监控触觉信息
    ros2 topic echo /touch_status 

else
    # 运行测试脚本
    # 检查节点
    ros2 node list
    # 查看话题
    ros2 topic list
    # 查看服务
    ros2 service list
    # 测试位置控制
    # ros2 run ros2_stark_controller stark_node_client.py 1    # 一代手ID默认为1
    # ros2 run ros2_stark_controller stark_node_client.py 0x7e # 二代手左手ID默认为0x7e
    ros2 run ros2_stark_controller stark_node_client.py 0x7f # 二代手右手ID默认为0x7f
fi