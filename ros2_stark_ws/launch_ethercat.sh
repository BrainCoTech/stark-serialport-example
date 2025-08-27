#!/bin/bash

# 不在当前目录时，切换到脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" # || exit 1

if [ "$1" == "launch" ]; then
    echo "=== Running EtherCAT controller ==="
fi    

# 设置环境变量
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

# 清理 conda 相关变量
for var in $(env | grep -i conda | cut -d= -f1 2>/dev/null); do
    unset $var 2>/dev/null
done

# 设置 ROS 2 环境
echo "Setting up ROS 2 environment..."
source /opt/ros/humble/setup.bash

# 添加额外的库路径，而不是覆盖
# export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"

# 验证ROS环境
echo "ROS_DISTRO: $ROS_DISTRO"

if [ "$3" == "clean" ]; then
    rm -rf build install log
fi    

# 是否编译, 根据传入的参数
if [ "$2" == "build" ]; then
    echo "Building workspace..."
    colcon build --packages-select ros2_stark_interfaces # 编译自定义消息包
    colcon build --packages-select ros2_ethercat_controller # 编译EtherCAT版本
    if [ $? -ne 0 ]; then
        echo "Build failed. Exiting."
        exit 1
    fi
fi

# 设置工作空间
source install/setup.bash

# 验证动态库依赖
# EC_NODE_PATH="install/ros2_ethercat_controller/lib/ros2_ethercat_controller/stark_ec_node"
# if [ -f "$EC_NODE_PATH" ]; then
#     echo "Checking library dependencies..."
#     MISSING_LIBS=$(ldd "$EC_NODE_PATH" | grep "not found")
#     if [ -n "$MISSING_LIBS" ]; then
#         echo "Error: Missing library dependencies:"
#         echo "$MISSING_LIBS"
#         echo "Current LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
#         exit 1
#     fi
#     echo "All libraries found successfully!"
# else
#     echo "Error: stark_ec_node not found. Please run: $0 test build"
#     exit 1
# fi

# 运行
if [ "$1" == "launch" ]; then
    echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
    # sudo setcap cap_sys_nice,cap_net_raw=eip install/ros2_ethercat_controller/lib/ros2_ethercat_controller/stark_ec_node
    # getcap install/ros2_ethercat_controller/lib/ros2_ethercat_controller/stark_ec_node
    # ros2 launch ros2_ethercat_controller ec_launch.py
    ros2 run ros2_ethercat_controller stark_ec_node --ros-args --params-file /home/yongle/projects/stark-serialport-example/ros2_stark_ws/src/ros2_ethercat_controller/config/params_revo2_ethercat.yaml

elif [ "$1" == "monitor" ]; then
    # 测试监控关节信息
    ros2 topic echo /motor_status_left

elif [ "$1" == "monitor_right" ]; then
    # 测试监控关节信息
    ros2 topic echo /motor_status_right    

else
    # 运行测试脚本
    echo "Available topics:"
    ros2 topic list
    echo ""
    echo "Available services:"
    ros2 service list
    echo ""
    echo "Testing EtherCAT controller..."
    ros2 run ros2_ethercat_controller stark_ec_client.py 0    # 从站pos=0
    # ros2 run ros2_ethercat_controller stark_ec_client.py 1    # 从站pos=1
fi