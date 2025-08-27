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
chmod +x launch_test.sh # 给脚本执行权限
./launch_test.sh launch # 启动控制器节点
# ./launch_test.sh launch build # 如果需要编译
# ./launch_test.sh launch build clean # 如果需要编译并清理
```

### 重启开启另一个终端

```shell
cd ros2_stark_ws
./launch_test.sh # 测试发送位置控制命令
./launch_test.sh monitor  # 监控电机状态
./launch_test.sh monitor_touch  # 监控触觉状态
```

## ROS2 EtherCAT 版本

### 启动控制器节点 - EtherCAT 版本

```shell
sudo systemctl status ethercat # 检查 EtherCAT 主站是否运行
ip link show # 检查网络接口
sudo ethercat slaves  # 检查是否有 EtherCAT 设备连接
chmod +x launch_ethercat.sh # 给脚本执行权限

cd ros2_stark_ws
./launch_ethercat.sh launch # 启动控制器节点
#./launch_ethercat.sh launch build # 如果需要编译
#./launch_ethercat.sh launch build clean # 如果需要编译并清理
```

### 重启开启另一个终端 - EtherCAT 版本

```shell
cd ros2_stark_ws
./launch_ethercat.sh # 测试发送位置控制命令
./launch_ethercat.sh monitor  # 监控左电机状态
./launch_ethercat.sh monitor_right  # 监控右电机状态
```
