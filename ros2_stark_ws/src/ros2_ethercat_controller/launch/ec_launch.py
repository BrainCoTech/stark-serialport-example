from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 二代灵巧手-EtherCAT版
        Node(
            package='ros2_ethercat_controller',
            executable='stark_ec_node',
            output='screen',
            parameters=['/home/yongle/projects/stark-serialport-example/ros2_stark_ws/src/ros2_ethercat_controller/config/params_revo2_ethercat.yaml'],  # 直接传递路径
        ),
    ])
