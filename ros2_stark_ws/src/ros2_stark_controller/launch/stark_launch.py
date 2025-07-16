from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # TODO: 更新以下为实际配置
        # 一代灵巧手触觉版
        Node(
            package='ros2_stark_controller',
            executable='stark_node',
            output='screen',
            parameters=['/home/nvidia/projects/stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/config/params_v1_touch.yaml'],  # 直接传递路径
        ),
        # 二代灵巧手
        # Node(
        #     package='ros2_stark_controller',
        #     executable='stark_node',
        #     output='screen',
        #     parameters=['/home/nvidia/projects/stark-serialport-example/ros2_stark_ws/src/ros2_stark_controller/config/params_v2.yaml'],  # 直接传递路径
        # ),
    ])
