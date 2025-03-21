from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='~/projects/ros2_stark_ws/src/ros2_stark_controller/config/params.yaml',
            description='Path to the ROS2 parameters file'
        ),
        
        Node(
            package='ros2_stark_controller',
            executable='stark_node',
            name='stark_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        ),
    ])