from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='path/to/ros2_stark_ws/src/ros2_stark_controller/config/params.yaml', # Update with the correct path
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