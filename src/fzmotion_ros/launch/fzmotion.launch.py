"""Launch the installed FZMotion bridge on ROS 2 Jazzy."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('fzmotion_ros'), 'config', 'fzmotion.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_config),
        Node(package='fzmotion_ros', executable='fzmotion_node',
             parameters=[LaunchConfiguration('params_file')], output='screen'),
    ])
