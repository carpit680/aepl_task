from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("vrx_gz"), '/launch', '/competition.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("vrx_gz"), '/launch', '/usv_joy_teleop.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("vrx_gazebo"), '/launch', '/rviz.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("slam_toolbox"), '/launch', '/online_async_launch.py'])
        ),
        Node(
            package='aepl_pkg',  # Replace with your package name
            executable='vrx_odom',  # Replace with your node executable
            name='vrx_odom',
            output='screen',
        ),
        Node(
            package='aepl_pkg',  # Replace with your package name
            executable='vrx_controller',  # Replace with your node executable
            name='vrx_controller',
            output='screen',
        ),
    ])
