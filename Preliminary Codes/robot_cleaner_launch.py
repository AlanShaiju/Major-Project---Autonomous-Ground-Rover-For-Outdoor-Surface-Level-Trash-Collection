# robot_cleaner_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_cleaner', executable='boundary_mapper', output='screen'),
        Node(package='robot_cleaner', executable='localization', output='screen'),
        Node(package='robot_cleaner', executable='sweep_cleaner', output='screen'),
        Node(package='robot_cleaner', executable='obstacle_avoidance', output='screen'),
        Node(package='robot_cleaner', executable='waste_management', output='screen'),
        Node(package='robot_cleaner', executable='charging_management', output='screen'),
        Node(package='robot_cleaner', executable='master_controller', output='screen'),])