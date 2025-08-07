from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('tape_detector_ros2')
    
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'tape_detector_params.yaml'
    ])
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    scan_topic = LaunchConfiguration('scan_topic', default='/scan')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/bot_sensor/lidar_front/laser_scan',
            description='LaserScan topic name'
        ),
        
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to the parameter configuration file'
        ),
        
        Node(
            package='tape_detector_ros2',
            executable='tape_detector_node',
            name='tape_detector_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('/scan', scan_topic)
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])