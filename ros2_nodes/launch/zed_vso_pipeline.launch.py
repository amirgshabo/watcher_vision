#!/usr/bin/env python3
"""
ROS2 Launch File - ZED VSO Processing Pipeline
=============================================

This launch file starts both the publisher and subscriber nodes
to create a complete ROS2 pipeline for ZED depth processing.

Usage:
ros2 launch zed_depth_processing zed_vso_pipeline.launch.py vso_file:=/path/to/file.vso
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    vso_file_arg = DeclareLaunchArgument(
        'vso_file',
        default_value='',
        description='Path to VSO file to process'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publishing rate in Hz'
    )
    
    loop_playback_arg = DeclareLaunchArgument(
        'loop_playback',
        default_value='true',
        description='Loop VSO playback when end is reached'
    )
    
    start_frame_arg = DeclareLaunchArgument(
        'start_frame',
        default_value='0',
        description='Frame number to start playback from'
    )
    
    # ZED Depth Publisher Node
    zed_publisher = Node(
        package='zed_depth_ros2',
        executable='zed_depth_publisher.py',
        name='zed_depth_publisher',
        parameters=[{
            'vso_file': LaunchConfiguration('vso_file'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'loop_playback': LaunchConfiguration('loop_playback'),
            'start_frame': LaunchConfiguration('start_frame')
        }],
        output='screen',
        emulate_tty=True
    )
    
    # ZED Depth Subscriber Node (Your ROS2 analysis patterns)
    zed_subscriber = Node(
        package='zed_depth_ros2', 
        executable='zed_depth_subscriber.py',
        name='zed_depth_subscriber',
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        vso_file_arg,
        publish_rate_arg,
        loop_playback_arg,
        start_frame_arg,
        zed_publisher,
        zed_subscriber
    ])