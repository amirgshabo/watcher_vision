#!/usr/bin/env python3
"""
ROS2 Launch File - Complete YOLO + ZED VSO Pipeline
===================================================

This launch file starts the complete pipeline:
1. ZED Depth Publisher - Publishes VSO data to ROS2 topics
2. YOLO Detector Node - Detects objects with 3D localization
3. YOLO Visualizer Node - Displays results

Usage:
    ros2 launch depth_vso_testing yolo_vso_pipeline.launch.py vso_file:=/path/to/file.vso

Advanced Usage:
    ros2 launch depth_vso_testing yolo_vso_pipeline.launch.py \
        vso_file:=/path/to/file.vso \
        model_path:=yolov8n.onnx \
        confidence_threshold:=0.5 \
        publish_rate:=10.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for YOLO + ZED VSO pipeline"""
    
    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================
    
    # VSO File
    vso_file_arg = DeclareLaunchArgument(
        'vso_file',
        default_value='',
        description='Path to VSO file to process (required)'
    )
    
    # YOLO Model
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.onnx',
        description='Path to YOLO ONNX model file'
    )
    
    # Detection Parameters
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Minimum confidence threshold for detections (0.0-1.0)'
    )
    
    # Publishing Rate
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publishing rate in Hz (frames per second)'
    )
    
    # Processing Rate
    processing_rate_arg = DeclareLaunchArgument(
        'processing_rate',
        default_value='10.0',
        description='YOLO processing rate in Hz'
    )
    
    # VSO Playback Options
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
    
    # Point Cloud vs Depth Image
    use_point_cloud_arg = DeclareLaunchArgument(
        'use_point_cloud',
        default_value='true',
        description='Use point cloud for 3D (true) or depth image (false)'
    )
    
    # Visualization Options
    save_output_arg = DeclareLaunchArgument(
        'save_output',
        default_value='false',
        description='Save visualization output to video file'
    )
    
    output_path_arg = DeclareLaunchArgument(
        'output_path',
        default_value='/tmp/yolo_detections.avi',
        description='Path to save output video'
    )
    
    # ========================================================================
    # NODES
    # ========================================================================
    
    # Node 1: ZED Depth Publisher
    # Publishes VSO data (RGB, depth, point cloud) to ROS2 topics
    zed_publisher = Node(
        package='depth_vso_testing',
        executable='zed_depth_publisher.py',
        name='zed_depth_publisher',
        parameters=[{
            'vso_file': LaunchConfiguration('vso_file'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'loop_playback': LaunchConfiguration('loop_playback'),
            'start_frame': LaunchConfiguration('start_frame')
        }],
        output='screen',
        emulate_tty=True,
        prefix='xterm -e'  # Run in separate terminal for easy monitoring
    )
    
    # Node 2: YOLO Detector
    # Subscribes to ZED topics, runs YOLO, publishes detections
    yolo_detector = Node(
        package='depth_vso_testing',
        executable='yolo_detector_node.py',
        name='yolo_detector_node',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'use_point_cloud': LaunchConfiguration('use_point_cloud'),
            'publish_debug_image': True,
            'processing_rate': LaunchConfiguration('processing_rate')
        }],
        output='screen',
        emulate_tty=True,
        prefix='xterm -e'  # Run in separate terminal
    )
    
    # Node 3: YOLO Visualizer
    # Subscribes to detections and displays results
    yolo_visualizer = Node(
        package='depth_vso_testing',
        executable='yolo_visualizer_node.py',
        name='yolo_visualizer_node',
        parameters=[{
            'window_name': 'YOLO Detections - ZED VSO',
            'use_debug_image': True,
            'display_stats': True,
            'save_output': LaunchConfiguration('save_output'),
            'output_path': LaunchConfiguration('output_path')
        }],
        output='screen',
        emulate_tty=True
    )
    
    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    
    return LaunchDescription([
        # Arguments
        vso_file_arg,
        model_path_arg,
        confidence_threshold_arg,
        publish_rate_arg,
        processing_rate_arg,
        loop_playback_arg,
        start_frame_arg,
        use_point_cloud_arg,
        save_output_arg,
        output_path_arg,
        
        # Nodes
        zed_publisher,
        yolo_detector,
        yolo_visualizer
    ])
