#!/usr/bin/env python3
"""
ARCS WATCHER - Vision System Launch File
Launches the ZED camera wrapper and YOLO detector node together.
Vision Team Lead: Amir Shabo
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --- Launch Arguments ---
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (set true for rosbag playback)'
    )

    input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='/zed/zed_node/rgb/image_rect_color',
        description='RGB image topic to subscribe to'
    )

    depth_topic = DeclareLaunchArgument(
        'depth_topic',
        default_value='/zed/zed_node/depth/depth_registered',
        description='Depth topic from ZED camera'
    )

    confidence_threshold = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.45',
        description='YOLO detection confidence threshold'
    )

    model_path = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.pt',
        description='Path to YOLO model weights'
    )

    # --- YOLO Detector Node ---
    yolo_detector_node = Node(
        package='arcs_yolo_detector',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('arcs_yolo_detector'),
                'config',
                'detector_params.yaml'
            ]),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': LaunchConfiguration('input_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'model_path': LaunchConfiguration('model_path'),
            }
        ]
    )

    return LaunchDescription([
        use_sim_time,
        input_topic,
        depth_topic,
        confidence_threshold,
        model_path,
        yolo_detector_node,
    ])