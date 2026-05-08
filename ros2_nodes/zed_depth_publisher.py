#!/usr/bin/env python3
"""
ZED Depth Publisher Node - Publishes depth data from VSO files to ROS2 topics
=============================================================================

This node reads VSO files and publishes the depth data to ROS2 topics,
allowing other nodes to subscribe and process the data.

Topics Published:
- /zed/depth/image_raw (sensor_msgs/Image) - Raw depth data
- /zed/rgb/image_raw (sensor_msgs/Image) - RGB image
- /zed/point_cloud (sensor_msgs/PointCloud2) - 3D point cloud
- /zed/depth/camera_info (sensor_msgs/CameraInfo) - Camera parameters
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import pyzed.sl as sl
import cv2
import numpy as np
import math
from threading import Thread
import time

class ZEDDepthPublisher(Node):
    """ROS2 Node that publishes depth data from ZED VSO files"""
    
    def __init__(self):
        super().__init__('zed_depth_publisher')
        
        # Parameters
        self.declare_parameter('vso_file', '')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('loop_playback', True)
        self.declare_parameter('start_frame', 0)
        
        # Load parameters
        self.vso_file = self.get_parameter('vso_file').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.loop_playback = self.get_parameter('loop_playback').get_parameter_value().bool_value
        self.start_frame = self.get_parameter('start_frame').get_parameter_value().integer_value
        
        if not self.vso_file:
            self.get_logger().error('VSO file parameter is required!')
            raise ValueError('vso_file parameter must be specified')
        
        # Initialize CV Bridge
        self.cv_bridge = CvBridge()
        
        # Publishers
        self.depth_pub = self.create_publisher(Image, '/zed/depth/image_raw', 10)
        self.rgb_pub = self.create_publisher(Image, '/zed/rgb/image_raw', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/zed/point_cloud', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/zed/depth/camera_info', 10)
        
        # Initialize ZED Camera
        self.zed = sl.Camera()
        self.init_camera()
        
        # Publishing timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_frame)
        
        # Frame tracking
        self.current_frame = 0
        self.total_frames = self.zed.get_svo_number_of_frames()
        
        # Seek to start frame if specified
        if self.start_frame > 0:
            self.zed.set_svo_position(self.start_frame)
            self.current_frame = self.start_frame
        
        self.get_logger().info(f'ZED Depth Publisher initialized')
        self.get_logger().info(f'VSO File: {self.vso_file}')
        self.get_logger().info(f'Total frames: {self.total_frames}')
        self.get_logger().info(f'Publishing at {self.publish_rate} Hz')
        self.get_logger().info(f'Loop playback: {self.loop_playback}')
    
    def init_camera(self):
        """Initialize ZED from VSO file"""
        init_params = sl.InitParameters()
        init_params.set_from_svo_file(self.vso_file)
        init_params.sdk_verbose = False
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Failed to open ZED camera: {err}')
            raise RuntimeError(f'ZED initialization failed: {err}')
        
        self.get_logger().info('ZED camera initialized successfully')
    
    def publish_frame(self):
        """Publish current frame data to ROS2 topics"""
        # ZED matrices
        image = sl.Mat()
        depth = sl.Mat()
        point_cloud = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        
        # Grab frame
        grab_status = self.zed.grab(runtime_parameters)
        
        if grab_status == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            if self.loop_playback:
                self.get_logger().info('End of VSO file reached, looping...')
                self.zed.set_svo_position(0)
                self.current_frame = 0
                return
            else:
                self.get_logger().info('End of VSO file reached, stopping...')
                rclpy.shutdown()
                return
        
        if grab_status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().warn(f'Failed to grab frame {self.current_frame}: {grab_status}')
            return
        
        # Retrieve data
        self.zed.retrieve_image(image, sl.VIEW.LEFT)
        self.zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        self.zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
        
        # Create timestamp
        timestamp = self.get_clock().now().to_msg()
        
        # Publish RGB image
        self.publish_rgb_image(image, timestamp)
        
        # Publish depth image
        self.publish_depth_image(depth, timestamp)
        
        # Publish point cloud (every 5th frame to reduce bandwidth)
        if self.current_frame % 5 == 0:
            self.publish_point_cloud(point_cloud, timestamp)
        
        # Publish camera info
        self.publish_camera_info(timestamp)
        
        # Log progress
        if self.current_frame % 50 == 0:
            progress = (self.current_frame / self.total_frames) * 100
            self.get_logger().info(f'Publishing frame {self.current_frame}/{self.total_frames} ({progress:.1f}%)')
        
        self.current_frame += 1
    
    def publish_rgb_image(self, image_mat, timestamp):
        """Convert and publish RGB image"""
        try:
            # Convert ZED Mat to numpy
            image_np = image_mat.get_data()
            
            # Convert BGRA to BGR (remove alpha channel)
            if image_np.shape[2] == 4:
                image_np = image_np[:, :, :3]
            
            # Convert to ROS2 Image message
            ros_image = self.cv_bridge.cv2_to_imgmsg(image_np, encoding='bgr8')
            ros_image.header.stamp = timestamp
            ros_image.header.frame_id = 'zed_left_camera_frame'
            
            self.rgb_pub.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish RGB image: {e}')
    
    def publish_depth_image(self, depth_mat, timestamp):
        """Convert and publish depth image"""
        try:
            # Convert ZED Mat to numpy (already in meters)
            depth_np = depth_mat.get_data()
            
            # Convert to 32-bit float for ROS2 standard
            depth_32f = depth_np.astype(np.float32)
            
            # Convert to ROS2 Image message
            ros_depth = self.cv_bridge.cv2_to_imgmsg(depth_32f, encoding='32FC1')
            ros_depth.header.stamp = timestamp
            ros_depth.header.frame_id = 'zed_left_camera_frame'
            
            self.depth_pub.publish(ros_depth)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish depth image: {e}')
    
    def publish_point_cloud(self, pc_mat, timestamp):
        """Convert and publish point cloud (simplified version)"""
        try:
            # Note: Full point cloud conversion is complex
            # This is a placeholder - you'd need sensor_msgs_py or similar
            # for full PointCloud2 message creation
            
            pc_data = pc_mat.get_data()
            if pc_data is not None:
                self.get_logger().debug(f'Point cloud shape: {pc_data.shape}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish point cloud: {e}')
    
    def publish_camera_info(self, timestamp):
        """Publish camera calibration information"""
        try:
            camera_info = self.zed.get_camera_information()
            calib = camera_info.camera_configuration.calibration_parameters.left_cam
            
            # Create CameraInfo message
            msg = CameraInfo()
            msg.header.stamp = timestamp
            msg.header.frame_id = 'zed_left_camera_frame'
            
            msg.width = camera_info.camera_configuration.resolution.width
            msg.height = camera_info.camera_configuration.resolution.height
            
            # Camera matrix [fx  0 cx]
            #               [ 0 fy cy]  
            #               [ 0  0  1]
            msg.k = [calib.fx, 0.0, calib.cx,
                     0.0, calib.fy, calib.cy,
                     0.0, 0.0, 1.0]
            
            # Distortion coefficients [k1, k2, p1, p2, k3]
            msg.d = [calib.disto[0], calib.disto[1], calib.disto[2], calib.disto[3], calib.disto[4]]
            
            # Rectification matrix (identity for rectified images)
            msg.r = [1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0]
            
            # Projection matrix
            msg.p = [calib.fx, 0.0, calib.cx, 0.0,
                     0.0, calib.fy, calib.cy, 0.0,
                     0.0, 0.0, 1.0, 0.0]
            
            self.camera_info_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish camera info: {e}')
    
    def destroy_node(self):
        """Clean up resources"""
        self.get_logger().info('Shutting down ZED Depth Publisher...')
        if hasattr(self, 'zed') and self.zed.is_opened():
            self.zed.close()
        super().destroy_node()

def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = ZEDDepthPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\\nShutdown requested by user')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()