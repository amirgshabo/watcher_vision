#!/usr/bin/env python3
"""
YOLO Detector Node - ROS2 implementation for ZED object detection with 3D localization
======================================================================================

This node subscribes to ZED camera topics and publishes YOLO detections with 3D coordinates.

Subscribed Topics:
- /zed/rgb/image_raw (sensor_msgs/Image) - RGB image for YOLO detection
- /zed/point_cloud (sensor_msgs/PointCloud2) - 3D point cloud for depth
- /zed/depth/image_raw (sensor_msgs/Image) - Depth image (alternative to point cloud)

Published Topics:
- /yolo/detections (vision_msgs/Detection2DArray) - 2D detections with metadata
- /yolo/detections_3d (vision_msgs/Detection3DArray) - 3D detections with spatial info
- /yolo/debug_image (sensor_msgs/Image) - Annotated image for visualization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from vision_msgs.msg import Detection3D, Detection3DArray
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import math
import sensor_msgs_py.point_cloud2 as pc2


class YOLODetectorNode(Node):
    """ROS2 Node for YOLO object detection with ZED 3D depth"""
    
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # Declare parameters
        self.declare_parameter('model_path', 'yolov8n.onnx')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('use_point_cloud', True)  # True: use point cloud, False: use depth image
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('processing_rate', 10.0)  # Hz - process every Nth frame
        
        # Get parameters
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.use_point_cloud = self.get_parameter('use_point_cloud').get_parameter_value().bool_value
        self.publish_debug_image = self.get_parameter('publish_debug_image').get_parameter_value().bool_value
        processing_rate = self.get_parameter('processing_rate').get_parameter_value().double_value
        
        # Initialize CV Bridge
        self.cv_bridge = CvBridge()
        
        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.yolo = YOLO(model_path)
        self.get_logger().info('✅ YOLO model loaded successfully')
        
        # Publishers
        self.detections_2d_pub = self.create_publisher(
            Detection2DArray, '/yolo/detections', 10)
        self.detections_3d_pub = self.create_publisher(
            Detection3DArray, '/yolo/detections_3d', 10)
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(
                Image, '/yolo/debug_image', 10)
        
        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, '/zed/rgb/image_raw', self.rgb_callback, 10)
        
        if self.use_point_cloud:
            self.pointcloud_sub = self.create_subscription(
                PointCloud2, '/zed/point_cloud', self.pointcloud_callback, 10)
        else:
            self.depth_sub = self.create_subscription(
                Image, '/zed/depth/image_raw', self.depth_callback, 10)
        
        # Data storage
        self.latest_rgb_image = None
        self.latest_rgb_msg = None
        self.latest_pointcloud = None
        self.latest_depth_image = None
        self.latest_depth_msg = None
        
        # Frame rate control
        self.frame_count = 0
        self.process_every_n_frames = max(1, int(30.0 / processing_rate))  # Assuming 30 FPS input
        
        # Processing timer
        self.timer = self.create_timer(1.0 / processing_rate, self.process_detections)
        
        # Statistics
        self.total_detections = 0
        self.frames_processed = 0
        
        self.get_logger().info('='*60)
        self.get_logger().info('YOLO Detector Node Initialized')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Model: {model_path}')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'Using: {"Point Cloud" if self.use_point_cloud else "Depth Image"}')
        self.get_logger().info(f'Processing rate: {processing_rate} Hz')
        self.get_logger().info('='*60)
    
    def rgb_callback(self, msg):
        """Callback for RGB image"""
        try:
            self.latest_rgb_msg = msg
            self.latest_rgb_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error in RGB callback: {e}')
    
    def pointcloud_callback(self, msg):
        """Callback for point cloud"""
        self.latest_pointcloud = msg
    
    def depth_callback(self, msg):
        """Callback for depth image"""
        try:
            self.latest_depth_msg = msg
            self.latest_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Error in depth callback: {e}')
    
    def process_detections(self):
        """Main processing loop - run YOLO and publish detections"""
        # Check if we have required data
        if self.latest_rgb_image is None:
            return
        
        if self.use_point_cloud and self.latest_pointcloud is None:
            return
        
        if not self.use_point_cloud and self.latest_depth_image is None:
            return
        
        # Run YOLO detection
        results = self.yolo(self.latest_rgb_image)
        
        # Prepare detection messages
        detections_2d_msg = Detection2DArray()
        detections_3d_msg = Detection3DArray()
        detections_2d_msg.header = self.latest_rgb_msg.header
        detections_3d_msg.header = self.latest_rgb_msg.header
        
        detections_2d = []
        detections_3d = []
        
        # Process each detection
        for detection in results[0].boxes:
            # Extract 2D bounding box
            x1, y1, x2, y2 = detection.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # Get class and confidence
            class_id = int(detection.cls[0])
            confidence = float(detection.conf[0])
            class_name = results[0].names[class_id]
            
            # Filter by confidence
            if confidence < self.confidence_threshold:
                continue
            
            # Calculate center
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            
            # Get 3D coordinates
            point_3d, valid_3d = self.get_3d_point(center_x, center_y)
            
            # Create 2D detection message
            det_2d = Detection2D()
            det_2d.bbox.center.position.x = float(center_x)
            det_2d.bbox.center.position.y = float(center_y)
            det_2d.bbox.size_x = float(x2 - x1)
            det_2d.bbox.size_y = float(y2 - y1)
            
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(class_id)
            hypothesis.hypothesis.score = confidence
            det_2d.results.append(hypothesis)
            
            detections_2d.append(det_2d)
            
            # Create 3D detection if valid depth
            if valid_3d:
                det_3d = Detection3D()
                det_3d.bbox.center.position.x = point_3d[0]
                det_3d.bbox.center.position.y = point_3d[1]
                det_3d.bbox.center.position.z = point_3d[2]
                
                # Calculate distance
                distance = math.sqrt(point_3d[0]**2 + point_3d[1]**2 + point_3d[2]**2)
                
                hypothesis_3d = ObjectHypothesisWithPose()
                hypothesis_3d.hypothesis.class_id = str(class_id)
                hypothesis_3d.hypothesis.score = confidence
                det_3d.results.append(hypothesis_3d)
                
                detections_3d.append(det_3d)
                
                # Log detection
                if self.frames_processed % 10 == 0:  # Log every 10th frame
                    self.get_logger().info(
                        f'Detected: {class_name} | Conf: {confidence:.2f} | '
                        f'3D Dist: {distance:.2f}m | Z: {point_3d[2]:.2f}m'
                    )
        
        # Publish detections
        detections_2d_msg.detections = detections_2d
        detections_3d_msg.detections = detections_3d
        
        self.detections_2d_pub.publish(detections_2d_msg)
        self.detections_3d_pub.publish(detections_3d_msg)
        
        # Publish debug image
        if self.publish_debug_image and len(detections_2d) > 0:
            debug_image = self.draw_detections(
                self.latest_rgb_image.copy(), 
                results[0].boxes, 
                results[0].names
            )
            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = self.latest_rgb_msg.header
            self.debug_image_pub.publish(debug_msg)
        
        # Update statistics
        self.total_detections += len(detections_2d)
        self.frames_processed += 1
        
        if self.frames_processed % 100 == 0:
            avg_detections = self.total_detections / self.frames_processed
            self.get_logger().info(
                f'Stats: {self.frames_processed} frames | '
                f'{self.total_detections} total detections | '
                f'Avg: {avg_detections:.2f} per frame'
            )
    
    def get_3d_point(self, x, y):
        """Get 3D coordinates from point cloud or depth image"""
        if self.use_point_cloud:
            return self.get_point_from_cloud(x, y)
        else:
            return self.get_point_from_depth(x, y)
    
    def get_point_from_cloud(self, x, y):
        """Extract 3D point from point cloud at pixel coordinates"""
        try:
            # Read point cloud data at specific pixel
            gen = pc2.read_points(
                self.latest_pointcloud, 
                field_names=('x', 'y', 'z'), 
                skip_nans=False,
                uvs=[[x, y]]
            )
            
            for point in gen:
                if not np.isnan(point[0]) and not np.isnan(point[2]):
                    return [point[0], point[1], point[2]], True
            
            return [0.0, 0.0, 0.0], False
            
        except Exception as e:
            self.get_logger().error(f'Error reading point cloud: {e}')
            return [0.0, 0.0, 0.0], False
    
    def get_point_from_depth(self, x, y):
        """Calculate 3D point from depth image (simplified, needs camera calibration)"""
        try:
            if y >= self.latest_depth_image.shape[0] or x >= self.latest_depth_image.shape[1]:
                return [0.0, 0.0, 0.0], False
            
            depth = self.latest_depth_image[y, x]
            
            if np.isnan(depth) or depth <= 0.0 or depth > 20.0:
                return [0.0, 0.0, 0.0], False
            
            # Simplified 3D calculation (assumes pinhole model)
            # For accurate results, use camera calibration from camera_info
            fx = 700.0  # Approximate focal length - should come from camera_info
            fy = 700.0
            cx = self.latest_depth_image.shape[1] / 2
            cy = self.latest_depth_image.shape[0] / 2
            
            z = float(depth)
            x_3d = (x - cx) * z / fx
            y_3d = (y - cy) * z / fy
            
            return [x_3d, y_3d, z], True
            
        except Exception as e:
            self.get_logger().error(f'Error calculating 3D from depth: {e}')
            return [0.0, 0.0, 0.0], False
    
    def draw_detections(self, image, boxes, class_names):
        """Draw bounding boxes and labels on image"""
        for detection in boxes:
            x1, y1, x2, y2 = detection.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            class_id = int(detection.cls[0])
            confidence = float(detection.conf[0])
            
            if confidence < self.confidence_threshold:
                continue
            
            class_name = class_names[class_id]
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            
            # Get 3D info
            point_3d, valid = self.get_3d_point(center_x, center_y)
            
            # Draw box
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label
            label = f'{class_name} {confidence:.2f}'
            cv2.putText(image, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Draw 3D distance if valid
            if valid:
                distance = math.sqrt(point_3d[0]**2 + point_3d[1]**2 + point_3d[2]**2)
                dist_label = f'{distance:.2f}m'
                cv2.putText(image, dist_label, (x1, y2 + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # Draw center point
            cv2.circle(image, (center_x, center_y), 5, (0, 0, 255), -1)
        
        return image


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YOLODetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
