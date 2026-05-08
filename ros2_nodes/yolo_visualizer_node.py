#!/usr/bin/env python3
"""
YOLO Visualizer Node - ROS2 visualization node for YOLO detections
===================================================================

This node subscribes to YOLO detections and displays them in a window.
Separates detection logic from visualization for better modularity.

Subscribed Topics:
- /yolo/detections (vision_msgs/Detection2DArray) - 2D detections
- /yolo/detections_3d (vision_msgs/Detection3DArray) - 3D detections
- /yolo/debug_image (sensor_msgs/Image) - Pre-annotated debug image
- /zed/rgb/image_raw (sensor_msgs/Image) - Raw RGB image (if debug image not available)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection3DArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class YOLOVisualizerNode(Node):
    """ROS2 Node for visualizing YOLO detections"""
    
    def __init__(self):
        super().__init__('yolo_visualizer_node')
        
        # Declare parameters
        self.declare_parameter('window_name', 'YOLO Detections - ZED VSO')
        self.declare_parameter('use_debug_image', True)  # Use pre-annotated image or draw manually
        self.declare_parameter('display_stats', True)
        self.declare_parameter('save_output', False)
        self.declare_parameter('output_path', '/tmp/yolo_detections.avi')
        
        # Get parameters
        self.window_name = self.get_parameter('window_name').get_parameter_value().string_value
        self.use_debug_image = self.get_parameter('use_debug_image').get_parameter_value().bool_value
        self.display_stats = self.get_parameter('display_stats').get_parameter_value().bool_value
        self.save_output = self.get_parameter('save_output').get_parameter_value().bool_value
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value
        
        # Initialize CV Bridge
        self.cv_bridge = CvBridge()
        
        # Subscribers
        if self.use_debug_image:
            self.debug_image_sub = self.create_subscription(
                Image, '/yolo/debug_image', self.debug_image_callback, 10)
        else:
            self.rgb_sub = self.create_subscription(
                Image, '/zed/rgb/image_raw', self.rgb_callback, 10)
        
        self.detections_2d_sub = self.create_subscription(
            Detection2DArray, '/yolo/detections', self.detections_2d_callback, 10)
        self.detections_3d_sub = self.create_subscription(
            Detection3DArray, '/yolo/detections_3d', self.detections_3d_callback, 10)
        
        # Data storage
        self.latest_image = None
        self.latest_detections_2d = None
        self.latest_detections_3d = None
        
        # Statistics
        self.frame_count = 0
        self.total_detections = 0
        self.detection_counts = {}
        
        # Video writer for saving output
        self.video_writer = None
        
        # OpenCV window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)
        
        # Create timer for display updates
        self.display_timer = self.create_timer(0.033, self.display_callback)  # ~30 FPS
        
        self.get_logger().info('YOLO Visualizer initialized')
        self.get_logger().info(f'Window: {self.window_name}')
        self.get_logger().info(f'Using debug image: {self.use_debug_image}')
        self.get_logger().info(f'Display stats: {self.display_stats}')
        self.get_logger().info(f'Save output: {self.save_output}')
        if self.save_output:
            self.get_logger().info(f'Output path: {self.output_path}')
        self.get_logger().info('Controls: Q=quit, SPACE=pause, S=screenshot')
    
    def debug_image_callback(self, msg):
        """Callback for pre-annotated debug image"""
        try:
            self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error in debug image callback: {e}')
    
    def rgb_callback(self, msg):
        """Callback for raw RGB image"""
        try:
            self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error in RGB callback: {e}')
    
    def detections_2d_callback(self, msg):
        """Callback for 2D detections"""
        self.latest_detections_2d = msg
        
        # Update statistics
        num_detections = len(msg.detections)
        self.total_detections += num_detections
        
        for det in msg.detections:
            if det.results:
                class_id = det.results[0].hypothesis.class_id
                self.detection_counts[class_id] = self.detection_counts.get(class_id, 0) + 1
    
    def detections_3d_callback(self, msg):
        """Callback for 3D detections"""
        self.latest_detections_3d = msg
    
    def display_callback(self):
        """Display the current frame with detections"""
        if self.latest_image is None:
            return
        
        # Create display image
        display_image = self.latest_image.copy()
        
        # If not using debug image, draw detections manually
        if not self.use_debug_image and self.latest_detections_2d is not None:
            display_image = self.draw_detections_manual(
                display_image,
                self.latest_detections_2d,
                self.latest_detections_3d
            )
        
        # Add statistics overlay
        if self.display_stats:
            display_image = self.add_stats_overlay(display_image)
        
        # Display image
        cv2.imshow(self.window_name, display_image)
        
        # Save to video if enabled
        if self.save_output:
            if self.video_writer is None:
                self.init_video_writer(display_image.shape)
            if self.video_writer is not None:
                self.video_writer.write(display_image)
        
        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Quit requested by user')
            self.cleanup()
            rclpy.shutdown()
        elif key == ord(' '):
            self.get_logger().info('⏸️  Paused - Press any key to continue')
            cv2.waitKey(0)
        elif key == ord('s'):
            screenshot_path = f'/tmp/yolo_screenshot_{self.frame_count}.png'
            cv2.imwrite(screenshot_path, display_image)
            self.get_logger().info(f'Screenshot saved: {screenshot_path}')
        
        self.frame_count += 1
    
    def draw_detections_manual(self, image, detections_2d, detections_3d):
        """Manually draw detections on image"""
        for i, det_2d in enumerate(detections_2d.detections):
            # Extract 2D bbox
            cx = int(det_2d.bbox.center.position.x)
            cy = int(det_2d.bbox.center.position.y)
            w = int(det_2d.bbox.size_x)
            h = int(det_2d.bbox.size_y)
            
            x1 = cx - w // 2
            y1 = cy - h // 2
            x2 = cx + w // 2
            y2 = cy + h // 2
            
            # Get class and confidence
            if det_2d.results:
                class_id = det_2d.results[0].hypothesis.class_id
                confidence = det_2d.results[0].hypothesis.score
                
                # Draw bounding box
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Draw label
                label = f'ID:{class_id} ({confidence:.2f})'
                cv2.putText(image, label, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Draw 3D info if available
                if detections_3d and i < len(detections_3d.detections):
                    det_3d = detections_3d.detections[i]
                    x_3d = det_3d.bbox.center.position.x
                    y_3d = det_3d.bbox.center.position.y
                    z_3d = det_3d.bbox.center.position.z
                    
                    distance = math.sqrt(x_3d**2 + y_3d**2 + z_3d**2)
                    dist_label = f'{distance:.2f}m'
                    cv2.putText(image, dist_label, (x1, y2 + 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                
                # Draw center point
                cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)
        
        return image
    
    def add_stats_overlay(self, image):
        """Add statistics overlay to image"""
        # Create semi-transparent overlay
        overlay = image.copy()
        height, width = image.shape[:2]
        
        # Stats box background
        cv2.rectangle(overlay, (10, 10), (350, 120), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, image, 0.4, 0, image)
        
        # Stats text
        stats_text = [
            f'Frame: {self.frame_count}',
            f'Total Detections: {self.total_detections}',
            f'Avg/Frame: {self.total_detections / max(self.frame_count, 1):.2f}',
            f'Classes: {len(self.detection_counts)}'
        ]
        
        y_offset = 30
        for text in stats_text:
            cv2.putText(image, text, (20, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            y_offset += 25
        
        return image
    
    def init_video_writer(self, shape):
        """Initialize video writer for saving output"""
        try:
            height, width = shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter(
                self.output_path, fourcc, 30.0, (width, height))
            self.get_logger().info(f'Video writer initialized: {self.output_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize video writer: {e}')
            self.save_output = False
    
    def cleanup(self):
        """Cleanup resources"""
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f'Video saved: {self.output_path}')
        
        cv2.destroyAllWindows()
        
        # Print final statistics
        self.get_logger().info('='*60)
        self.get_logger().info('FINAL STATISTICS')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Frames processed: {self.frame_count}')
        self.get_logger().info(f'Total detections: {self.total_detections}')
        self.get_logger().info(f'Average detections/frame: {self.total_detections / max(self.frame_count, 1):.2f}')
        self.get_logger().info('Detections by class:')
        for class_id, count in sorted(self.detection_counts.items(), key=lambda x: x[1], reverse=True):
            self.get_logger().info(f'  Class {class_id}: {count}')
        self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YOLOVisualizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            node.cleanup()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
