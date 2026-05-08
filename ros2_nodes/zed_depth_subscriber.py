#!/usr/bin/env python3
"""
ZED Depth Subscriber Node - Your ROS2 depth analysis patterns applied to ZED data
================================================================================

This is your depth camera subscriber code adapted to work with ZED VSO data
published by the zed_depth_publisher node. It uses your exact ROS2 patterns
and obstacle detection logic.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ZEDDepthSubscriber(Node):
    """
    Your ROS2 depth analysis patterns applied to ZED depth data
    Based on your depth camera subscriber code
    """
    
    def __init__(self):
        super().__init__('zed_depth_subscriber')
        
        # Initialize OpenCV-ROS2 bridge (your pattern)
        self.cv_bridge = CvBridge()
        
        # ZED DEPTH IMAGE SUBSCRIPTION (adapted from your code)
        self.depth_subscription = self.create_subscription(
            Image,                     # Message type
            '/zed/depth/image_raw',    # ZED depth topic
            self.depth_callback,       # Your callback function
            10                         # Queue size
        )
        
        # RGB IMAGE SUBSCRIPTION (your pattern)
        self.rgb_subscription = self.create_subscription(
            Image,
            '/zed/rgb/image_raw',      # ZED RGB topic
            self.rgb_callback,
            10
        )
        
        # INSTANCE VARIABLES (your pattern)
        self.latest_depth_image = None
        self.latest_rgb_image = None
        
        # Detection parameters (your exact values)
        self.min_depth = 0.1          # Minimum depth in meters
        self.max_depth = 5.0          # Maximum depth in meters
        self.obstacle_threshold = 1.0  # Objects closer than 1m are \"obstacles\"
        
        # Logging (your pattern)
        self.get_logger().info('ZED Depth Subscriber started!')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - /zed/depth/image_raw (ZED depth data)')
        self.get_logger().info('  - /zed/rgb/image_raw (ZED RGB data)')
        
        # Console output (your pattern)
        print(\"=\" * 60)
        print(\"ZED Depth Subscriber - VSO Data Analysis\")
        print(\"=\" * 60)
        print(\"Processing ZED depth data with your ROS2 patterns\")
        print(f\"Obstacle detection: Objects closer than {self.obstacle_threshold}m\")
        print(\"Press Ctrl+C to stop\")
        print(\"=\" * 60)
    
    def depth_callback(self, msg):
        \"\"\"Your exact depth processing logic adapted for ZED data\"\"\"
        try:
            # CONVERT ROS2 IMAGE TO OPENCV FORMAT (your pattern)
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            # ↑ ZED publishes in meters as 32FC1
            
            # Store for processing (your pattern)
            self.latest_depth_image = depth_image
            
            # PROCESS THE DEPTH DATA (your function)
            self.process_depth_image(depth_image)
            
        except Exception as e:
            # ERROR HANDLING (your pattern)
            self.get_logger().error(f'Error processing ZED depth image: {e}')
    
    def rgb_callback(self, msg):
        \"\"\"Your RGB processing (adapted for ZED)\"\"\"
        try:
            # Convert RGB image (your pattern)
            rgb_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_rgb_image = rgb_image
            
        except Exception as e:
            self.get_logger().error(f'Error processing ZED RGB image: {e}')
    
    def process_depth_image(self, depth_image):
        \"\"\"
        Your EXACT depth processing algorithm adapted for ZED data
        \"\"\"
        # GET IMAGE DIMENSIONS (your pattern)
        height, width = depth_image.shape
        
        # FILTER VALID DEPTHS (your exact logic)
        valid_depths = np.logical_and(
            depth_image >= self.min_depth,
            depth_image <= self.max_depth
        )
        
        # FIND OBSTACLES (your exact logic)
        obstacles = np.logical_and(
            valid_depths,
            depth_image < self.obstacle_threshold
        )
        
        # COUNT OBSTACLE PIXELS (your pattern)
        obstacle_pixel_count = np.sum(obstacles)
        total_valid_pixels = np.sum(valid_depths)
        
        # CALCULATE STATISTICS (your exact calculations)
        if total_valid_pixels > 0:
            obstacle_percentage = (obstacle_pixel_count / total_valid_pixels) * 100
            
            # Find closest object (your pattern)
            valid_depth_values = depth_image[valid_depths]
            if len(valid_depth_values) > 0:
                closest_distance = np.min(valid_depth_values)
                average_distance = np.mean(valid_depth_values)
                
                # LOGGING WITH ANALYSIS (your exact format)
                self.get_logger().info(
                    f'ZED Depth Analysis - '
                    f'Closest: {closest_distance:.2f}m, '
                    f'Average: {average_distance:.2f}m, '
                    f'Obstacles: {obstacle_percentage:.1f}%'
                )
                
                # CONSOLE OUTPUT (your exact format)
                print(f\"📊 ZED Stats: Closest={closest_distance:.2f}m | \"
                      f\"Avg={average_distance:.2f}m | \"
                      f\"Obstacles={obstacle_percentage:.1f}% | \"
                      f\"Resolution={width}x{height}\")
                
                # ALERT FOR CLOSE OBSTACLES (your exact logic)
                if closest_distance < 0.5:  # Within 50cm
                    print(\"⚠️  WARNING: Very close obstacle detected in ZED data!\")
                    self.get_logger().warn(f'Close obstacle at {closest_distance:.2f}m!')
        
        # DISPLAY IMAGES (your pattern)
        self.display_images(depth_image, obstacles)
    
    def display_images(self, depth_image, obstacles):
        \"\"\"Your exact visualization code\"\"\"
        try:
            # NORMALIZE DEPTH FOR VISUALIZATION (your pattern)
            depth_normalized = cv2.normalize(
                depth_image, None, 0, 255, cv2.NORM_MINMAX
            ).astype(np.uint8)
            
            # APPLY COLOR MAP (your pattern)
            depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            
            # HIGHLIGHT OBSTACLES (your pattern)
            obstacle_display = obstacles.astype(np.uint8) * 255
            
            # DISPLAY WINDOWS (your pattern)
            cv2.imshow('ZED Depth Camera View', depth_colored)
            cv2.imshow('ZED Obstacle Detection', obstacle_display)
            
            # Show RGB if available (your pattern)
            if self.latest_rgb_image is not None:
                cv2.imshow('ZED RGB Camera View', self.latest_rgb_image)
            
            cv2.waitKey(1)  # Required for OpenCV display
            
        except Exception as e:
            self.get_logger().error(f'Error displaying ZED images: {e}')

def main(args=None):
    \"\"\"Your exact main function pattern\"\"\"
    try:
        # INITIALIZE ROS2 (your pattern)
        rclpy.init(args=args)
        
        print(\"Initializing ZED Depth Subscriber...\")
        
        # CREATE NODE INSTANCE (your pattern)
        zed_depth_subscriber = ZEDDepthSubscriber()
        
        print(\"Node created successfully!\")
        print(\"Waiting for ZED camera data... (Press Ctrl+C to stop)\")
        
        # KEEP NODE RUNNING (your pattern)
        rclpy.spin(zed_depth_subscriber)
        
    except KeyboardInterrupt:
        # HANDLE CTRL+C (your pattern)
        print(\"\\nShutting down ZED Depth Subscriber...\")
        zed_depth_subscriber.get_logger().info('Node interrupted by user')
        
    finally:
        # CLEANUP RESOURCES (your pattern)
        cv2.destroyAllWindows()
        if 'zed_depth_subscriber' in locals():
            zed_depth_subscriber.destroy_node()
        rclpy.shutdown()
        print(\"ZED Depth Subscriber shutdown complete!\")

if __name__ == '__main__':
    main()