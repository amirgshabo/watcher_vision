#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyzed.sl as sl
import numpy as np

class SVOPublisher(Node):
    def __init__(self):
        super().__init__('svo_publisher')
        
        # Declare parameters
        self.declare_parameter('svo_file', '/ros2_ws/test_data/sample.svo')
        self.declare_parameter('loop', True)
        self.declare_parameter('fps', 30.0)
        
        svo_path = self.get_parameter('svo_file').value
        self.loop = self.get_parameter('loop').value
        fps = self.get_parameter('fps').value
        
        # Initialize ZED camera for SVO playback
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.set_from_svo_file(svo_path)
        init_params.svo_real_time_mode = False  # Process as fast as possible
        
        self.get_logger().info(f'Opening SVO file: {svo_path}')
        err = self.zed.open(init_params)
        
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Failed to open SVO file: {err}')
            self.get_logger().error(f'Check that file exists at: {svo_path}')
            return
        
        # Get SVO information
        num_frames = self.zed.get_svo_number_of_frames()
        self.get_logger().info(f'SVO contains {num_frames} frames')
        
        # Create image objects
        self.image_zed = sl.Mat()
        self.depth_zed = sl.Mat()
        self.bridge = CvBridge()
        
        # Publishers
        self.image_pub = self.create_publisher(
            Image,
            '/camera/rgb/image',
            10
        )
        
        self.depth_pub = self.create_publisher(
            Image,
            '/camera/depth/image',
            10
        )
        
        # Timer to publish frames at specified FPS
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        self.frame_count = 0
        self.get_logger().info(f'SVO Publisher ready! Publishing at {fps} FPS')
    
    def publish_frame(self):
        # Grab next frame from SVO
        grab_status = self.zed.grab()
        
        if grab_status == sl.ERROR_CODE.SUCCESS:
            # Retrieve RGB image
            self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
            image_np = self.image_zed.get_data()
            
            # Convert BGRA to BGR (drop alpha channel)
            image_bgr = image_np[:, :, :3]
            
            # Create ROS message
            img_msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'
            
            # Publish RGB image
            self.image_pub.publish(img_msg)
            
            # Retrieve and publish depth
            self.zed.retrieve_measure(self.depth_zed, sl.MEASURE.DEPTH)
            depth_np = self.depth_zed.get_data()
            
            # Convert depth to 16-bit (millimeters)
            depth_mm = (depth_np * 1000).astype(np.uint16)
            
            depth_msg = self.bridge.cv2_to_imgmsg(depth_mm, encoding='16UC1')
            depth_msg.header.stamp = img_msg.header.stamp
            depth_msg.header.frame_id = 'camera_frame'
            
            self.depth_pub.publish(depth_msg)
            
            self.frame_count += 1
            
            # Log progress every 30 frames
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')
        
        elif grab_status == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            # End of SVO file
            if self.loop:
                self.get_logger().info('End of SVO reached, looping back to start...')
                self.zed.set_svo_position(0)
                self.frame_count = 0
            else:
                self.get_logger().info('End of SVO reached, stopping playback')
                self.timer.cancel()
        else:
            self.get_logger().warn(f'Grab failed with status: {grab_status}')
    
    def __del__(self):
        if hasattr(self, 'zed'):
            self.zed.close()
            self.get_logger().info('ZED camera closed')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SVOPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
