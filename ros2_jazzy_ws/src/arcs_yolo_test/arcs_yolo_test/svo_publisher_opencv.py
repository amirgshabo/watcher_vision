#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SVOPublisherOpenCV(Node):
    def __init__(self):
        super().__init__('svo_publisher_opencv')
        
        # Declare parameters
        self.declare_parameter('svo_file', '/ros2_ws/test_data/wall.svo')
        self.declare_parameter('loop', True)
        self.declare_parameter('fps', 30.0)
        
        svo_path = self.get_parameter('svo_file').value
        self.loop = self.get_parameter('loop').value
        fps = self.get_parameter('fps').value
        
        # Open video file
        self.cap = cv2.VideoCapture(svo_path)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open SVO file: {svo_path}')
            return
        
        # Get video info
        total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        video_fps = self.cap.get(cv2.CAP_PROP_FPS)
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self.get_logger().info(f'Opened SVO: {svo_path}')
        self.get_logger().info(f'Resolution: {width}x{height}')
        self.get_logger().info(f'Total frames: {total_frames}')
        self.get_logger().info(f'Original FPS: {video_fps}')
        
        self.bridge = CvBridge()
        
        # Publisher
        self.image_pub = self.create_publisher(
            Image,
            '/camera/rgb/image',
            10
        )
        
        # Timer to publish frames
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        self.frame_count = 0
        self.get_logger().info(f'Publishing at {fps} FPS')
    
    def publish_frame(self):
        ret, frame = self.cap.read()
        
        if ret:
            # Create ROS message
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'
            
            self.image_pub.publish(img_msg)
            
            self.frame_count += 1
            
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')
        else:
            # End of video
            if self.loop:
                self.get_logger().info('End of video, looping...')
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.frame_count = 0
            else:
                self.get_logger().info('End of video')
                self.timer.cancel()
    
    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SVOPublisherOpenCV()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
