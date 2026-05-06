import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyzed.sl as sl
import cv2

class SVOPublisher(Node):
    def __init__(self):
        super().__init__('svo_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # Open SVO
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.set_from_svo_file("C:/ARCS_Vision/svo_samples/sofa.svo")
        err = self.zed.open(init_params)

        if err != sl.ERROR_CODE.SUCCESS:
            raise Exception("Failed to open SVO")

        self.mat = sl.Mat()
        self.timer = self.create_timer(0.03, self.grab_frame)

    def grab_frame(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.mat, sl.VIEW.LEFT)
            frame = self.mat.get_data()

            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = SVOPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
