import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DummyYoloPublisher(Node):
    def __init__(self):
        super().__init__("dummy_yolo_publisher")
        self.publisher = self.create_publisher(String, "fake_yolo_detections", 10)
        self.timer = self.create_timer(0.5, self.publish_msg)

    def publish_msg(self):
        msg = String()
        msg.data = "person, x=1.2 y=0.5 z=3.4"
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = DummyYoloPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
