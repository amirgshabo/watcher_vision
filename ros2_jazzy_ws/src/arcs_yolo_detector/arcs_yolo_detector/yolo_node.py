import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
import numpy as np
import cv2
import onnxruntime as ort

class YoloOnnxNode(Node):
    def __init__(self):
        super().__init__('yolo_onnx_node')

        self.pub = self.create_publisher(Detection2DArray, '/yolo_detections', 10)

        # Timer at 10 FPS
        self.timer = self.create_timer(0.1, self.run_inference)

        # Load ONNX model
        self.model_path = "/ros2_ws/onnx/model.onnx"
        self.session = ort.InferenceSession(self.model_path, providers=["CPUExecutionProvider"])

        self.input_name = self.session.get_inputs()[0].name

        # Webcam
        self.cap = cv2.VideoCapture(0)

        self.get_logger().info("YOLO ONNX Node started")

    def preprocess(self, frame):
        img = cv2.resize(frame, (640, 640))
        img = img[:, :, ::-1]  # BGR → RGB
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # CHW
        img = np.expand_dims(img, axis=0)
        return img

    def run_inference(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        input_tensor = self.preprocess(frame)

        outputs = self.session.run(None, {self.input_name: input_tensor})[0]

        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        for det in outputs:
            x1, y1, x2, y2, score, class_id = det[:6]

            if score < 0.5:
                continue

            detection = Detection2D()

            detection.bbox.center.x = float((x1 + x2) / 2)
            detection.bbox.center.y = float((y1 + y2) / 2)
            detection.bbox.size_x = float(x2 - x1)
            detection.bbox.size_y = float(y2 - y1)

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = int(class_id)
            hypothesis.score = float(score)

            detection.results.append(hypothesis)
            msg.detections.append(detection)

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloOnnxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
