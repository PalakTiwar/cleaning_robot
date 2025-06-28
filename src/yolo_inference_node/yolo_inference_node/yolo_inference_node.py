import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloInferenceNode(Node):
    def __init__(self):
        super().__init__('yolo_inference_node')

        # Load both models
        self.model_coco = YOLO('yolov8n.pt')  # Detects bed, couch
        self.model_custom = YOLO('/home/palak/clean_bot/src/sam_bot_nav2_gz/models/custom_yolo.pt')  # Detects bathtub, dining

        self.subscription = self.create_subscription(Image, '/robot_cam', self.image_callback, 10)
        self.publisher_ = self.create_publisher(String, '/detected_objects', 10)
        self.bridge = CvBridge()

        self.target_objects = {'bed', 'couch', 'bathtub', 'dining'}  # 👈 Only allow these

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run inference
            results_coco = self.model_coco(frame, verbose=False)
            results_custom = self.model_custom(frame, verbose=False)

            detected = set()

            # Collect COCO detections
            for result in results_coco:
                for c in result.boxes.cls:
                    class_name = self.model_coco.names[int(c)]
                    if class_name in self.target_objects:
                        detected.add(class_name)

            # Collect custom detections
            for result in results_custom:
                for c in result.boxes.cls:
                    class_name = self.model_custom.names[int(c)]
                    if class_name in self.target_objects:
                        detected.add(class_name)

            # Only publish allowed objects (no logs)
            for obj in detected:
                self.publisher_.publish(String(data=obj))

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

