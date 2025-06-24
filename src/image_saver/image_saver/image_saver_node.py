import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/robot_cam', self.image_callback, 10)
        self.image_count = 0

        # Configuration for saving 3 images per second
        self.min_save_interval_sec = 0.333
        self.last_save_time = self.get_clock().now()

        # Define object class label and create output directory
        self.label = 'dining_table'
        self.output_dir = f'/home/palak/custom_dataset/{self.label}'
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f"ImageSaver node started. Saving to: {self.output_dir}")
        self.get_logger().info(f"Aiming to save 3 images per second (min interval: {self.min_save_interval_sec}s).")

    def image_callback(self, msg):
        current_time = self.get_clock().now()
        time_since_last_save = (current_time - self.last_save_time).nanoseconds / 1e9

        # Skip frame if too soon to save another image
        if time_since_last_save < self.min_save_interval_sec:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename = os.path.join(self.output_dir, f"{self.label}_{timestamp}_{self.image_count}.jpg")
            cv2.imwrite(filename, frame)
            self.get_logger().info(f"📸 Saved image: {filename}")
            self.image_count += 1
            
            self.last_save_time = current_time

        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ImageSaver node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
