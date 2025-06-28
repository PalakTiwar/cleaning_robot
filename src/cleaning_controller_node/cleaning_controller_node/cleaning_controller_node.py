#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class CleaningControllerNode(Node):
    def __init__(self):
        super().__init__('cleaning_controller_node')
        self.nav_status_sub = self.create_subscription(String, '/nav2_status', self.status_callback, 10)
        self.clean_pub = self.create_publisher(String, '/cleaning_complete', 10)


def status_callback(self, msg):
    if msg.data == "reached":
        self.get_logger().info("Starting cleaning...")
        start = time.time()
        time.sleep(5)  # Simulate cleaning
        duration = int(time.time() - start)
        done_msg = String()
        done_msg.data = f"cleaning_done,{duration}"
        self.clean_pub.publish(done_msg)
        self.get_logger().info(f"Cleaning complete in {duration} sec")

def main(args=None):
    rclpy.init(args=args)
    node = CleaningControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
