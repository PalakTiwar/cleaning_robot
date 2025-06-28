#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os

class FlaskBridgeNode(Node):
    def __init__(self):
        super().__init__('flask_bridge_node')
        self.subscriber = self.create_subscription(String, '/cleaning_complete', self.callback, 10)
        self.json_path = '/home/palak/clean_bot/status.json'

    def callback(self, msg):
        status, duration = msg.data.split(',')
        status_data = {
            "status": "Cleaning Completed",
            "duration": f"{duration} seconds"
        }
        with open(self.json_path, 'w') as f:
            json.dump(status_data, f)
        self.get_logger().info("Status written to JSON")
    
def main(args=None):
    rclpy.init(args=args)
    node = FlaskBridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
'''import rclpy
from rclpy.node import Node

class FlaskBridgeNode(Node):
    def __init__(self):
        super().__init__('flask_bridge_node')
        self.get_logger().info("FlaskBridgeNode is starting its minimal setup.")
        # self.create_timer(1.0, self.timer_callback) # Comment out any timers, publishers, subscribers
        # self.declare_parameter('my_param', 'default_value') # Comment out parameter declarations
        # self.get_logger().info("FlaskBridgeNode has finished its minimal setup.")

    # def timer_callback(self):
    #     self.get_logger().info("Minimal FlaskBridgeNode heartbeat.")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FlaskBridgeNode()
        node.get_logger().info("FlaskBridgeNode created, spinning...")
        rclpy.spin(node) # This keeps the node alive
    except Exception as e:
        if node:
            node.get_logger().error(f"FlaskBridgeNode crashed: {e}")
        else:
            print(f"FlaskBridgeNode failed to initialize: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()'''
