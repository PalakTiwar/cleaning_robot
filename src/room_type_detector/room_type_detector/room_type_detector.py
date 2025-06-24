import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RoomTypeDetector(Node):
    def __init__(self):
        super().__init__('room_type_detector')
        self.subscription = self.create_subscription(
            String,
            '/detected_objects',
            self.detect_callback,
            10
        )
        self.publisher = self.create_publisher(String, '/current_room_type', 10)

        self.room_map = {
            'bed': 'bedroom',
            'bathtub': 'bathroom',
            'couch': 'living_room',
            'dining': 'kitchen',
        }

    def detect_callback(self, msg):
        obj = msg.data.lower()
        if obj in self.room_map:
            room = self.room_map[obj]
            self.get_logger().info(f'Object: {obj} → Room: {room}')
            self.publisher.publish(String(data=room))

def main(args=None):
    rclpy.init(args=args)
    node = RoomTypeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

