#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose # Import the Nav2 action type
from rclpy.action import ActionClient      # Import ActionClient
import sys

class RoomNavigationNode(Node):
    def __init__(self):
        super().__init__('room_navigation_node')
        self.subscriber = self.create_subscription(String, '/room_to_clean', self.room_callback, 10)

        # Initialize the ActionClient for Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('RoomNavigationNode initialized and ready to receive commands.')
        

    def room_callback(self, msg):
        room = msg.data.lower()
        goal_pose = self.get_pose_from_room(room)

        if goal_pose:
            self.send_navigation_goal(goal_pose, room)
        else:
            self.get_logger().warn(f"No coordinates found for room: {room}")

    def get_pose_from_room(self, room_name):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg() # Use current time

        room_coords = {
            "bedroom": (4.0, -3.0),
            "kitchen": (6.0, 4.0),
            "living room": (-2.0, 4.0),
            "bathroom": (-4.0, -3.0)
        }
        if room_name in room_coords:
            pose.pose.position.x = room_coords[room_name][0]
            pose.pose.position.y = room_coords[room_name][1]
            pose.pose.orientation.w = 1.0 # Default orientation (no rotation)
            return pose
        return None

    def send_navigation_goal(self, goal_pose, room_name):
        self.get_logger().info(f"Waiting for navigation action server for {room_name}...")
        self._action_client.wait_for_server(timeout_sec=10.0) # Wait for the action server to be ready

        if not self._action_client.server_is_ready():
            self.get_logger().error("Navigation action server not available after waiting.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose # Assign your PoseStamped to the goal message

        self.get_logger().info(f'Sending goal for {room_name} to Nav2: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}')

        # Send the goal and get a future for the goal handle
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        # Register callbacks for goal response and result
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        # Get a future for the result and add a callback
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        # Check action_msgs.msg.GoalStatus for status codes
        from action_msgs.msg import GoalStatus
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Robot reached destination.')
        else:
            self.get_logger().info(f'Goal failed with status: {status}')


def main(args=None):
    rclpy.init(args=args)
    node = RoomNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly by Ctrl+C')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
