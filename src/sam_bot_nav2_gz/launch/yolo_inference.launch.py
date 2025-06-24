from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sam_bot_nav2_gz',
            executable='yolo_inference_node',
            name='yolo_inference_node',
            output='screen',
        ),
    ])

