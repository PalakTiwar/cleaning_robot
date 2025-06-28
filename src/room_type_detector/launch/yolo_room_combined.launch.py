from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # YOLO Inference Node (silent)
        Node(
            package='yolo_inference_node',
            executable='yolo_inference_node',
            name='yolo_inference_node',
            output='log',  # Silent in terminal
        ),

        # Room Type Detector (visible output)
        Node(
            package='room_type_detector',
            executable='room_type_detector',
            name='room_type_detector',
            output='screen',  # Display output of this only
        ),
    ])
