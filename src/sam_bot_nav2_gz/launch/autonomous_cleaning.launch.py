from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import os

def generate_launch_description():
    pkg_nav2 = os.path.join(os.getenv('HOME'), 'clean_bot', 'src', 'sam_bot_nav2_gz', 'launch')
    nav2_params = os.path.join(os.getenv('HOME'), 'clean_bot', 'src', 'sam_bot_nav2_gz', 'config', 'nav2_params.yaml')

    return LaunchDescription([
        Node(
            package='custom_explorer',
            executable='explore_node',
            name='explore_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='yolo_inference_node',
            executable='yolo_inference_node',
            name='yolo_inference_node',
            output='screen'
        ),

        Node(
            package='room_type_detector',
            executable='room_type_detector',
            name='room_type_detector',
            output='screen'
        ),

        Node(
            package='cleaning_controller_node',
            executable='cleaning_controller_node',
            name='cleaning_controller_node',
            output='screen'
        )
    ])
