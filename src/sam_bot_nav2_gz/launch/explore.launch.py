from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='explore_lite',  # Name as defined in package.xml of the explore node
            executable='explore',
            name='explore',
            output='screen',
            parameters=[
                {
                    'visualize': True,
                    'frequency': 1.0,
                    'progress_timeout': 90.0,
                    'use_sim_time': True
                }
            ],
            remappings=[
                ('costmap', '/global_costmap/costmap')
            ]
        )
    ])

