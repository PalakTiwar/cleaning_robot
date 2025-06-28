import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg_share = FindPackageShare("sam_bot_nav2_gz").find("sam_bot_nav2_gz")
    world_file = LaunchConfiguration("world_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    run_headless = LaunchConfiguration("run_headless")

    model_path = PathJoinSubstitution([pkg_share, "src/description/sam_bot_description.urdf"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz/navigation_config.rviz"])
    gz_models_path = ":".join([pkg_share, os.path.join(pkg_share, "models")])
    world_path = PathJoinSubstitution([pkg_share, "world", world_file])

    # Gazebo Ignition environment setup
    gz_env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''), os.environ.get('LD_LIBRARY_PATH', '')]),
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''), os.environ.get('LD_LIBRARY_PATH', '')]),
    }

    gazebo = [
        ExecuteProcess(
            condition=IfCondition(run_headless),
            cmd=['ruby', 'ign', 'gazebo', '-r', '-v', '3', '--headless-rendering', world_path],
            output='screen',
            additional_env=gz_env,
            shell=False
        ),
        ExecuteProcess(
            condition=UnlessCondition(run_headless),
            cmd=['ruby', 'ign', 'gazebo', '-r', '-v', '3', world_path],
            output='screen',
            additional_env=gz_env,
            shell=False
        )
    ]

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "sam_bot",
            "-topic", "robot_description",
            "-z", "1.0",
            "-x", "-3.0",
            "-y", "4.5",
            "-Y", "0.0"
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": PathJoinSubstitution([pkg_share, "src/description/sam_bot_description.urdf"])},
            {"use_sim_time": use_sim_time}
        ]
    )

    flask_bridge_node = Node(
        package="flask_bridge_node",
        executable="flask_bridge_node",
        name="flask_bridge_node",
        output="screen"
    )

    room_navigation_node = Node(
        package="room_navigation_node",
        executable="room_navigation_node",
        name="room_navigation_node",
        output="screen"
    )

    cleaning_controller_node = Node(
        package="cleaning_controller_node",
        executable="cleaning_controller_node",
        name="cleaning_controller_node",
        output="screen"
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config]
    )

    return LaunchDescription([
        DeclareLaunchArgument("world_file", default_value="empty.sdf", description="Gazebo world"),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        DeclareLaunchArgument("run_headless", default_value="False"),
        DeclareLaunchArgument("log_level", default_value="info"),
        SetEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=gz_models_path),

        robot_state_publisher,
        spawn_entity,
        flask_bridge_node,
        room_navigation_node,
        cleaning_controller_node,
        rviz_node,
    ] + gazebo)

