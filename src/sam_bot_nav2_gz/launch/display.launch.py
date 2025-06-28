import launch
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    AndSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
import os
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="sam_bot_nav2_gz"
    ).find("sam_bot_nav2_gz")
    default_model_path = os.path.join(
        pkg_share, "src/description/sam_bot_description.urdf"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_localization = LaunchConfiguration("use_localization")
    use_rviz = LaunchConfiguration("use_rviz")
    log_level = LaunchConfiguration("log_level")
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    run_headless = LaunchConfiguration("run_headless")
    world_file_name = LaunchConfiguration("world_file")
    robot_controllers_config = PathJoinSubstitution([
        FindPackageShare("sam_bot_nav2_gz"), # <-- THIS IS WHERE YOUR PACKAGE NAME COMES IN
        "config",
        "robot_controllers.yaml"
    ])
    gz_models_path = ":".join([pkg_share, os.path.join(pkg_share, "models")])
    #gz_models_path = os.path.join(pkg_share, "models")
    world_path = PathJoinSubstitution([pkg_share, "world", world_file_name])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")]),
             'use_sim_time': use_sim_time},
        ],
    )
    
     # Controller Manager Node
    # This node loads the robot_controllers.yaml and makes controllers available
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers_config, {'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True, # Recommended for better output in a terminal
        arguments=['--ros-args', '--log-level', log_level],
    )

    rviz_node = Node(
        condition=IfCondition(AndSubstitution(NotSubstitution(run_headless), use_rviz)),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Localize using odometry and IMU data. 
    # It can be turned off because the navigation stack uses AMCL with lidar data for localization
    robot_localization_node = Node(
        condition=launch.conditions.IfCondition(use_localization),
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/ekf.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # gazebo have to be executed with shell=False, or test_launch won't terminate it
    #   see: https://github.com/ros2/launch/issues/545
    # This code is form taken ros_gz_sim and modified to work with shell=False
    #   see: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim/launch/gz_sim.launch.py.in
    gz_env = {'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':  # TODO(CH3): To support pre-garden. Deprecated.
                      ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                                os.environ.get('LD_LIBRARY_PATH', default='')])}
    gazebo = [
        ExecuteProcess(
            condition=launch.conditions.IfCondition(run_headless),
            cmd=['ruby', FindExecutable(name="ign"), 'gazebo',  '-r', '-v', gz_verbosity, '-s', '--headless-rendering', world_path],
            output='screen',
            additional_env=gz_env, # type: ignore
            shell=False,
        ),
        ExecuteProcess(
            condition=launch.conditions.UnlessCondition(run_headless),
            cmd=['ruby', FindExecutable(name="ign"), 'gazebo',  '-r', '-v', gz_verbosity, world_path],
            output='screen',
            additional_env=gz_env, # type: ignore
            shell=False,
        )
    ]

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "sam_bot",
            "-topic",
            "robot_description",
            "-z",
            "1.0", #leave it as it is
            "-x",
            "-3.0",# x here
            "-y",
	    "4.5",# y here
	    "-Y",   # <--- ADDED: Yaw argument (rotation around Z-axis)
            "0.0",
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[
    {"use_sim_time": use_sim_time},
],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/sky_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/robot_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            # Clock message is necessary for the diff_drive_controller to accept commands https://github.com/ros-controls/gz_ros2_control/issues/106
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )

    '''controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
    PathJoinSubstitution([
        FindPackageShare("sam_bot_nav2_gz"),
        "config",
        "brush_controllers.yaml"
    ]),
    {'use_sim_time': use_sim_time}
],
 # Pass your controller config here
        output='screen',
        emulate_tty=True, # Recommended for better output in a terminal
        arguments=['--ros-args', '--log-level', log_level],
    )'''
    
    load_joint_state_broadcaster = ExecuteProcess(
        name="activate_joint_state_broadcaster",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        shell=False,
        output="screen",
    )
     
    load_soft_brush_controller = ExecuteProcess(
    cmd=["ros2", "control", "load_controller", "--set-state", "active", "soft_brush_controller"],
    output="screen"
    )

    load_hard_brush_controller = ExecuteProcess(
    cmd=["ros2", "control", "load_controller", "--set-state", "active", "hard_brush_controller"],
    output="screen"
    )
    
    load_diff_drive_base_controller = ExecuteProcess(
        name="activate_diff_drive_base_controller",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "diff_drive_base_controller",
        ],
        shell=False,
        output="screen",
    )

    relay_odom = Node(
        name="relay_odom",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/diff_drive_base_controller/odom",
                "output_topic": "/odom",
            }
        ],
        output="screen",
    )

    relay_cmd_vel = Node(
        name="relay_cmd_vel",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/cmd_vel",
                "output_topic": "/diff_drive_base_controller/cmd_vel_unstamped",
            }
        ],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=gz_models_path,
            ),
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="True",
                description="Start RViz",
            ),
            DeclareLaunchArgument(
                name="run_headless",
                default_value="False",
                description="Start GZ in hedless mode and don't start RViz (overrides use_rviz)",
            ),
            DeclareLaunchArgument(
                name="world_file",
                default_value="empty.sdf",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            DeclareLaunchArgument(
                name="use_localization",
                default_value="True",
                description="Use EKF to estimagte odom->base_link transform from IMU + wheel odometry",
            ),
            DeclareLaunchArgument(
                "gz_verbosity",
                default_value="3",
                description="Verbosity level for Ignition Gazebo (0~4).",
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value="",
                description="Extra args for Gazebo (ie. '-s' for running headless)",
            ),
            DeclareLaunchArgument(
                name="log_level",
                default_value="warn",
                description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
            ),
            bridge,
            robot_state_publisher_node,
            controller_manager_node,
            spawn_entity,
            robot_localization_node,
            rviz_node,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_diff_drive_base_controller],
                )
            ),
            RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_diff_drive_base_controller,
                on_exit=[
                    load_soft_brush_controller,
                    load_hard_brush_controller]
               )
           ),          
            relay_odom,
            relay_cmd_vel,
        ] + gazebo
    )
