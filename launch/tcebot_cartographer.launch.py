"""
Launch file for TCE-Robot navigation (without Gazebo), including Cartographer for odometry

Author: Dhanush M (https://github.com/dhanushshettigar)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('tcebot_gz')
    nav2_dir = get_package_share_directory('nav2_bringup')
    cartographer_dir = get_package_share_directory('cartographer_mapping')  # Cartographer package
    launch_dir = os.path.join(nav2_dir, 'launch')
    desc_dir = get_package_share_directory('tcebot_description')

    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='false', description='Whether to apply a namespace to the navigation stack'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=os.path.join(bringup_dir, 'maps', 'depot.yaml'), description='Full path to map file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock if True'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(bringup_dir, 'params', 'nav2_params_robot.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument('use_composition', default_value='True', description='Use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument('use_respawn', default_value='False', description='Respawn if a node crashes')

    declare_use_rviz_cmd = DeclareLaunchArgument('use_rviz', default_value='True', description='Whether to start RVIZ')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use',
    )

    # Start Robot State Publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', os.path.join(desc_dir, 'urdf', 'tcebot.urdf.xacro')])}],
    )

    # make this True to map - False to Nav2

    # Start Cartographer for Odometry (without SLAM)
    cartographer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cartographer_dir, 'launch', 'cartographer_mapping_launch.py')),
        launch_arguments={'exploration': 'True'}.items(),  # Run in localization mode
    )

    # Start Nav2 Bringup
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': 'False',  # No SLAM, using pre-saved map
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items(),
    )

    # Start RViz
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'use_sim_time': use_sim_time,
            'rviz_config': LaunchConfiguration('rviz_config_file'),
        }.items(),
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add the actions to launch all nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(cartographer_cmd)  # Include Cartographer for odometry
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)

    return ld
