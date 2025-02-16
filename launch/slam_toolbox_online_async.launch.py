import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from ament_index_python.packages import get_package_share_directory

########################
#    Modifications     #
########################
#ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
#ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
#ros2 service call /slam_toolbox/get_state lifecycle_msgs/srv/GetState "{}"



def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("tcebot_slam"),
                                   'config', 'slam_toolbox_async.yaml'),
        description='Full path to the ROS 2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': True, 'autostart': True}],
    )

    # Emit events to transition SLAM toolbox to configured and then active states
    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_node_name('/slam_toolbox'),
            transition_id=1,
        )
    )

    activate_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_node_name('/slam_toolbox'),
            transition_id=3,  
        )
    )

    return LaunchDescription([
        declare_slam_params_file_cmd,
        start_async_slam_toolbox_node,
        TimerAction(period=2.0, actions=[configure_slam]),
        TimerAction(period=4.0, actions=[activate_slam]),
    ])
