from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os
from typing import List


def launch_setup(context, *args, **kwargs) -> List[Node]:

    # Declared Args
    use_sim_time    = LaunchConfiguration("use_sim_time")
    log_level       = LaunchConfiguration("log_level")

    # Share Directories
    fer_bt_share = get_package_share_directory("fer_behavior_trees")


    bt_server_config = os.path.join(fer_bt_share, "config", "bt_server.yaml")
    bt_server_node = Node(
        package="fer_behavior_trees",
        executable="bt_server_node",
        name="bt_action_server",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            bt_server_config,
            {"use_sim_time": use_sim_time},
        ]
    )

    return [bt_server_node]


def generate_launch_description():

    return LaunchDescription(generate_declared_arguments() + 
                             [OpaqueFunction(function=launch_setup)]
    )


def generate_declared_arguments() -> List[DeclareLaunchArgument]:

    return [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock"
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="Level of logging for the ros2_nodes. Possible args ('debug', 'info', 'warn', 'error', 'fatal')."
        ),
    ]
