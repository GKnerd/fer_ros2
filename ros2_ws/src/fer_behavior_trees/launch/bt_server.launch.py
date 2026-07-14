from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("fer_behavior_trees"),
        "config",
        "bt_server.yaml",
    )

    bt_server_node = Node(
        package="fer_behavior_trees",
        executable="bt_server_node",
        name="bt_action_server",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([bt_server_node])
