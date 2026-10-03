from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution([FindPackageShare('fer_behavior_trees'), 'config'])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='fer_behavior_trees',
            executable='fer_bt_server',
            name='fer_bt_server',
            output='both',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[
                PathJoinSubstitution([config, 'fer_bt_server.yaml']),
                PathJoinSubstitution([config, 'poses.yaml']),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
