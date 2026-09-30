"""MoveIt motion server of the FER platform; move_group comes from fer_moveit_launch.py."""
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

HARDWARE = ('real', 'mujoco')


def launch_setup(context, *args, **kwargs) -> List[Node]:
    hardware = LaunchConfiguration('hardware').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context) or PathJoinSubstitution(
        [FindPackageShare('fer_moveit_config'), 'config', 'fer_moveit_motion_server.yaml']
    ).perform(context)

    return [
        Node(
            package='fer_moveit_config',
            executable='fer_moveit_motion_server',
            name='fer_moveit_motion_server',
            output='both',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[params_file, {'hardware': hardware,
                                      'use_sim_time': hardware == 'mujoco'}],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        generate_declared_arguments() + [OpaqueFunction(function=launch_setup)])


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            'hardware', default_value='mujoco', choices=list(HARDWARE),
            description="'real' (robot mode from franka_robot_state_broadcaster) or 'mujoco'."),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='Node log level (debug|info|warn|error|fatal).'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description="Parameters; '' selects config/fer_moveit_motion_server.yaml."),
    ]
