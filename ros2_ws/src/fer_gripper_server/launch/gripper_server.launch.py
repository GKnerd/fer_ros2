"""Gripper server for the real Franka Hand or the MuJoCo gripper."""
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
        [FindPackageShare('fer_gripper_server'), 'config', f'gripper_{hardware}.yaml']
    ).perform(context)

    return [
        Node(
            package='fer_gripper_server',
            executable='gripper_server',
            name='fer_gripper_server',
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
            description="'real' (Franka Hand via franka_gripper) or 'mujoco'."),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='Node log level (debug|info|warn|error|fatal).'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description="Parameters; '' selects config/gripper_<hardware>.yaml."),
    ]
