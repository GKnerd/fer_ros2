"""Grasp planner: /grasp/candidates and /place/candidates."""
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs) -> List[Node]:

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    log_level = LaunchConfiguration('log_level').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    catalog_file = LaunchConfiguration('catalog_file').perform(context)

    return [
        Node(
            package='fer_grasp_planner',
            executable='grasp_planner_server',
            name='fer_grasp_planner',
            output='both',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[params_file,
                        {'use_sim_time': use_sim_time, 'catalog_file': catalog_file}],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        generate_declared_arguments() + [OpaqueFunction(function=launch_setup)])


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='If true, use the simulated clock.'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='Node log level (debug|info|warn|error|fatal).'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fer_grasp_planner'), 'config', 'grasp_planner.yaml']),
            description='Grasp planner parameters.'),
        DeclareLaunchArgument(
            'catalog_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fer_grasp_planner'), 'config', 'object_catalog.yaml']),
            description='Grasp force per object class.'),
    ]
