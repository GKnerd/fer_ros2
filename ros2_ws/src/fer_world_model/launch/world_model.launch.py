"""World model server, optionally with mock perception."""
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PERCEPTION = ('none', 'mock')


def launch_setup(context, *args, **kwargs) -> List[Node]:

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    log_level = LaunchConfiguration('log_level').perform(context)
    perception = LaunchConfiguration('perception').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    mock_objects_file = LaunchConfiguration('mock_objects_file').perform(context)
    fixtures_file = LaunchConfiguration('fixtures_file').perform(context)

    overrides = {'use_sim_time': use_sim_time, 'fixtures_file': fixtures_file}
    if perception == 'mock':
        overrides['detection_source'] = 'mock_perception'

    nodes = [
        Node(
            package='fer_world_model',
            executable='world_model_server',
            name='fer_world_model',
            output='both',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[params_file, overrides],
        )
    ]
    if perception == 'mock':
        nodes.append(Node(
            package='fer_world_model',
            executable='mock_perception',
            name='fer_mock_perception',
            output='both',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time, 'objects_file': mock_objects_file}],
        ))
    return nodes


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
            'perception', default_value='none', choices=list(PERCEPTION),
            description="'mock' also starts mock_perception with mock_objects_file."),
        DeclareLaunchArgument(
            'mock_objects_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fer_world_model'), 'config', 'mock_objects.yaml']),
            description='Detections published by mock_perception (perception:=mock).'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fer_world_model'), 'config', 'world_model.yaml']),
            description='World model parameters.'),
        DeclareLaunchArgument(
            'fixtures_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fer_world_model'), 'config', 'fixtures.yaml']),
            description="Fixed objects (the table the FER stands on); '' for none."),
    ]
