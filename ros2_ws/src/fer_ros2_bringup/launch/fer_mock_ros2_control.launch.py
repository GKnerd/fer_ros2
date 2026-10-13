"""FER on mock_components/GenericSystem (no physics) + ros2_control, controllers loaded."""
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs) -> List[Node]:

    # Launch Args
    log_level = LaunchConfiguration('log_level').perform(context)
    hand = LaunchConfiguration('hand').perform(context)

    # Package Share
    fer_ros2_bringup_share = FindPackageShare('fer_ros2_bringup')

    # Controllers
    controller_files = ['fer_controllers.yaml', 'fer_controllers_gripper.yaml']
    param_files = [
        PathJoinSubstitution([fer_ros2_bringup_share, 'config', f]) for f in controller_files
    ]

    # FER description
    fer_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([fer_ros2_bringup_share, 'urdf', 'fer.urdf.xacro']),
            ' hardware:=mock',
            ' hand:=', hand,
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(
            value=fer_description_content.perform(context), value_type=str)
    }

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[robot_description],
    )

    ros2_control = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=param_files,
        on_exit=Shutdown(),
        remappings=[('~/robot_description', '/robot_description')],
    )

    def controller_spawner(names, *args):
        arguments = [*names, *args]
        for param_file in param_files:
            arguments += ['--param-file', param_file]
        return Node(
            package='controller_manager',
            executable='spawner',
            output='both',
            arguments=arguments + ['--ros-args', '--log-level', log_level],
        )

    # Active on start: broadcasters only
    active_spawner = controller_spawner(['joint_state_broadcaster'])

    # Inactive; only the position controllers move a mock joint
    inactive_spawner = controller_spawner(
        ['position_trajectory_controller', 'gripper_position_controller'], '--inactive')

    return [robot_state_pub, ros2_control, active_spawner, inactive_spawner]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        generate_declared_arguments() + [OpaqueFunction(function=launch_setup)])


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='Node log level (debug|info|warn|error|fatal).'),
        DeclareLaunchArgument(
            'hand', default_value='true',
            description="Mount the hand at the flange: 'true' or 'false'."),
    ]
