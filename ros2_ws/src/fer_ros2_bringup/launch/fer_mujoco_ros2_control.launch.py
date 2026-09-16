#!/usr/bin/env python3
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # Launch Args
    use_sim_time = True
    log_level = LaunchConfiguration("log_level")
    use_rviz = LaunchConfiguration("use_rviz")
    hand_control_type = LaunchConfiguration("hand_control_type")
    arm_control_type = LaunchConfiguration("arm_control_type")
    hand = LaunchConfiguration("hand")
    scene = LaunchConfiguration("scene")

    # Package Shares
    fer_ros2_bringup_share = FindPackageShare("fer_ros2_bringup")

    # Controllers
    controller_files = ["fer_controllers.yaml", "fer_controllers_gripper.yaml"]
    param_files = [
        PathJoinSubstitution([fer_ros2_bringup_share, "config", f]) for f in controller_files
    ]

    # FER description
    fer_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([fer_ros2_bringup_share, "urdf", "fer.urdf.xacro"]),
            " hardware:=mujoco",
            " mujoco_control_type:=", arm_control_type,
            " hand_control_type:=", hand_control_type,
            " hand:=", hand,
        ]
    )
    robot_description_str = fer_description_content.perform(context)
    robot_description = {"robot_description": ParameterValue(value=robot_description_str, value_type=str)}

    # Robot State Publisher
    robot_state_pub = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # URDF to MJCF conversion
    urdf_to_mjcf_conversion = Node(
            package="mujoco_ros2_control",
            executable="robot_description_to_mjcf.sh",
            output="both",
            emulate_tty=True,
            arguments=[
                "--robot_description", robot_description_str,
                "--scene", scene,
                "--publish_topic", "/mujoco_robot_description",
            ],
        )

    # MuJoCo ROS2 Control
    mjc_ros2_control = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        emulate_tty=True,
        output="both",
        parameters=[{"use_sim_time": use_sim_time}] + param_files,
        on_exit=Shutdown(),
        remappings=[("~/robot_description", "/robot_description")],
    )

    # Controllers
    # Taken from https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/blob/ros2/cartesian_controller_simulation/launch/simulation.launch.py (last access: 23.03.2026)
    def controller_spawner(names, *args):
        arguments = [*names, *args]
        for param_file in param_files:
            arguments += ["--param-file", param_file]
        return Node(
            package="controller_manager",
            executable="spawner",
            output="both",
            arguments=arguments + ["--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}]
        )

    # Active on start: broadcasters only
    active_spawner = controller_spawner(["joint_state_broadcaster"])

    # Inactive controllers; switch with `ros2 control switch_controllers`
    inactive_list = [
        "effort_trajectory_controller",
        "position_trajectory_controller",
        "gripper_position_controller",
        "gripper_effort_controller",
    ]
    inactive_spawner = controller_spawner(inactive_list, "--inactive")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="mjc_rviz2",
        condition=IfCondition(use_rviz),
        output="both",
        arguments=["-d", PathJoinSubstitution(
            [fer_ros2_bringup_share, "rviz", "fer_mujoco.rviz"])],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    return [robot_state_pub, urdf_to_mjcf_conversion, mjc_ros2_control, rviz, active_spawner, inactive_spawner]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(generate_declared_arguments() + [OpaqueFunction(function=launch_setup)])


def generate_declared_arguments() -> List[DeclareLaunchArgument]:

    return [
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="Level of logging for the ros2_nodes. Possible args ('debug', 'info', 'warn', 'error', 'fatal')."
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Use rviz2 or not. Defaults to true."
        ),
        DeclareLaunchArgument(
            "arm_control_type",
            default_value="effort",
            description="What interface to spawn the fer arm in the simulation with. Supported are: 'position', 'effort'"
        ),
        DeclareLaunchArgument(
            "hand",
            default_value="true",
            description="Whether to spawn the hand or not."
        ),
        DeclareLaunchArgument(
            "hand_control_type",
            default_value="position",
            description="What interface to spawn the fer gripper in the simulation with. Supported are: 'position', 'effort'"
        ),
        DeclareLaunchArgument(
            "scene",
            default_value=PathJoinSubstitution(
                [FindPackageShare("fer_ros2_bringup"), "scenes", "base_world.xml"]),
            description="MuJoCo scene (MJCF) the robot is inserted into."
        ),
    ]
