"""Hardware + MoveIt + fer_skills.

Includes:
  - fer_moveit.launch.py   (with use_rviz:=false; we own the view)
  - fer_skills.launch.py   (skill server with its own RViz)
"""
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

HARDWARE = ("real", "mujoco")


def launch_setup(context, *args, **kwargs):

    # Launch Args
    hardware = LaunchConfiguration("hardware").perform(context)
    if hardware not in HARDWARE:
        raise RuntimeError(f"hardware must be one of {HARDWARE}, got '{hardware}'")
    use_sim_time = str(hardware == "mujoco").lower()
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    log_level = LaunchConfiguration("log_level")
    robot_ip = LaunchConfiguration("robot_ip")
    arm_control_type = LaunchConfiguration("arm_control_type")
    hand_control_type = LaunchConfiguration("hand_control_type")
    arm_group = LaunchConfiguration("arm_group")
    load_gripper = LaunchConfiguration("load_gripper")
    ee_id = LaunchConfiguration("ee_id")

    # Hardware + MoveIt: don't spawn its RViz
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("fer_ros2_bringup"),
                "launch",
                "fer_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "hardware": hardware,
            "robot_ip": robot_ip,
            "use_rviz": "false",
            "log_level": log_level,
            "arm_control_type": arm_control_type,
            "hand_control_type": hand_control_type,
        }.items(),
    )

    # Skill server with its own RViz.
    skills_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("fer_skills"),
                "launch",
                "fer_skills.launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz": use_rviz,
            "log_level": log_level,
            "arm_group": arm_group,
            "load_gripper": load_gripper,
            "ee_id": ee_id,
            "robot_ip": robot_ip,
        }.items(),
    )

    return [moveit_launch, skills_launch]


def generate_launch_description():
    return LaunchDescription(generate_declared_arguments() + [OpaqueFunction(function=launch_setup)])


def generate_declared_arguments() -> List[DeclareLaunchArgument]:

    return [
        DeclareLaunchArgument(
            "hardware",
            default_value="mujoco",
            choices=list(HARDWARE),
            description="Hardware backend: 'real' (FER via libfranka) or 'mujoco' (simulation)."
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="",
            description="IP address or hostname of the robot. Required for hardware:=real."
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Spawn this launch's own RViz. Set to false when included by a higher-level launch."
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="Level of logging for the ros2_nodes. Possible args ('debug', 'info', 'warn', 'error', 'fatal')."
        ),
        DeclareLaunchArgument(
            "arm_control_type",
            default_value="effort",
            description="Arm trajectory controller MoveIt uses: 'effort' or 'position' (sim), "
                        "'effort', 'velocity' or 'position' (real)."
        ),
        DeclareLaunchArgument(
            "hand_control_type",
            default_value="position",
            description="Sim gripper controller MoveIt uses: 'position' or 'effort'. Ignored for hardware:=real."
        ),
        DeclareLaunchArgument(
            "arm_group",
            default_value="fer_arm",
            description="MoveIt planning group used by fer_skills."
        ),
        DeclareLaunchArgument(
            "load_gripper",
            default_value="true",
            description="Whether the URDF includes the Franka gripper."
        ),
        DeclareLaunchArgument(
            "ee_id",
            default_value="franka_hand",
            description="End-effector id (matches franka_description macros)."
        ),
    ]
