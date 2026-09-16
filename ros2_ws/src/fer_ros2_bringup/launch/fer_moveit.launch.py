"""Hardware + MoveIt.

Includes:
  - fer_<hardware>_ros2_control.launch.py  (real robot or MuJoCo sim + ros2_control)
  - fer_moveit_launch.py                   (move_group + RViz)

RViz is owned by fer_moveit_launch.py, which builds the MoveIt config (SRDF,
kinematics) and hands it to the RViz node so the MotionPlanning display works.
The use_rviz arg is forwarded to the included launches.
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
    log_level = LaunchConfiguration("log_level")
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    robot_ip = LaunchConfiguration("robot_ip")
    hand_control_type = LaunchConfiguration("hand_control_type")
    arm_control_type = LaunchConfiguration("arm_control_type")

    hardware_arguments = {
        "use_rviz": "false",
        "log_level": log_level,
    }
    if hardware == "real":
        hardware_arguments["robot_ip"] = robot_ip
    else:
        hardware_arguments["arm_control_type"] = arm_control_type
        hardware_arguments["hand_control_type"] = hand_control_type

    # Real robot or MuJoCo sim + ros2_control
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("fer_ros2_bringup"),
                "launch",
                f"fer_{hardware}_ros2_control.launch.py",
            ])
        ),
        launch_arguments=hardware_arguments.items(),
    )

    # Forward the controller-type args so move_group routes trajectories to
    # the controllers of the selected hardware.
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("fer_moveit_config"),
                "launch",
                "fer_moveit_launch.py",
            ])
        ),
        launch_arguments={
            "hardware": hardware,
            "use_sim_time": use_sim_time,
            "use_rviz": use_rviz,
            "log_level": log_level,
            "robot_ip": robot_ip,
            "arm_control_type": arm_control_type,
            "hand_control_type": hand_control_type,
        }.items()
    )

    return [hardware_launch, moveit_launch]


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
            "log_level",
            default_value="warn",
            description="Level of logging for the ros2_nodes. Possible args ('debug', 'info', 'warn', 'error', 'fatal')."
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Spawn this launch's own RViz. Set to false when included by a higher-level launch."
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
        )
    ]
