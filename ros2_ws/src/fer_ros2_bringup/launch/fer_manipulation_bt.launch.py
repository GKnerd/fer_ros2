"""fer_manipulation.launch.py + the behavior tree server (/execute_tree).

Includes:
  - fer_manipulation.launch.py  (hardware, MoveIt, the manipulation servers)
  - fer_bt_server.launch.py     (/execute_tree)

Declares every argument of fer_manipulation.launch.py and the BT server's parameter files,
and forwards them; the defaults here decide.
"""
from typing import Dict, List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

HARDWARE = ("real", "mujoco")
PERCEPTION = ("none", "mock")


def include(package: str, file: str, arguments: Dict) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", file])
        ),
        launch_arguments=arguments.items(),
    )


def launch_setup(context, *args, **kwargs):

    # Launch Args
    hardware = LaunchConfiguration("hardware").perform(context)
    server_log_level = LaunchConfiguration("server_log_level").perform(context)
    bt_server_params_file = LaunchConfiguration("bt_server_params_file").perform(context)
    poses_file = LaunchConfiguration("poses_file").perform(context)
    arguments = {
        "hardware": hardware,
        "server_log_level": server_log_level,
        **{name: LaunchConfiguration(name).perform(context) for name in (
            "robot_ip", "log_level", "use_rviz", "arm_control_type", "hand_control_type",
            "scene", "perception", "mock_objects_file", "world_model_params_file",
            "fixtures_file", "grasp_planner_params_file", "catalog_file",
            "motion_server_params_file", "gripper_params_file")},
    }

    manipulation_launch = include("fer_ros2_bringup", "fer_manipulation.launch.py", arguments)

    bt_launch = include("fer_behavior_trees", "fer_bt_server.launch.py", {
        "use_sim_time": str(hardware == "mujoco").lower(),
        "log_level": server_log_level,
        "params_file": bt_server_params_file,
        "poses_file": poses_file,
    })

    return [manipulation_launch, bt_launch]


def generate_launch_description():
    return LaunchDescription(
        generate_declared_arguments() + [OpaqueFunction(function=launch_setup)])


def generate_declared_arguments() -> List[DeclareLaunchArgument]:

    def share(package: str, *path: str) -> PathJoinSubstitution:
        return PathJoinSubstitution([FindPackageShare(package), *path])

    return [
        # Hardware + MoveIt
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
            description="Log level of the hardware and MoveIt nodes "
                        "('debug', 'info', 'warn', 'error', 'fatal')."
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Spawn RViz with the MoveIt MotionPlanning display."
        ),
        DeclareLaunchArgument(
            "arm_control_type",
            default_value="effort",
            description="Arm trajectory controller to activate and route to: 'effort' or "
                        "'position' (sim), 'effort', 'velocity' or 'position' (real)."
        ),
        DeclareLaunchArgument(
            "hand_control_type",
            default_value="effort",
            description="Sim gripper controller to activate: 'position' or 'effort'; must match "
                        "the controller in gripper_params_file. Ignored for hardware:=real."
        ),
        DeclareLaunchArgument(
            "scene",
            default_value=share("fer_ros2_bringup", "scenes", "pick_place_world.xml"),
            description="MuJoCo scene (MJCF) the robot is inserted into. "
                        "Ignored for hardware:=real."
        ),
        # Manipulation and BT servers
        DeclareLaunchArgument(
            "server_log_level",
            default_value="info",
            description="Log level of the manipulation and BT servers "
                        "('debug', 'info', 'warn', 'error', 'fatal')."
        ),
        DeclareLaunchArgument(
            "perception",
            default_value="mock",
            choices=list(PERCEPTION),
            description="'mock' also starts mock_perception with mock_objects_file."
        ),
        DeclareLaunchArgument(
            "mock_objects_file",
            default_value=share("fer_world_model", "config", "mock_objects.yaml"),
            description="Detections published by mock_perception (perception:=mock)."
        ),
        DeclareLaunchArgument(
            "world_model_params_file",
            default_value=share("fer_world_model", "config", "world_model.yaml"),
            description="World model parameters."
        ),
        DeclareLaunchArgument(
            "fixtures_file",
            default_value=share("fer_world_model", "config", "fixtures.yaml"),
            description="Fixed objects (the table the FER stands on); '' for none."
        ),
        DeclareLaunchArgument(
            "grasp_planner_params_file",
            default_value=share("fer_grasp_planner", "config", "grasp_planner.yaml"),
            description="Grasp planner parameters."
        ),
        DeclareLaunchArgument(
            "catalog_file",
            default_value=share("fer_grasp_planner", "config", "object_catalog.yaml"),
            description="Grasp force per object class."
        ),
        DeclareLaunchArgument(
            "motion_server_params_file",
            default_value="",
            description="Motion server parameters; '' selects fer_moveit_motion_server.yaml."
        ),
        DeclareLaunchArgument(
            "gripper_params_file",
            default_value="",
            description="Gripper server parameters; '' selects gripper_<hardware>.yaml."
        ),
        DeclareLaunchArgument(
            "bt_server_params_file",
            default_value=share("fer_behavior_trees", "config", "fer_bt_server.yaml"),
            description="BT server parameters (timeouts, default speed_scaling)."
        ),
        DeclareLaunchArgument(
            "poses_file",
            default_value=share("fer_behavior_trees", "config", "poses.yaml"),
            description="Named arm configurations of the trees (home, view)."
        ),
    ]
