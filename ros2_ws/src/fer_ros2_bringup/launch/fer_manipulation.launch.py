"""Hardware + MoveIt + the FER manipulation servers + the behavior tree server.

Includes:
  - fer_moveit.launch.py                (hardware, ros2_control, move_group, RViz)
  - world_model.launch.py               (world model, mock perception with perception:=mock)
  - grasp_planner.launch.py             (/grasp/candidates, /place/candidates)
  - fer_moveit_motion_server.launch.py  (/motion/*)
  - gripper_server.launch.py            (/gripper/*)
  - fer_bt_server.launch.py             (/execute_tree)

The hardware launch loads the motion and gripper controllers inactive. Once that spawner
exits, this launch activates <arm_control_type>_trajectory_controller and, for
hardware:=mujoco, gripper_<hand_control_type>_controller. The gripper server starts after
the switch, because a gripper controller only offers its action while active.
"""
from typing import Dict, List

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
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
    if hardware not in HARDWARE:
        raise RuntimeError(f"hardware must be one of {HARDWARE}, got '{hardware}'")
    use_sim_time = str(hardware == "mujoco").lower()
    log_level = LaunchConfiguration("log_level")
    server_log_level = LaunchConfiguration("server_log_level")
    arm_control_type = LaunchConfiguration("arm_control_type").perform(context)
    hand_control_type = LaunchConfiguration("hand_control_type").perform(context)

    # Hardware + MoveIt
    moveit_launch = include("fer_ros2_bringup", "fer_moveit.launch.py", {
        "hardware": hardware,
        "robot_ip": LaunchConfiguration("robot_ip"),
        "log_level": log_level,
        "use_rviz": LaunchConfiguration("use_rviz"),
        "arm_control_type": arm_control_type,
        "hand_control_type": hand_control_type,
        "scene": LaunchConfiguration("scene"),
    })

    world_model_launch = include("fer_world_model", "world_model.launch.py", {
        "use_sim_time": use_sim_time,
        "log_level": server_log_level,
        "perception": LaunchConfiguration("perception"),
        "mock_objects_file": LaunchConfiguration("mock_objects_file"),
        "params_file": LaunchConfiguration("world_model_params_file"),
        "fixtures_file": LaunchConfiguration("fixtures_file"),
    })

    grasp_planner_launch = include("fer_grasp_planner", "grasp_planner.launch.py", {
        "use_sim_time": use_sim_time,
        "log_level": server_log_level,
        "params_file": LaunchConfiguration("grasp_planner_params_file"),
        "catalog_file": LaunchConfiguration("catalog_file"),
    })

    motion_server_launch = include("fer_moveit_config", "fer_moveit_motion_server.launch.py", {
        "hardware": hardware,
        "log_level": server_log_level,
        "params_file": LaunchConfiguration("motion_server_params_file"),
    })

    gripper_server_launch = include("fer_gripper_server", "gripper_server.launch.py", {
        "hardware": hardware,
        "log_level": server_log_level,
        "params_file": LaunchConfiguration("gripper_params_file"),
    })

    bt_launch = include("fer_behavior_trees", "fer_bt_server.launch.py", {
        "use_sim_time": use_sim_time,
        "log_level": server_log_level,
    })

    # Controller activation, after the hardware launch has loaded them inactive
    controllers = [f"{arm_control_type}_trajectory_controller"]
    if hardware == "mujoco":
        controllers.append(f"gripper_{hand_control_type}_controller")
    activate_controllers = ExecuteProcess(
        cmd=["ros2", "control", "switch_controllers", "--strict", "--activate", *controllers],
        output="both",
    )

    def after_inactive_spawner(event, context):
        if "--inactive" in event.cmd and event.returncode == 0:
            return [activate_controllers]
        return None

    def after_activation(event, context):
        if event.returncode == 0:
            return [gripper_server_launch]
        return None

    return [
        moveit_launch,
        world_model_launch,
        grasp_planner_launch,
        motion_server_launch,
        bt_launch,
        RegisterEventHandler(OnProcessExit(on_exit=after_inactive_spawner)),
        RegisterEventHandler(
            OnProcessExit(target_action=activate_controllers, on_exit=after_activation)),
    ]


def generate_launch_description():
    return LaunchDescription(generate_declared_arguments() + [OpaqueFunction(function=launch_setup)])


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
            description="Log level of the hardware and MoveIt nodes ('debug', 'info', 'warn', 'error', 'fatal')."
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Spawn RViz with the MoveIt MotionPlanning display."
        ),
        DeclareLaunchArgument(
            "arm_control_type",
            default_value="effort",
            description="Arm trajectory controller to activate and route to: 'effort' or 'position' (sim), "
                        "'effort', 'velocity' or 'position' (real)."
        ),
        DeclareLaunchArgument(
            "hand_control_type",
            default_value="effort",
            description="Sim gripper controller to activate: 'position' or 'effort'; must match the "
                        "controller in gripper_params_file. Ignored for hardware:=real."
        ),
        DeclareLaunchArgument(
            "scene",
            default_value=share("fer_ros2_bringup", "scenes", "pick_place_world.xml"),
            description="MuJoCo scene (MJCF) the robot is inserted into. Ignored for hardware:=real."
        ),
        # Manipulation servers
        DeclareLaunchArgument(
            "server_log_level",
            default_value="info",
            description="Log level of the manipulation and BT servers ('debug', 'info', 'warn', 'error', 'fatal')."
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
    ]
