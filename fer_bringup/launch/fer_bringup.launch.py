from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):

    # Launch Args
    log_level   = LaunchConfiguration("log_level")
    use_rviz    = LaunchConfiguration("use_rviz")

    # xacro args (mirrors the necessary <xacro:arg> declarations in fer.urdf.xacro)
    robot_type           = LaunchConfiguration("robot_type")
    arm_prefix           = LaunchConfiguration("arm_prefix")
    no_prefix            = LaunchConfiguration("no_prefix")
    hand                 = LaunchConfiguration("hand")
    ee_id                = LaunchConfiguration("ee_id")
    xyz_ee               = LaunchConfiguration("xyz_ee")
    rpy_ee               = LaunchConfiguration("rpy_ee")
    tcp_xyz              = LaunchConfiguration("tcp_xyz")
    tcp_rpy              = LaunchConfiguration("tcp_rpy")
    safety_distance      = LaunchConfiguration("safety_distance")
    with_sc              = LaunchConfiguration("with_sc")
    ros2_control         = LaunchConfiguration("ros2_control")
    robot_ip             = LaunchConfiguration("robot_ip")
    xyz                  = LaunchConfiguration("xyz")
    rpy                  = LaunchConfiguration("rpy")

    # Package Share
    fer_bringup_share = FindPackageShare("fer_bringup")

    # Controllers
    controllers_yaml = PathJoinSubstitution(
        [fer_bringup_share, "config", "controllers.yaml"]
    )

    # FER description
    fer_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([fer_bringup_share, "urdf", "fer.xacro"]),
            " ",
            "robot_type:=",           robot_type,           " ",
            "arm_prefix:=",           arm_prefix,           " ",
            "no_prefix:=",            no_prefix,            " ",
            "hand:=",                 hand,                 " ",
            "ee_id:=",                ee_id,                " ",
            "xyz_ee:=",               xyz_ee,               " ",
            "rpy_ee:=",               rpy_ee,               " ",
            "tcp_xyz:=",              tcp_xyz,              " ",
            "tcp_rpy:=",              tcp_rpy,              " ",
            "safety_distance:=",      safety_distance,      " ",
            "with_sc:=",              with_sc,              " ",
            "ros2_control:=",         ros2_control,         " ",
            "robot_ip:=",             robot_ip,             " ",
            "xyz:=",                  xyz,                  " ",
            "rpy:=",                  rpy,                  " ",
        ]
    )
    robot_description_str = fer_description_content.perform(context)
    robot_description = {"robot_description": ParameterValue(value=robot_description_str, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        arguments=["--ros-args", "--log-level", log_level],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'source_list': ['fer/joint_states', 'fer_gripper/joint_states'],
                'rate': 30
            }
            ],
    )

    # ROS2 Control
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        remappings=[("joint_states", "fer/joint_states")],
        output="both",
        arguments=["--ros-args", "--log-level", log_level]
    )
    
    # One spawner mechanism for broadcasters and motion controllers.
    # --param-file passes controllers.yaml so generate_parameter_library controllers
    # (the trajectory controllers) receive their required params (joints, gains). (TODO item 2)
    def controller_spawner(name, *args):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="both",
            arguments=[name, *args,
                       "--param-file", controllers_yaml,
                       "--ros-args", "--log-level", log_level],
        )

    # Active on start.
    active_controllers = [
        "joint_state_broadcaster",
        "franka_robot_state_broadcaster",
    ]
    active_spawners = [controller_spawner(c) for c in active_controllers]

    # Loaded but inactive: only one motion controller may drive the joints at a time.
    # Switch to it at runtime with `ros2 control switch_controllers`.
    inactive_controllers = [
        "effort_joint_trajectory_controller",
        "vel_joint_trajectory_controller",
    ]
    inactive_spawners = [controller_spawner(c, "--inactive") for c in inactive_controllers]

    gripper_launch_desc = PathJoinSubstitution(
        [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py']
    )
    fer_gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gripper_launch_desc),
            launch_arguments={'robot_ip': robot_ip,}.items(),
            condition=IfCondition(hand)
    )
    
    rviz_config_file = PathJoinSubstitution(
        [fer_bringup_share, "rviz", "visualize_franka.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", log_level],
        parameters=[robot_description],
        condition=IfCondition(use_rviz),
        on_exit=Shutdown(),
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_node,
        ros2_control_node,
        *active_spawners,
        *inactive_spawners,
        fer_gripper_launch,
        rviz_node,
    ]


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
            "robot_type",
            default_value="fer",
            description="The ID of the robot model to be used."
        ),
        DeclareLaunchArgument(
            "arm_prefix",
            default_value="",
            description="The prefix of the robot."
        ),
        DeclareLaunchArgument(
            "no_prefix",
            default_value="false",
            description="Don't print a prefix for joints, visuals, and links."
        ),
        DeclareLaunchArgument(
            "hand",
            default_value="true",
            description="Should an end-effector be mounted at the flange?"
        ),
        DeclareLaunchArgument(
            "ee_id",
            default_value="franka_hand",
            description="Which end-effector would be mounted at the flange?"
        ),
        DeclareLaunchArgument(
            "xyz_ee",
            default_value="0 0 0",
            description="Position offset between ee and parent frame."
        ),
        DeclareLaunchArgument(
            "rpy_ee",
            default_value="0 0 -0.7853981633974483",
            description="Rotation offset between ee and parent frame (default -pi/4 about z)."
        ),
        DeclareLaunchArgument(
            "tcp_xyz",
            default_value="0 0 0.1034",
            description="Position offset between ee frame and tcp frame."
        ),
        DeclareLaunchArgument(
            "tcp_rpy",
            default_value="0 0 0",
            description="Rotation offset between ee frame and tcp frame."
        ),
        DeclareLaunchArgument(
            "safety_distance",
            default_value="0.03",
            description="Safety distance for the collision capsules."
        ),
        DeclareLaunchArgument(
            "with_sc",
            default_value="false",
            description="Should self-collision be enabled?"
        ),
        DeclareLaunchArgument(
            "ros2_control",
            default_value="true",
            description="Is the robot being controlled with ros2_control?"
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="",
            description="IP address or hostname of the robot."
        ),
        DeclareLaunchArgument(
            "xyz",
            default_value="0 0 0",
            description="Position offset with respect to the world."
        ),
        DeclareLaunchArgument(
            "rpy",
            default_value="0 0 0",
            description="Rotation offset with respect to the world."
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        generate_declared_arguments() + [OpaqueFunction(function=launch_setup)]
    )
