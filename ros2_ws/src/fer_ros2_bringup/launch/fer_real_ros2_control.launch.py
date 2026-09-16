from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # Launch Args
    log_level = LaunchConfiguration("log_level")
    use_rviz = LaunchConfiguration("use_rviz")
    robot_ip = LaunchConfiguration("robot_ip")
    hand = LaunchConfiguration("hand")

    if not robot_ip.perform(context):
        raise RuntimeError("robot_ip is empty. Pass robot_ip:=<FCI address> for hardware:=real.")

    # xacro args passed through to upstream fer.urdf.xacro
    xacro_args = [
        "robot_type", "arm_prefix", "no_prefix", "hand", "ee_id", "xyz_ee", "rpy_ee",
        "tcp_xyz", "tcp_rpy", "safety_distance", "with_sc", "robot_ip", "xyz", "rpy",
    ]

    # Package Share
    fer_ros2_bringup_share = FindPackageShare("fer_ros2_bringup")

    # Controllers
    controller_files = ["fer_controllers.yaml", "fer_controllers_real.yaml"]
    param_files = [
        PathJoinSubstitution([fer_ros2_bringup_share, "config", f]) for f in controller_files
    ]

    # FER description
    fer_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([fer_ros2_bringup_share, "urdf", "fer.urdf.xacro"]),
            " hardware:=real",
        ]
        + [item for arg in xacro_args for item in (f" {arg}:='", LaunchConfiguration(arg), "'")]
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

    # Merges the arm (fer/joint_states) and gripper (fer_gripper/joint_states) states into /joint_states
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

    # ROS2 Control: robot_description arrives on the topic from robot_state_publisher
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=param_files,
        remappings=[("~/robot_description", "/robot_description")],
        output="both",
        arguments=["--ros-args", "--log-level", log_level]
    )

    def controller_spawner(names, *args):
        arguments = [*names, *args]
        for param_file in param_files:
            arguments += ["--param-file", param_file]
        return Node(
            package="controller_manager",
            executable="spawner",
            output="both",
            arguments=arguments + ["--ros-args", "--log-level", log_level],
        )

    # Active on start: broadcasters only.
    joint_state_broadcaster_spawner = controller_spawner(
        ["joint_state_broadcaster"],
        "--controller-ros-args", "-r joint_states:=fer/joint_states",
    )
    robot_state_broadcaster_spawner = controller_spawner(["franka_robot_state_broadcaster"])

    # Loaded but inactive: only one motion controller may drive the joints at a time.
    # Switch to it at runtime with `ros2 control switch_controllers`.
    inactive_controllers = [
        "effort_trajectory_controller",
        "velocity_trajectory_controller",
        "position_trajectory_controller",
    ]
    inactive_spawner = controller_spawner(inactive_controllers, "--inactive")

    # Same node as franka_gripper/launch/gripper.launch.py; the whole bringup
    # shuts down if the gripper connection fails.
    arm_id = LaunchConfiguration("robot_type").perform(context)
    gripper_node = Node(
        package="franka_gripper",
        executable="franka_gripper_node",
        name=f"{arm_id}_gripper",
        output="both",
        parameters=[
            {
                "robot_ip": robot_ip,
                "joint_names": [f"{arm_id}_finger_joint1", f"{arm_id}_finger_joint2"],
            },
            PathJoinSubstitution(
                [FindPackageShare("franka_gripper"), "config", "franka_gripper_node.yaml"]
            ),
        ],
        condition=IfCondition(hand),
        on_exit=Shutdown(),
    )

    rviz_config_file = PathJoinSubstitution(
        [fer_ros2_bringup_share, "rviz", "fer_real.rviz"]
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
        joint_state_broadcaster_spawner,
        robot_state_broadcaster_spawner,
        inactive_spawner,
        gripper_node,
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
            "robot_ip",
            default_value="",
            description="IP address or hostname of the robot. Required."
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
