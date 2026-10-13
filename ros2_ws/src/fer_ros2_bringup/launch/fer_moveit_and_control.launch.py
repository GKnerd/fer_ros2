"""
Hardware + MoveIt.

Includes:
  - fer_<hardware>_ros2_control.launch.py  (real robot or MuJoCo sim + ros2_control)
  - fer_move_group.launch.py               (move_group + RViz, from fer_moveit)

RViz is owned by fer_move_group.launch.py, which builds the MoveIt config (SRDF,
kinematics) and hands it to the RViz node so the MotionPlanning display works.

Declares every argument of the included files and forwards them; the defaults here decide.
`hand` goes to the hardware launch as `hand` and to move_group as `load_gripper`;
use_sim_time follows the hardware. No controller is activated.
"""
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

HARDWARE = ('real', 'mujoco')

# Upstream fer.urdf.xacro arguments of fer_real_ros2_control.launch.py.
REAL_DESCRIPTION_ARGUMENTS = (
    'robot_type', 'arm_prefix', 'no_prefix', 'xyz_ee', 'rpy_ee', 'tcp_xyz', 'tcp_rpy',
    'safety_distance', 'with_sc', 'xyz', 'rpy',
)


def launch_setup(context, *args, **kwargs):

    # Launch Args
    hardware = LaunchConfiguration('hardware').perform(context)
    if hardware not in HARDWARE:
        raise RuntimeError(f"hardware must be one of {HARDWARE}, got '{hardware}'")
    use_sim_time = str(hardware == 'mujoco').lower()
    log_level = LaunchConfiguration('log_level').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    hand = LaunchConfiguration('hand').perform(context)
    ee_id = LaunchConfiguration('ee_id').perform(context)
    arm_control_type = LaunchConfiguration('arm_control_type').perform(context)
    hand_control_type = LaunchConfiguration('hand_control_type').perform(context)

    hardware_arguments = {
        'use_rviz': 'false',
        'log_level': log_level,
        'hand': hand,
    }
    if hardware == 'real':
        hardware_arguments['robot_ip'] = LaunchConfiguration('robot_ip').perform(context)
        hardware_arguments['ee_id'] = ee_id
        for name in REAL_DESCRIPTION_ARGUMENTS:
            hardware_arguments[name] = LaunchConfiguration(name).perform(context)
    else:
        hardware_arguments['arm_control_type'] = arm_control_type
        hardware_arguments['hand_control_type'] = hand_control_type
        hardware_arguments['scene'] = LaunchConfiguration('scene').perform(context)

    # Real robot or MuJoCo sim + ros2_control
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fer_ros2_bringup'),
                'launch',
                f'fer_{hardware}_ros2_control.launch.py',
            ])
        ),
        launch_arguments=hardware_arguments.items(),
    )

    # Forward the controller-type args so move_group routes trajectories to
    # the controllers of the selected hardware.
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fer_moveit'),
                'launch',
                'fer_move_group.launch.py',
            ])
        ),
        launch_arguments={
            'hardware': hardware,
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'log_level': log_level,
            'namespace': namespace,
            'load_gripper': hand,
            'ee_id': ee_id,
            'arm_control_type': arm_control_type,
            'hand_control_type': hand_control_type,
        }.items()
    )

    return [hardware_launch, moveit_launch]


def generate_launch_description():
    return LaunchDescription(
        generate_declared_arguments() + [OpaqueFunction(function=launch_setup)])


def generate_declared_arguments() -> List[DeclareLaunchArgument]:

    return [
        DeclareLaunchArgument(
            'hardware',
            default_value='mujoco',
            choices=list(HARDWARE),
            description="Hardware backend: 'real' (FER via libfranka) or 'mujoco' (simulation)."
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='',
            description='IP address or hostname of the robot. Required for hardware:=real.'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='warn',
            description="Level of logging for the ros2_nodes. Possible args ('debug', 'info', "
                        "'warn', 'error', 'fatal')."
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Spawn RViz with the MoveIt MotionPlanning display.'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace of move_group.'
        ),
        DeclareLaunchArgument(
            'arm_control_type',
            default_value='effort',
            description="Arm trajectory controller MoveIt uses: 'effort' or 'position' (sim), "
                        "'effort', 'velocity' or 'position' (real)."
        ),
        DeclareLaunchArgument(
            'hand_control_type',
            default_value='position',
            description="Sim gripper controller MoveIt uses: 'position' or 'effort'. "
                        'Ignored for hardware:=real.'
        ),
        DeclareLaunchArgument(
            'scene',
            default_value=PathJoinSubstitution(
                [FindPackageShare('fer_ros2_bringup'), 'scenes', 'base_world.xml']),
            description='MuJoCo scene (MJCF) the robot is inserted into. '
                        'Ignored for hardware:=real.'
        ),
        # Description
        DeclareLaunchArgument(
            'hand',
            default_value='true',
            description="Mount the hand at the flange: 'true' or 'false'."
        ),
        DeclareLaunchArgument(
            'ee_id',
            default_value='franka_hand',
            description="End-effector at the flange (MoveIt; real hardware): 'none', "
                        "'franka_hand', 'cobot_pump'."
        ),
        DeclareLaunchArgument(
            'robot_type',
            default_value='fer',
            description='The ID of the robot model to be used. Ignored for hardware:=mujoco.'
        ),
        DeclareLaunchArgument(
            'arm_prefix',
            default_value='',
            description='The prefix of the robot. Ignored for hardware:=mujoco.'
        ),
        DeclareLaunchArgument(
            'no_prefix',
            default_value='false',
            description="Don't print a prefix for joints, visuals, and links. "
                        'Ignored for hardware:=mujoco.'
        ),
        DeclareLaunchArgument(
            'xyz_ee',
            default_value='0 0 0',
            description='Position offset between ee and parent frame. '
                        'Ignored for hardware:=mujoco.'
        ),
        DeclareLaunchArgument(
            'rpy_ee',
            default_value='0 0 -0.7853981633974483',
            description='Rotation offset between ee and parent frame (default -pi/4 about z). '
                        'Ignored for hardware:=mujoco.'
        ),
        DeclareLaunchArgument(
            'tcp_xyz',
            default_value='0 0 0.1034',
            description='Position offset between ee frame and tcp frame. '
                        'Ignored for hardware:=mujoco.'
        ),
        DeclareLaunchArgument(
            'tcp_rpy',
            default_value='0 0 0',
            description='Rotation offset between ee frame and tcp frame. '
                        'Ignored for hardware:=mujoco.'
        ),
        DeclareLaunchArgument(
            'safety_distance',
            default_value='0.03',
            description='Safety distance for the collision capsules. Ignored for hardware:=mujoco.'
        ),
        DeclareLaunchArgument(
            'with_sc',
            default_value='false',
            description='Should self-collision be enabled? Ignored for hardware:=mujoco.'
        ),
        DeclareLaunchArgument(
            'xyz',
            default_value='0 0 0',
            description='Position offset with respect to the world. Ignored for hardware:=mujoco.'
        ),
        DeclareLaunchArgument(
            'rpy',
            default_value='0 0 0',
            description='Rotation offset with respect to the world. Ignored for hardware:=mujoco.'
        ),
    ]
