#  Copyright (c) 2024 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# This file is an adapted version of
# https://github.com/ros-planning/moveit_resources/blob/ca3f7930c630581b5504f3b22c40b4f82ee6369d/panda_moveit_config/launch/demo.launch.py

import os
import yaml
from typing import List


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue





def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


# Maps the high-level launch arg (effort/position) to the controller name
# defined in moveit_controllers.yaml. Update if you rename controllers.
ARM_CONTROLLER_NAME = {
    "effort":   "joint_effort_traj_controller",
    "position": "joint_pos_traj_controller",
}
HAND_CONTROLLER_NAME = {
    "effort":   "gripper_effort_controller",
    "position": "gripper_position_controller",
}


def select_default_controllers(controllers_yaml, arm_type, hand_type):
    """Mutate the moveit_controllers dict so the chosen controllers are the defaults.

    Keeps moveit_controllers.yaml as the single source of truth for controller
    *definitions* (type, action_ns, joints). The `default` flags are derived
    from the same launch args that drive the URDF and ros2_control choices.
    """
    chosen = {
        ARM_CONTROLLER_NAME[arm_type],
        HAND_CONTROLLER_NAME[hand_type],
    }
    for name in controllers_yaml.get("controller_names", []):
        entry = controllers_yaml.get(name)
        if isinstance(entry, dict):
            entry["default"] = name in chosen
    return controllers_yaml


def moveit_launch_setup(context, *args, **kwargs):
    # Launch Config
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    db = LaunchConfiguration("db")
    log_level = LaunchConfiguration("log_level")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    namespace = LaunchConfiguration("namespace")
    load_gripper = LaunchConfiguration("load_gripper")
    ee_id = LaunchConfiguration("ee_id")
    arm_control_type = LaunchConfiguration("arm_control_type").perform(context)
    hand_control_type = LaunchConfiguration("hand_control_type").perform(context)


    # Package Shares
    franka_description_share    = get_package_share_directory("franka_description")
    fer_moveit_config_share  = get_package_share_directory("fer_moveit_config")


    # FER description
    fer_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([franka_description_share, "robots", "fer", "fer.urdf.xacro"]),
        " hand:=", load_gripper,
        " robot_ip:=", robot_ip,
        " ee_id:=", ee_id,
        " use_fake_hardware:=", use_fake_hardware,
        " fake_sensor_commands:=", fake_sensor_commands,
        " ros2_control:=false"
    ])
    robot_description = {
        "robot_description": ParameterValue(fer_description.perform(context), value_type=str)
    }

    # FER semantic description
    fer_semantic_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([franka_description_share, "robots", "fer", "fer.srdf.xacro"]),
        " hand:=", load_gripper,
        " ee_id:=", ee_id,
    ])
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(fer_semantic_description.perform(context), value_type=str)
    }

    # Kinematics 
    kinematics_yaml = load_yaml(
        "fer_moveit_config", "config/kinematics.yaml"
    )
    kinematics = {
        "robot_description_kinematics": kinematics_yaml
    }

    # Cartesian Limits
    cartesian_limits_yaml = load_yaml(
        "fer_moveit_config", "config/cartesian_limits.yaml"
    )
    cartesian_limits = {
        "robot_description_cartesian_limits": cartesian_limits_yaml
    }

    # Joint Limits
    joint_limits_yaml = load_yaml(
        "fer_moveit_config", "config/joint_limits.yaml"
    )
    joint_limits = {
        'robot_description_planning': joint_limits_yaml
    }

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
                                ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath'
                                  ],
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "fer_moveit_config", 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)
    

    # Trajectory Execution Functionality
    #
    # The YAML defines every controller MoveIt may need to route to. The
    # `default: true|false` flag in the YAML is overridden here based on the
    # arm_control_type / hand_control_type launch args, so we have ONE source
    # of truth for "which controller is active" — the launch arg — without
    # editing the YAML for every switch.
    moveit_simple_controllers_yaml = load_yaml(
        "fer_moveit_config", 'config/moveit_controllers.yaml'
    )
    select_default_controllers(
        moveit_simple_controllers_yaml, arm_control_type, hand_control_type
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.execution_duration_monitoring': False,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Load the MTC capability so task_.execute(solution) has its action server
    # ('execute_task_solution') available inside move_group.
    move_group_capabilities = {
        'capabilities': 'move_group/ExecuteTaskSolutionCapability',
    }

    # Move Group Action Server
    move_group_node = Node(
        package='moveit_ros_move_group',
        namespace=namespace,
        executable='move_group',
        output='both',
        parameters=[
            {"use_sim_time": use_sim_time},
            robot_description,
            robot_description_semantic,
            kinematics,
            joint_limits,
            cartesian_limits,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities
        ],
        arguments=[ "--ros-args", "--log-level", log_level]
    )

    # RViz
    rviz_base = os.path.join(fer_moveit_config_share, 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit_conf.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='moveit_rviz2',
        output='both',
        arguments=['-d', rviz_full_config,
                   "--ros-args", "--log-level", log_level],
        parameters=[
            {"use_sim_time": use_sim_time},
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics,
        ],
        condition=IfCondition(use_rviz)
    )

    return [move_group_node, rviz_node]

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(generate_declared_arguments() + [OpaqueFunction(function=moveit_launch_setup)])





def generate_declared_arguments() -> List[DeclareLaunchArgument]:

    return [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock"
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Use rviz2 or not. Defaults to true."
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="Level of logging for the ros2_nodes. Possible args ('debug', 'info', 'warn', 'error', 'fatal')."
        ),
        DeclareLaunchArgument(
            "db",
            default_value="False",
            description="Database flag"
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="",
            description='Hostname or IP address of the robot.'
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value='',
            description='Namespace for the robot.'
        ),
        DeclareLaunchArgument(
            "load_gripper",
            default_value='true',
            description='Whether to load the gripper or not (true or false)'
        ),
        DeclareLaunchArgument(
            "ee_id",
            default_value='franka_hand',
            description='The end-effector id to use. Available options: none, franka_hand, cobot_pump'
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value='false',
            description='Use fake hardware'),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value='false',
            description="Fake sensor commands. Only valid when '{}' is true".format("use_fake_hardware")
        ),
        DeclareLaunchArgument(
            "arm_control_type",
            default_value="effort",
            description="Which arm controller is the active default for move_group "
                        "to route trajectories to: 'effort' or 'position'."
        ),
        DeclareLaunchArgument(
            "hand_control_type",
            default_value="position",
            description="Which gripper controller is the active default for move_group: "
                        "'effort' or 'position'."
        ),
    ]
