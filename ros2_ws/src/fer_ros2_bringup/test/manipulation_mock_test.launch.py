"""Starts fer_manipulation.launch.py on mock hardware and waits for every server."""
import os
import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing.actions
import pytest

# Logged once a server has finished its startup. The gripper server starts only after
# switch_controllers activated the arm and gripper controllers.
READY = (
    'world model up',
    'grasp planner ready',
    'Motion server ready',
    'gripper server ready',
)


@pytest.mark.launch_test
def generate_test_description():
    """Start the manipulation level with hardware:=mock in its own DDS domain."""
    manipulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('fer_ros2_bringup'), 'launch', 'fer_manipulation.launch.py'])),
        launch_arguments={
            'hardware': 'mock',
            'arm_control_type': 'position',
            'hand_control_type': 'position',
            'use_rviz': 'false',
            'log_level': 'warn',
            'server_log_level': 'info',
        }.items())
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(1 + os.getpid() % 100)),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        manipulation,
        launch_testing.actions.ReadyToTest(),
    ])


class TestManipulationMock(unittest.TestCase):
    """Every server of the manipulation level comes up."""

    def test_servers_ready(self, proc_output):
        """Each server logs its ready line."""
        for line in READY:
            with self.subTest(server=line):
                proc_output.assertWaitFor(line, timeout=120)
