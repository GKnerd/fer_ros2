"""Runs test_move_group against a real move_group with the FER configuration."""
import os
import unittest

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest


@pytest.mark.launch_test
def generate_test_description():
    """Start move_group and the gtest binary in their own DDS domain."""
    gtest = launch_testing.actions.GTest(
        path=LaunchConfiguration('test_binary'), timeout=280.0, output='screen')
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('fer_moveit_config'), 'launch', 'fer_moveit_launch.py'])),
        launch_arguments={
            'hardware': 'mujoco',
            'use_sim_time': 'false',
            'use_rviz': 'false',
            'log_level': 'warn',
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument('test_binary', description='Path to test_move_group.'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(1 + os.getpid() % 100)),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        move_group,
        gtest,
        launch_testing.actions.ReadyToTest(),
    ]), {'gtest': gtest}


class TestMoveGroupGTest(unittest.TestCase):
    """Wait for the gtest binary."""

    def test_gtest_finishes(self, proc_info, gtest):
        """Give the gtest binary time to run all tests."""
        proc_info.assertWaitForShutdown(process=gtest, timeout=300)


@launch_testing.post_shutdown_test()
class TestMoveGroupGTestResult(unittest.TestCase):
    """Check the gtest binary's exit code."""

    def test_gtest_passed(self, proc_info, gtest):
        """All gtests passed."""
        launch_testing.asserts.assertExitCodes(proc_info, process=gtest)
