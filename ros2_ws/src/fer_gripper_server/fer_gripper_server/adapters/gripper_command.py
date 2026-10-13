"""
Gripper adapter for control_msgs/GripperCommand (MuJoCo `gripper_effort_controller`).

A grasp closes fully with `max_effort` = force and stalls on the object; the controller
must allow stalling so it keeps squeezing. The width is twice the finger joint position.
"""
from __future__ import annotations

from typing import Callable

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from fer_gripper_server.adapters.actions import run_goal, WidthMonitor
from fer_gripper_server.core.grasp import GripperError
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GripperCommandGripper:

    def __init__(self, node: Node, callback_group: CallbackGroup) -> None:
        node.declare_parameter('controller', 'gripper_effort_controller')
        node.declare_parameter('joint', 'fer_finger_joint1')
        node.declare_parameter('joint_states_topic', '/joint_states')
        node.declare_parameter('move_force', 20.0)
        node.declare_parameter('action_timeout', 10.0)
        node.declare_parameter('state_timeout', 1.0)

        self._controller = node.get_parameter('controller').value
        self._joint = node.get_parameter('joint').value
        self._move_force = float(node.get_parameter('move_force').value)
        self._action_timeout = float(node.get_parameter('action_timeout').value)
        self._state_timeout = float(node.get_parameter('state_timeout').value)

        self._action_name = f'/{self._controller}/gripper_cmd'
        self._action = ActionClient(
            node, GripperCommand, self._action_name, callback_group=callback_group)
        self._width = WidthMonitor()
        node.create_subscription(
            JointState, node.get_parameter('joint_states_topic').value, self._on_joint_states,
            10, callback_group=callback_group)

    def prepare(self, timeout: float) -> None:
        """
        Wait for the gripper controller's action.

        Raises GripperError if it does not appear within `timeout`.
        """
        if not self._action.wait_for_server(timeout_sec=timeout):
            raise GripperError(
                f"'{self._action_name}' not available; is '{self._controller}' active?")

    def move(self, width: float, interrupted: Callable[[], bool]) -> None:
        self._command(width / 2.0, self._move_force, interrupted)

    def grasp(
        self, width: float, force: float, tolerance: float, interrupted: Callable[[], bool]
    ) -> None:
        self._command(0.0, force, interrupted)

    def width(self) -> float | None:
        return self._width.latest()

    def fresh_width(self) -> float:
        return self._width.fresh(self._state_timeout)

    def holding(self) -> bool | None:
        return None

    def _command(
        self, position: float, max_effort: float, interrupted: Callable[[], bool]
    ) -> None:
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        wrapped = run_goal(
            self._action, self._action_name, goal, self._action_timeout, interrupted)
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise GripperError(f"'{self._action_name}' ended with status {wrapped.status}")

    def _on_joint_states(self, msg: JointState) -> None:
        if self._joint in msg.name:
            self._width.update(2.0 * msg.position[msg.name.index(self._joint)])
