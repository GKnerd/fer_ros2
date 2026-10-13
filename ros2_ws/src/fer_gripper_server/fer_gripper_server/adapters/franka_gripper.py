"""
Gripper adapter for the Franka Hand through franka_gripper (`franka_msgs`).

Only imported for hardware:=real; `franka_msgs` exists with the real profile only.
The grasp epsilon is the goal's tolerance, so libfranka's verdict matches the server's.
"""
from __future__ import annotations

from typing import Callable

from fer_gripper_server.adapters.actions import run_goal, WidthMonitor
from fer_gripper_server.core.grasp import GripperError
from franka_msgs.action import Grasp, Move
from franka_msgs.msg import GripperState
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node


class FrankaGripper:

    def __init__(self, node: Node, callback_group: CallbackGroup) -> None:
        node.declare_parameter('gripper_namespace', '/fer_gripper')
        node.declare_parameter('speed', 0.1)
        node.declare_parameter('action_timeout', 10.0)
        node.declare_parameter('state_timeout', 1.0)

        namespace = node.get_parameter('gripper_namespace').value
        self._speed = float(node.get_parameter('speed').value)
        self._action_timeout = float(node.get_parameter('action_timeout').value)
        self._state_timeout = float(node.get_parameter('state_timeout').value)

        self._move_name = f'{namespace}/move'
        self._grasp_name = f'{namespace}/grasp'
        self._move = ActionClient(node, Move, self._move_name, callback_group=callback_group)
        self._grasp = ActionClient(node, Grasp, self._grasp_name, callback_group=callback_group)
        self._width = WidthMonitor()
        self._is_grasped: bool | None = None
        node.create_subscription(
            GripperState, f'{namespace}/gripper_state', self._on_state, 10,
            callback_group=callback_group)

    def prepare(self, timeout: float) -> None:
        for client, name in ((self._move, self._move_name), (self._grasp, self._grasp_name)):
            if not client.wait_for_server(timeout_sec=timeout):
                raise GripperError(f"'{name}' not available")

    def move(self, width: float, interrupted: Callable[[], bool]) -> None:
        goal = Move.Goal(width=width, speed=self._speed)
        self._check(run_goal(
            self._move, self._move_name, goal, self._action_timeout, interrupted))

    def grasp(
        self, width: float, force: float, tolerance: float, interrupted: Callable[[], bool]
    ) -> None:
        goal = Grasp.Goal(width=width, speed=self._speed, force=force)
        goal.epsilon.inner = tolerance
        goal.epsilon.outer = tolerance
        self._check(run_goal(
            self._grasp, self._grasp_name, goal, self._action_timeout, interrupted))

    def width(self) -> float | None:
        return self._width.latest()

    def fresh_width(self) -> float:
        return self._width.fresh(self._state_timeout)

    def holding(self) -> bool | None:
        return self._is_grasped

    @staticmethod
    def _check(wrapped) -> None:
        # success=False without an error means "nothing within epsilon"; the server judges
        # that from the width. An error string is a real gripper failure.
        if wrapped.result.error:
            raise GripperError(wrapped.result.error)

    def _on_state(self, msg: GripperState) -> None:
        self._is_grasped = msg.is_grasped
        self._width.update(msg.width)
