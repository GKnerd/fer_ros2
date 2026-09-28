"""
Gripper node: serves the gripper part of fer_interfaces.

MoveGripper, Grasp and Release run one at a time: a new goal of any of them ends the
running one with outcome CANCELLED. A grasp is judged from the measured width; the world
model learns GRASPED, FREE and LOST from here. While an object is held, a watch sets it
LOST as soon as the grip is gone.
"""
from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import Callable

from fer_gripper_server.adapters.gripper_command import GripperCommandGripper
from fer_gripper_server.adapters.world_model_client import WorldModelClient, WorldModelError
from fer_gripper_server.core.grasp import (
    force_valid,
    grasp_succeeded,
    Gripper,
    GripperError,
    Interrupted,
    release_confirmed,
    still_holding,
    width_valid,
)
from fer_interfaces.action import Grasp, MoveGripper, Release
from fer_interfaces.msg import Outcome, WorldObject
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class _Held:
    object_id: str
    grasp_width: float
    tolerance: float
    pose_in_hand: PoseStamped


def _outcome(code: int, message: str = '') -> Outcome:
    return Outcome(code=code, message=message)


class GripperServer:

    def __init__(
        self,
        node: Node,
        gripper: Gripper,
        world_model: WorldModelClient,
        callback_group: CallbackGroup,
    ) -> None:
        self._node = node
        self._gripper = gripper
        self._world_model = world_model
        node.declare_parameter('hand_frame', 'fer_hand_tcp')
        node.declare_parameter('base_frame', 'base')
        node.declare_parameter('max_width', 0.08)
        node.declare_parameter('max_force', 70.0)
        node.declare_parameter('min_hold_width', 0.002)
        node.declare_parameter('move_tolerance', 0.003)
        node.declare_parameter('watch_rate', 10.0)
        node.declare_parameter('tf_timeout', 0.2)
        node.declare_parameter('startup_timeout', 10.0)

        self._hand_frame = node.get_parameter('hand_frame').value
        self._base_frame = node.get_parameter('base_frame').value
        self._max_width = float(node.get_parameter('max_width').value)
        self._max_force = float(node.get_parameter('max_force').value)
        self._min_hold_width = float(node.get_parameter('min_hold_width').value)
        self._move_tolerance = float(node.get_parameter('move_tolerance').value)
        self._tf_timeout = Duration(seconds=float(node.get_parameter('tf_timeout').value))
        self._startup_timeout = float(node.get_parameter('startup_timeout').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        self._ready = False
        self._goal_lock = threading.Lock()
        self._active_goal: ServerGoalHandle | None = None
        # Held by the goal that drives the gripper; a replaced goal releases it only after
        # its hardware command has stopped.
        self._hardware = threading.Lock()
        self._state_lock = threading.Lock()
        self._held: _Held | None = None
        self._busy = 0  # Grasp or Release running: the watch pauses

        for action_type, name, execute in (
            (MoveGripper, '/gripper/move', self._execute_move),
            (Grasp, '/gripper/grasp', self._execute_grasp),
            (Release, '/gripper/release', self._execute_release),
        ):
            ActionServer(
                node, action_type, name, self._one_at_a_time(execute),
                goal_callback=self._on_goal,
                handle_accepted_callback=self._on_goal_accepted,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=callback_group)
        self._startup_timer = node.create_timer(
            0.1, self._startup, callback_group=callback_group)
        node.create_timer(
            1.0 / float(node.get_parameter('watch_rate').value), self._watch,
            callback_group=callback_group)

    @property
    def ready(self) -> bool:
        return self._ready

    def _startup(self) -> None:
        self._startup_timer.cancel()
        try:
            self._gripper.prepare(self._startup_timeout)
        except GripperError as exc:
            self._node.get_logger().fatal(f'gripper not ready: {exc}')
            self._node.context.try_shutdown()
            return
        try:
            for obj in self._world_model.held_by(self._hand_frame):
                self._node.get_logger().warn(
                    f"'{obj.id}' is GRASPED in the world model; not watched until the next grasp")
        except WorldModelError as exc:
            self._node.get_logger().warn(f'{exc}')
        self._ready = True
        self._node.get_logger().info('gripper server ready')

    def _on_goal(self, _goal) -> GoalResponse:
        return GoalResponse.ACCEPT if self._ready else GoalResponse.REJECT

    def _on_goal_accepted(self, goal_handle: ServerGoalHandle) -> None:
        with self._goal_lock:
            self._active_goal = goal_handle
        goal_handle.execute()

    def _one_at_a_time(self, execute: Callable) -> Callable:
        def serialized(goal_handle: ServerGoalHandle):
            with self._hardware:
                return execute(goal_handle)
        return serialized

    def _interrupted(self, goal_handle: ServerGoalHandle) -> Callable[[], bool]:
        return lambda: goal_handle.is_cancel_requested or self._active_goal is not goal_handle

    def _finish(
        self,
        goal_handle: ServerGoalHandle,
        result: MoveGripper.Result | Grasp.Result | Release.Result,
        outcome: Outcome,
    ) -> MoveGripper.Result | Grasp.Result | Release.Result:
        result.outcome = outcome
        if outcome.code == Outcome.OK:
            goal_handle.succeed()
        elif outcome.code == Outcome.CANCELLED and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        with self._goal_lock:
            if self._active_goal is goal_handle:
                self._active_goal = None
        return result

    def _stopped(self, goal_handle: ServerGoalHandle) -> Outcome:
        if goal_handle.is_cancel_requested:
            return _outcome(Outcome.CANCELLED, 'cancelled by the client')
        return _outcome(Outcome.CANCELLED, 'replaced by a newer goal')

    def _in_frame(self, pose: PoseStamped, frame: str) -> PoseStamped:
        if pose.header.frame_id == frame:
            return pose
        transform = self._tf_buffer.lookup_transform(
            frame, pose.header.frame_id, Time(), timeout=self._tf_timeout)
        out = PoseStamped()
        out.header.frame_id = frame
        out.header.stamp = transform.header.stamp
        out.pose = do_transform_pose(pose.pose, transform)
        return out

    def _execute_move(self, goal_handle: ServerGoalHandle) -> MoveGripper.Result:
        result = MoveGripper.Result()
        width = goal_handle.request.width
        if not width_valid(width, self._max_width):
            return self._finish(goal_handle, result, _outcome(
                Outcome.INVALID_GOAL, f'width {width} not in [0, {self._max_width}]'))
        try:
            self._gripper.move(width, self._interrupted(goal_handle))
            result.measured_width = self._gripper.fresh_width()
        except Interrupted:
            return self._finish(goal_handle, result, self._stopped(goal_handle))
        except GripperError as exc:
            return self._finish(goal_handle, result, _outcome(Outcome.GRIPPER_FAILED, str(exc)))
        if abs(result.measured_width - width) > self._move_tolerance:
            return self._finish(goal_handle, result, _outcome(
                Outcome.GRIPPER_FAILED,
                f'stopped at {result.measured_width:.4f} m instead of {width:.4f} m'))
        return self._finish(goal_handle, result, _outcome(Outcome.OK))

    def _execute_grasp(self, goal_handle: ServerGoalHandle) -> Grasp.Result:
        result = Grasp.Result()
        goal = goal_handle.request
        if not width_valid(goal.width, self._max_width):
            return self._finish(goal_handle, result, _outcome(
                Outcome.INVALID_GOAL, f'width {goal.width} not in [0, {self._max_width}]'))
        if not force_valid(goal.force, self._max_force):
            return self._finish(goal_handle, result, _outcome(
                Outcome.INVALID_GOAL, f'force {goal.force} not in (0, {self._max_force}]'))
        if goal.tolerance <= 0.0:
            return self._finish(goal_handle, result, _outcome(
                Outcome.INVALID_GOAL, 'tolerance must be positive'))
        with self._state_lock:
            held = self._held
            self._busy += 1
        try:
            obj = self._world_model.get(goal.object_id)
            if obj is None:
                return self._finish(goal_handle, result, _outcome(
                    Outcome.NOT_FOUND, f"no object with id '{goal.object_id}'"))
            if obj.status != WorldObject.FREE:
                return self._finish(goal_handle, result, _outcome(
                    Outcome.INVALID_STATE, f"'{goal.object_id}' is not FREE"))
            if held is not None:
                return self._finish(goal_handle, result, _outcome(
                    Outcome.INVALID_STATE, f"already holding '{held.object_id}'"))

            interrupted = self._interrupted(goal_handle)
            width_before = self._gripper.width()
            self._gripper.grasp(goal.width, goal.force, goal.tolerance, interrupted)
            result.measured_width = self._gripper.fresh_width()
            if not grasp_succeeded(
                result.measured_width, goal.width, goal.tolerance, self._min_hold_width
            ) or self._gripper.holding() is False:
                if width_before is not None:
                    self._gripper.move(width_before, interrupted)
                return self._finish(goal_handle, result, _outcome(
                    Outcome.GRASP_FAILED,
                    f'measured {result.measured_width:.4f} m, expected {goal.width:.4f} m'))

            pose_in_hand = self._in_frame(obj.pose, self._hand_frame)
            outcome = self._world_model.set_status(
                goal.object_id, WorldObject.GRASPED, self._hand_frame, pose_in_hand)
            if outcome.code == Outcome.OK:
                with self._state_lock:
                    self._held = _Held(
                        goal.object_id, result.measured_width, goal.tolerance, pose_in_hand)
            return self._finish(goal_handle, result, outcome)
        except Interrupted:
            return self._finish(goal_handle, result, self._stopped(goal_handle))
        except (GripperError, TransformException) as exc:
            return self._finish(goal_handle, result, _outcome(Outcome.GRIPPER_FAILED, str(exc)))
        except WorldModelError as exc:
            return self._finish(goal_handle, result, _outcome(Outcome.TIMEOUT, str(exc)))
        finally:
            with self._state_lock:
                self._busy -= 1

    def _execute_release(self, goal_handle: ServerGoalHandle) -> Release.Result:
        result = Release.Result()
        goal = goal_handle.request
        if not width_valid(goal.open_width, self._max_width):
            return self._finish(goal_handle, result, _outcome(
                Outcome.INVALID_GOAL,
                f'open_width {goal.open_width} not in [0, {self._max_width}]'))
        with self._state_lock:
            self._busy += 1
        try:
            obj = self._world_model.get(goal.object_id)
            if obj is None:
                return self._finish(goal_handle, result, _outcome(
                    Outcome.NOT_FOUND, f"no object with id '{goal.object_id}'"))
            if obj.status != WorldObject.GRASPED:
                return self._finish(goal_handle, result, _outcome(
                    Outcome.INVALID_STATE, f"'{goal.object_id}' is not GRASPED"))

            self._gripper.move(goal.open_width, self._interrupted(goal_handle))
            measured = self._gripper.fresh_width()
            if not release_confirmed(measured, goal.open_width, self._move_tolerance):
                return self._finish(goal_handle, result, _outcome(
                    Outcome.RELEASE_FAILED,
                    f'opened to {measured:.4f} m instead of {goal.open_width:.4f} m'))

            released = self._in_frame(obj.pose, self._base_frame)
            outcome = self._world_model.set_status(
                goal.object_id, WorldObject.FREE, '', released)
            if outcome.code == Outcome.OK:
                result.released_pose = released
                with self._state_lock:
                    if self._held is not None and self._held.object_id == goal.object_id:
                        self._held = None
            return self._finish(goal_handle, result, outcome)
        except Interrupted:
            return self._finish(goal_handle, result, self._stopped(goal_handle))
        except (GripperError, TransformException) as exc:
            return self._finish(goal_handle, result, _outcome(Outcome.RELEASE_FAILED, str(exc)))
        except WorldModelError as exc:
            return self._finish(goal_handle, result, _outcome(Outcome.TIMEOUT, str(exc)))
        finally:
            with self._state_lock:
                self._busy -= 1

    def _watch(self) -> None:
        with self._state_lock:
            held = self._held
            if held is None or self._busy:
                return
        holding = self._gripper.holding()
        if holding is None:
            width = self._gripper.width()
            if width is None:
                return
            holding = still_holding(width, held.grasp_width, held.tolerance)
        if holding:
            return
        try:
            lost_at = self._in_frame(held.pose_in_hand, self._base_frame)
            outcome = self._world_model.set_status(
                held.object_id, WorldObject.LOST, '', lost_at)
        except (TransformException, WorldModelError) as exc:
            self._node.get_logger().warn(f"grip on '{held.object_id}' lost, not reported: {exc}")
            return
        with self._state_lock:
            if self._held is held:
                self._held = None
        self._node.get_logger().warn(
            f"grip on '{held.object_id}' lost -> LOST ({outcome.message or 'ok'})")


def make_gripper(node: Node, hardware: str, callback_group: CallbackGroup) -> Gripper:
    if hardware == 'real':
        # franka_msgs exists with the real profile only.
        from fer_gripper_server.adapters.franka_gripper import FrankaGripper
        return FrankaGripper(node, callback_group)
    if hardware == 'mujoco':
        return GripperCommandGripper(node, callback_group)
    raise ValueError(f"hardware must be 'real' or 'mujoco', got '{hardware}'")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Node('fer_gripper_server')
    node.declare_parameter('hardware', 'mujoco')
    callback_group = ReentrantCallbackGroup()
    gripper = make_gripper(node, node.get_parameter('hardware').value, callback_group)
    GripperServer(node, gripper, WorldModelClient(node, callback_group), callback_group)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
