"""
World model node: serves the world-model part of fer_interfaces.

Detections arrive on the detection topic; `DetectObjects` and `RefineObject` each use the
first message stamped after the goal was accepted. One detection goal runs at a time: a new
goal of either action ends the running one with outcome CANCELLED.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import threading
import time

from fer_interfaces.action import DetectObjects, RefineObject
from fer_interfaces.msg import Outcome, WorldObjectArray
from fer_interfaces.srv import QueryObjects, SetObjectStatus
from fer_world_model.adapters.ros_conversions import (
    detection_from_msg,
    object_to_msg,
    outcome,
    outcome_from_result,
    pose_from_msg,
    pose_to_msg,
)
from fer_world_model.core.config import fixtures_from_config
from fer_world_model.core.world_model import WorldModel
from fer_world_model.core.world_object import Detection, ObjectStatus, Point
from geometry_msgs.msg import Pose as PoseMsg
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray
import yaml


class _Stop(Enum):
    CANCELLED = 'cancelled'
    REPLACED = 'replaced'
    TIMEOUT = 'timeout'


@dataclass(frozen=True)
class _Snapshot:
    detections: list[Detection]
    stamp: Time


def build_world_model(node: Node) -> WorldModel:
    """Build the world model from the node's parameters and load its fixtures."""
    node.declare_parameter('base_frame', 'base')
    node.declare_parameter('association_gate', 0.05)
    node.declare_parameter('refine_gate', 0.10)
    node.declare_parameter('min_score', 0.5)
    node.declare_parameter('fixtures_file', '')

    model = WorldModel(
        base_frame=node.get_parameter('base_frame').value,
        association_gate=float(node.get_parameter('association_gate').value),
        refine_gate=float(node.get_parameter('refine_gate').value),
        min_score=float(node.get_parameter('min_score').value),
    )
    fixtures_file = node.get_parameter('fixtures_file').value
    if fixtures_file:
        with Path(fixtures_file).open() as f:
            model.load_fixtures(fixtures_from_config(yaml.safe_load(f), model.base_frame))
    return model


class WorldModelServer:

    def __init__(self, node: Node, model: WorldModel) -> None:
        self._node: Node = node
        self._model = model
        self._base_frame = model.base_frame
        self._node.declare_parameter('detection_topic', '/perception/detections')
        self._node.declare_parameter('detection_source', 'perception')
        self._node.declare_parameter('detection_timeout', 5.0)
        self._node.declare_parameter('tf_timeout', 0.1)
        self._node.declare_parameter('removal_timeout', 30.0)

        self._source = self._node.get_parameter('detection_source').value
        self._detection_timeout = float(self._node.get_parameter('detection_timeout').value)
        self._tf_timeout = Duration(
            seconds=float(self._node.get_parameter('tf_timeout').value))
        self._removal_timeout = float(self._node.get_parameter('removal_timeout').value)

        self._lock = threading.Lock()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

        self._snapshot_condition = threading.Condition()
        self._latest: Detection3DArray | None = None
        self._goal_lock = threading.Lock()
        self._active_goal: ServerGoalHandle | None = None

        callback_group = ReentrantCallbackGroup()
        self._node.create_subscription(
            Detection3DArray, self._node.get_parameter('detection_topic').value,
            self._on_detections, qos_profile_sensor_data, callback_group=callback_group)
        self._objects_pub = self._node.create_publisher(
            WorldObjectArray, '/world_model/objects',
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self._node.create_service(
            QueryObjects, '/world_model/query_objects', self._on_query,
            callback_group=callback_group)
        self._node.create_service(
            SetObjectStatus, '/world_model/set_object_status', self._on_set_status,
            callback_group=callback_group)
        for action_type, name, execute in (
            (DetectObjects, '/world_model/detect_objects', self._execute_detect),
            (RefineObject, '/world_model/refine_object', self._execute_refine),
        ):
            ActionServer(
                self._node, action_type, name, execute,
                goal_callback=lambda _: GoalResponse.ACCEPT,
                handle_accepted_callback=self._on_goal_accepted,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=callback_group)
        self._node.create_timer(1.0, self._sweep, callback_group=callback_group)

        with self._lock:
            self._publish_locked()
        self._node.get_logger().info(
            f'world model up: {len(self._model.snapshot())} fixture(s), detections on '
            f"'{self._node.get_parameter('detection_topic').value}'")

    def _now(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    def _publish_locked(self) -> None:
        msg = WorldObjectArray()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.revision = self._model.revision
        msg.objects = [object_to_msg(o) for o in self._model.snapshot()]
        self._objects_pub.publish(msg)

    def _on_detections(self, msg: Detection3DArray) -> None:
        with self._snapshot_condition:
            self._latest = msg
            self._snapshot_condition.notify_all()

    def _on_goal_accepted(self, goal_handle: ServerGoalHandle) -> None:
        with self._goal_lock:
            self._active_goal = goal_handle
        with self._snapshot_condition:
            self._snapshot_condition.notify_all()
        goal_handle.execute()

    def _wait_for_message(
        self, goal_handle: ServerGoalHandle, after: Time, deadline: float
    ) -> Detection3DArray | _Stop:
        with self._snapshot_condition:
            while True:
                if goal_handle.is_cancel_requested:
                    return _Stop.CANCELLED
                if self._active_goal is not goal_handle:
                    return _Stop.REPLACED
                msg = self._latest
                if msg is not None and Time.from_msg(msg.header.stamp) > after:
                    return msg
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return _Stop.TIMEOUT
                self._snapshot_condition.wait(min(remaining, 0.1))

    def _take_snapshot(self, goal_handle: ServerGoalHandle) -> _Snapshot | _Stop:
        """Wait for the first detection message stamped after now that TF can resolve."""
        after = self._node.get_clock().now()
        deadline = time.monotonic() + self._detection_timeout
        while True:
            msg = self._wait_for_message(goal_handle, after, deadline)
            if isinstance(msg, _Stop):
                return msg
            stamp = Time.from_msg(msg.header.stamp)
            detections = self._detections_in_base(msg, stamp)
            if detections is not None:
                return _Snapshot(detections=detections, stamp=stamp)
            after = stamp

    def _detections_in_base(
        self, msg: Detection3DArray, stamp: Time
    ) -> list[Detection] | None:
        frame = msg.header.frame_id
        transform = None
        if frame != self._base_frame:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._base_frame, frame, stamp, timeout=self._tf_timeout)
            except TransformException as exc:
                self._node.get_logger().warn(f'detections skipped: {exc}')
                return None
        detections = []
        for detection_msg in msg.detections:
            center = detection_msg.bbox.center
            pose = center if transform is None else do_transform_pose(center, transform)
            detection = detection_from_msg(detection_msg, pose)
            if detection is not None:
                detections.append(detection)
        return detections

    def _grasped_positions(self, stamp: Time) -> list[Point]:
        with self._lock:
            grasped = self._model.query([], [], [int(ObjectStatus.GRASPED)], False)
        positions = []
        for obj in grasped:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._base_frame, obj.frame, stamp, timeout=self._tf_timeout)
            except TransformException as exc:
                self._node.get_logger().warn(f"no position for held '{obj.object_id}': {exc}")
                continue
            p = do_transform_pose(pose_to_msg(obj.pose), transform).position
            positions.append(Point(x=p.x, y=p.y, z=p.z))
        return positions

    def _end(
        self,
        goal_handle: ServerGoalHandle,
        result: DetectObjects.Result | RefineObject.Result,
        stop: _Stop,
    ) -> DetectObjects.Result | RefineObject.Result:
        if stop is _Stop.CANCELLED:
            result.outcome = outcome(Outcome.CANCELLED, 'cancelled by the client')
            goal_handle.canceled()
        elif stop is _Stop.REPLACED:
            result.outcome = outcome(Outcome.CANCELLED, 'replaced by a newer goal')
            goal_handle.abort()
        else:
            result.outcome = outcome(
                Outcome.TIMEOUT,
                f'no detections within {self._detection_timeout:.1f} s')
            goal_handle.abort()
        return result

    def _release(self, goal_handle: ServerGoalHandle) -> None:
        with self._goal_lock:
            if self._active_goal is goal_handle:
                self._active_goal = None

    def _execute_detect(self, goal_handle: ServerGoalHandle) -> DetectObjects.Result:
        result = DetectObjects.Result()
        try:
            snapshot = self._take_snapshot(goal_handle)
            if isinstance(snapshot, _Stop):
                return self._end(goal_handle, result, snapshot)
            grasped_positions = self._grasped_positions(snapshot.stamp)
            with self._lock:
                update = self._model.apply_detections(
                    snapshot.detections, list(goal_handle.request.class_ids),
                    grasped_positions, snapshot.stamp.nanoseconds * 1e-9, self._source)
                if update.added or update.updated:
                    self._publish_locked()
                result.revision = self._model.revision
            result.added = update.added
            result.updated = update.updated
            result.not_seen = update.not_seen
            result.outcome = outcome(Outcome.OK)
            goal_handle.succeed()
            return result
        finally:
            self._release(goal_handle)

    def _execute_refine(self, goal_handle: ServerGoalHandle) -> RefineObject.Result:
        result = RefineObject.Result()
        object_id = goal_handle.request.object_id
        try:
            with self._lock:
                found = self._model.query([object_id], [], [], True)
                result.revision = self._model.revision
            if not found:
                result.outcome = outcome(Outcome.NOT_FOUND, f"no object with id '{object_id}'")
                goal_handle.abort()
                return result
            if found[0].fixed or found[0].status is ObjectStatus.GRASPED:
                result.outcome = outcome(
                    Outcome.INVALID_STATE,
                    f"'{object_id}' is {'fixed' if found[0].fixed else 'GRASPED'}")
                goal_handle.abort()
                return result

            snapshot = self._take_snapshot(goal_handle)
            if isinstance(snapshot, _Stop):
                return self._end(goal_handle, result, snapshot)
            with self._lock:
                refined = self._model.refine(
                    object_id, snapshot.detections, snapshot.stamp.nanoseconds * 1e-9,
                    self._source)
                if refined.ok:
                    self._publish_locked()
                result.revision = self._model.revision
            result.outcome = outcome_from_result(refined)
            if refined.ok:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        finally:
            self._release(goal_handle)

    def _on_query(
        self, request: QueryObjects.Request, response: QueryObjects.Response
    ) -> QueryObjects.Response:
        with self._lock:
            objects = self._model.query(
                list(request.ids), list(request.class_ids), list(request.statuses),
                request.include_fixed)
            response.revision = self._model.revision
        response.objects = [object_to_msg(o) for o in objects]
        response.outcome = outcome(Outcome.OK)
        return response

    def _on_set_status(
        self, request: SetObjectStatus.Request, response: SetObjectStatus.Response
    ) -> SetObjectStatus.Response:
        try:
            status = ObjectStatus(request.status)
        except ValueError:
            with self._lock:
                response.revision = self._model.revision
            response.outcome = outcome(
                Outcome.INVALID_GOAL, f'unknown status {request.status}')
            return response

        frame = request.pose.header.frame_id
        pose: PoseMsg = request.pose.pose
        if status is not ObjectStatus.GRASPED and frame and frame != self._base_frame:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._base_frame, frame, Time.from_msg(request.pose.header.stamp),
                    timeout=self._tf_timeout)
            except TransformException as exc:
                with self._lock:
                    response.revision = self._model.revision
                response.outcome = outcome(Outcome.INVALID_GOAL, f'pose frame: {exc}')
                return response
            pose = do_transform_pose(pose, transform)
            frame = self._base_frame

        with self._lock:
            changed = self._model.set_status(
                request.id, status, request.held_by, pose_from_msg(pose), frame, self._now())
            if changed.ok:
                self._publish_locked()
            response.revision = self._model.revision
        response.outcome = outcome_from_result(changed)
        return response

    def _sweep(self) -> None:
        with self._lock:
            removed = self._model.sweep(self._now(), self._removal_timeout)
            if removed:
                self._publish_locked()
        if removed:
            self._node.get_logger().info(
                f'removed after {self._removal_timeout:.0f} s missing: {removed}')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Node('fer_world_model')
    WorldModelServer(node, build_world_model(node))
    executor = MultiThreadedExecutor(num_threads=4)
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
