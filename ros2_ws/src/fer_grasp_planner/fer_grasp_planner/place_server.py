"""
Place candidates: serves GetPlaceCandidates of fer_interfaces.

The target is converted to `base` once, with the latest transform. The held object's pose
relative to `fer_hand_tcp` comes from the world model.
"""
from __future__ import annotations

from fer_grasp_planner.adapters.ros_conversions import placement_to_msg, pose_from_msg
from fer_grasp_planner.adapters.world_model_client import WorldModelClient, WorldModelError
from fer_grasp_planner.core.place import place_candidates, PlaceParams
from fer_interfaces.msg import Outcome, WorldObject
from fer_interfaces.srv import GetPlaceCandidates
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import CallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException

BASE_FRAME = 'base'
HAND_FRAME = 'fer_hand_tcp'


def _outcome(code: int, message: str = '') -> Outcome:
    return Outcome(code=code, message=message)


class PlacePlannerServer:

    def __init__(
        self,
        node: Node,
        world_model: WorldModelClient,
        tf_buffer: Buffer,
        callback_group: CallbackGroup,
    ) -> None:
        self._node = node
        self._world_model = world_model
        self._tf_buffer = tf_buffer
        node.declare_parameter('place_clearance', 0.005)
        node.declare_parameter('preplace_distance', 0.10)
        node.declare_parameter('retreat_height', 0.10)
        node.declare_parameter('tf_timeout', 0.2)
        self._params = PlaceParams(
            place_clearance=float(node.get_parameter('place_clearance').value),
            preplace_distance=float(node.get_parameter('preplace_distance').value),
            retreat_height=float(node.get_parameter('retreat_height').value),
        )
        self._tf_timeout = Duration(seconds=float(node.get_parameter('tf_timeout').value))
        node.create_service(
            GetPlaceCandidates, '/place/candidates', self._on_request,
            callback_group=callback_group)

    def _on_request(
        self, request: GetPlaceCandidates.Request, response: GetPlaceCandidates.Response
    ) -> GetPlaceCandidates.Response:
        q = request.target.pose.orientation
        if q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w < 1e-6:
            response.outcome = _outcome(
                Outcome.INVALID_GOAL, 'target orientation is not a rotation')
            return response
        try:
            target = self._in_base(request.target)
        except TransformException as exc:
            response.outcome = _outcome(
                Outcome.INVALID_GOAL,
                f"no transform from '{request.target.header.frame_id}' to '{BASE_FRAME}': "
                f'{exc}')
            return response

        try:
            obj = self._world_model.get(request.object_id)
        except WorldModelError as exc:
            response.outcome = _outcome(Outcome.TIMEOUT, str(exc))
            return response
        if obj is None:
            response.outcome = _outcome(
                Outcome.NOT_FOUND, f"no object with id '{request.object_id}'")
            return response
        if obj.status != WorldObject.GRASPED:
            response.outcome = _outcome(
                Outcome.INVALID_STATE, f"'{request.object_id}' is not GRASPED")
            return response
        if obj.pose.header.frame_id != HAND_FRAME:
            response.outcome = _outcome(
                Outcome.INVALID_STATE,
                f"'{request.object_id}' is held by '{obj.pose.header.frame_id}', "
                f'not {HAND_FRAME}')
            return response

        placements = place_candidates(
            tuple(float(d) for d in obj.shape.dimensions[:3]),
            pose_from_msg(obj.pose.pose), pose_from_msg(target), self._params)
        stamp = self._node.get_clock().now().to_msg()
        response.candidates = [placement_to_msg(p, BASE_FRAME, stamp) for p in placements]
        response.outcome = _outcome(Outcome.OK)
        return response

    def _in_base(self, pose: PoseStamped) -> PoseMsg:
        if pose.header.frame_id == BASE_FRAME:
            return pose.pose
        transform = self._tf_buffer.lookup_transform(
            BASE_FRAME, pose.header.frame_id, Time(), timeout=self._tf_timeout)
        return do_transform_pose(pose.pose, transform)
