"""
Grasp planner node: serves GetGraspCandidates and GetPlaceCandidates of fer_interfaces.

Grasp candidates are computed top-down from the object's bounding box, in the object's
frame (`base` for FREE objects). The grasp poses of the latest answer are published on
/grasp/debug/candidates for RViz; that topic is not part of the contract. Place
candidates are in `base`.
"""
from __future__ import annotations

from fer_grasp_planner.adapters.ros_conversions import (
    box_from_msg,
    candidate_to_msg,
    pose_to_msg,
)
from fer_grasp_planner.adapters.world_model_client import WorldModelClient, WorldModelError
from fer_grasp_planner.core.catalog import Catalog, catalog_from_dict, CatalogError
from fer_grasp_planner.core.top_down import top_down_candidates, TopDownParams
from fer_grasp_planner.place_server import PlacePlannerServer
from fer_interfaces.msg import Outcome, WorldObject
from fer_interfaces.srv import GetGraspCandidates
from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformListener
import yaml


def _outcome(code: int, message: str = '') -> Outcome:
    return Outcome(code=code, message=message)


class GraspPlannerServer:

    def __init__(
        self,
        node: Node,
        catalog: Catalog,
        world_model: WorldModelClient,
        callback_group: CallbackGroup,
    ) -> None:
        self._node = node
        self._catalog = catalog
        self._world_model = world_model
        node.declare_parameter('max_object_width', 0.07)
        node.declare_parameter('pregrasp_distance', 0.10)
        node.declare_parameter('lift_height', 0.10)
        node.declare_parameter('max_grasp_depth', 0.04)
        node.declare_parameter('min_tip_clearance', 0.012)
        self._params = TopDownParams(
            max_object_width=float(node.get_parameter('max_object_width').value),
            pregrasp_distance=float(node.get_parameter('pregrasp_distance').value),
            lift_height=float(node.get_parameter('lift_height').value),
            max_grasp_depth=float(node.get_parameter('max_grasp_depth').value),
            min_tip_clearance=float(node.get_parameter('min_tip_clearance').value),
        )
        self._debug = node.create_publisher(
            PoseArray, '/grasp/debug/candidates',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_service(
            GetGraspCandidates, '/grasp/candidates', self._on_request,
            callback_group=callback_group)

    def _on_request(
        self, request: GetGraspCandidates.Request, response: GetGraspCandidates.Response
    ) -> GetGraspCandidates.Response:
        try:
            obj = self._world_model.get(request.object_id)
        except WorldModelError as exc:
            response.outcome = _outcome(Outcome.TIMEOUT, str(exc))
            return response
        if obj is None:
            response.outcome = _outcome(
                Outcome.NOT_FOUND, f"no object with id '{request.object_id}'")
            return response
        if obj.fixed:
            response.outcome = _outcome(
                Outcome.INVALID_STATE, f"'{request.object_id}' is fixed")
            return response
        if obj.status != WorldObject.FREE:
            response.outcome = _outcome(
                Outcome.INVALID_STATE, f"'{request.object_id}' is not FREE")
            return response

        candidates = top_down_candidates(box_from_msg(obj), self._params)
        if request.max_candidates > 0:
            candidates = candidates[:request.max_candidates]
        frame_id = obj.pose.header.frame_id
        stamp = self._node.get_clock().now().to_msg()
        force = self._catalog.force_for(obj.class_id)
        response.candidates = [candidate_to_msg(c, frame_id, stamp, force) for c in candidates]
        response.outcome = _outcome(Outcome.OK)

        debug = PoseArray(poses=[pose_to_msg(c.grasp) for c in candidates])
        debug.header.frame_id = frame_id
        debug.header.stamp = stamp
        self._debug.publish(debug)
        return response


def load_catalog(path: str) -> Catalog:
    try:
        with open(path) as f:
            return catalog_from_dict(yaml.safe_load(f))
    except (OSError, yaml.YAMLError) as exc:
        raise CatalogError(f"cannot read catalog '{path}': {exc}") from None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Node('fer_grasp_planner')
    node.declare_parameter('catalog_file', '')

    try:
        catalog = load_catalog(node.get_parameter('catalog_file').value)

    except CatalogError as exc:
        node.get_logger().fatal(str(exc))
        node.destroy_node()
        rclpy.try_shutdown()
        return

    callback_group = ReentrantCallbackGroup()
    world_model = WorldModelClient(node, callback_group)
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    GraspPlannerServer(node, catalog, world_model, callback_group)
    PlacePlannerServer(node, world_model, tf_buffer, callback_group)
    node.get_logger().info('grasp planner ready')
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
