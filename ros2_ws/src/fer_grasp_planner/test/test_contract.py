"""
Contract test: the grasp planner against the fer_interfaces rules.

Both servers run in-process with a fake world model serving QueryObjects and a static
transform base -> place_frame.
"""
import math
import threading
import time
from typing import Any, Callable, Iterator

from fer_grasp_planner.adapters.world_model_client import WorldModelClient
from fer_grasp_planner.core.catalog import Catalog
from fer_grasp_planner.grasp_planner_server import GraspPlannerServer
from fer_grasp_planner.place_server import PlacePlannerServer
from fer_interfaces.msg import Outcome, WorldObject
from fer_interfaces.srv import GetGraspCandidates, GetPlaceCandidates, QueryObjects
from geometry_msgs.msg import PoseArray, TransformStamped
import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

CATALOG = Catalog(default_force=15.0, forces={'sphere': 10.0})


def wait(future: Future, timeout: float = 5.0) -> Any:
    deadline = time.monotonic() + timeout
    while not future.done():
        assert time.monotonic() < deadline, 'timed out'
        time.sleep(0.01)
    return future.result()


def wait_until(condition: Callable[[], bool], timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while not condition():
        assert time.monotonic() < deadline, 'timed out'
        time.sleep(0.01)


class FakeWorldModel:

    def __init__(self, node: Node) -> None:
        self.objects: dict[str, WorldObject] = {}
        node.create_service(
            QueryObjects, '/world_model/query_objects', self._query,
            callback_group=ReentrantCallbackGroup())

    def add(
        self,
        object_id: str,
        class_id: str = 'box',
        size: tuple = (0.05, 0.05, 0.05),
        status: int = WorldObject.FREE,
        fixed: bool = False,
        held_by: str = 'fer_hand_tcp',
    ) -> None:
        obj = WorldObject(id=object_id, class_id=class_id, status=status, fixed=fixed)
        if status == WorldObject.GRASPED:
            # Upright, grasped at its center from above: hand z down, hand x along base x.
            obj.held_by = held_by
            obj.pose.header.frame_id = held_by
            obj.pose.pose.orientation.x = 1.0
        else:
            obj.pose.header.frame_id = 'base'
            obj.pose.pose.position.x = 0.4
            obj.pose.pose.position.z = size[2] / 2.0
            obj.pose.pose.orientation.w = 1.0
        obj.shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))
        self.objects[object_id] = obj

    def _query(
        self, request: QueryObjects.Request, response: QueryObjects.Response
    ) -> QueryObjects.Response:
        response.objects = [
            o for o in self.objects.values()
            if (not request.ids or o.id in request.ids)
            and (request.include_fixed or not o.fixed)]
        response.outcome = Outcome(code=Outcome.OK)
        return response


class Harness:

    def __init__(self, with_world_model: bool = True) -> None:
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.server_node = Node(
            'fer_grasp_planner', context=self.context,
            parameter_overrides=[Parameter('world_model_timeout', value=1.0),
                                 Parameter('tf_timeout', value=1.0)])
        group = ReentrantCallbackGroup()
        world_model = WorldModelClient(self.server_node, group)
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self.server_node)
        GraspPlannerServer(self.server_node, CATALOG, world_model, group)
        PlacePlannerServer(self.server_node, world_model, self.tf_buffer, group)

        self.node = Node('contract_client', context=self.context)
        self.world = FakeWorldModel(self.node) if with_world_model else None
        place_frame = TransformStamped()
        place_frame.header.frame_id = 'base'
        place_frame.child_frame_id = 'place_frame'
        place_frame.transform.translation.x = 0.1
        place_frame.transform.rotation.z = math.sin(math.pi / 4.0)
        place_frame.transform.rotation.w = math.cos(math.pi / 4.0)
        self._tf = StaticTransformBroadcaster(self.node)
        self._tf.sendTransform(place_frame)
        self.client = self.node.create_client(GetGraspCandidates, '/grasp/candidates')
        self.place_client = self.node.create_client(GetPlaceCandidates, '/place/candidates')
        self._debug_lock = threading.Lock()
        self.debug: list[PoseArray] = []
        self.node.create_subscription(
            PoseArray, '/grasp/debug/candidates', self._on_debug,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.executor = MultiThreadedExecutor(context=self.context)
        self.executor.add_node(self.server_node)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()
        assert self.client.wait_for_service(timeout_sec=5.0)
        assert self.place_client.wait_for_service(timeout_sec=5.0)
        wait_until(lambda: self.tf_buffer.can_transform('base', 'place_frame', Time()))
        if with_world_model:
            probe = self.node.create_client(QueryObjects, '/world_model/query_objects')
            assert probe.wait_for_service(timeout_sec=5.0)

    def close(self) -> None:
        self.executor.shutdown()
        self.server_node.destroy_node()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)
        self._thread.join(timeout=5.0)

    def _on_debug(self, msg: PoseArray) -> None:
        with self._debug_lock:
            self.debug.append(msg)

    def candidates(
        self, object_id: str, max_candidates: int = 0
    ) -> GetGraspCandidates.Response:
        request = GetGraspCandidates.Request(object_id=object_id, max_candidates=max_candidates)
        return wait(self.client.call_async(request))

    def places(
        self, object_id: str, x: float = 0.35, y: float = 0.30, frame: str = 'base'
    ) -> GetPlaceCandidates.Response:
        request = GetPlaceCandidates.Request(object_id=object_id)
        request.target.header.frame_id = frame
        request.target.pose.position.x = x
        request.target.pose.position.y = y
        request.target.pose.orientation.w = 1.0
        return wait(self.place_client.call_async(request))


@pytest.fixture
def harness() -> Iterator[Harness]:
    h = Harness()
    yield h
    h.close()


def test_candidates_for_a_free_object(harness):
    harness.world.add('box_1')
    response = harness.candidates('box_1')
    assert response.outcome.code == Outcome.OK
    assert len(response.candidates) == 4
    for c in response.candidates:
        for pose in (c.pregrasp_pose, c.grasp_pose, c.lift_pose):
            assert pose.header.frame_id == 'base'
        assert c.grasp_pose.pose.position.x == pytest.approx(0.4)
        assert c.grasp_pose.pose.position.z == pytest.approx(0.025)
        assert c.pregrasp_pose.pose.position.z == pytest.approx(0.125)
        assert c.lift_pose.pose.position.z == pytest.approx(0.125)
        assert c.width == pytest.approx(0.05)
        assert c.force == pytest.approx(15.0)


def test_force_from_the_catalog(harness):
    harness.world.add('sphere_1', class_id='sphere', size=(0.06, 0.06, 0.06))
    response = harness.candidates('sphere_1')
    assert response.outcome.code == Outcome.OK
    assert all(c.force == pytest.approx(10.0) for c in response.candidates)


def test_max_candidates(harness):
    harness.world.add('box_1')
    response = harness.candidates('box_1', max_candidates=1)
    assert response.outcome.code == Outcome.OK
    assert len(response.candidates) == 1


def test_unknown_object(harness):
    assert harness.candidates('box_9').outcome.code == Outcome.NOT_FOUND


@pytest.mark.parametrize('status', [WorldObject.GRASPED, WorldObject.LOST])
def test_object_not_free(harness, status):
    harness.world.add('box_1', status=status)
    assert harness.candidates('box_1').outcome.code == Outcome.INVALID_STATE


def test_fixed_object(harness):
    harness.world.add('table', class_id='table', size=(1.2, 1.2, 0.02), fixed=True)
    assert harness.candidates('table').outcome.code == Outcome.INVALID_STATE


def test_no_side_fits_the_gripper(harness):
    harness.world.add('box_1', size=(0.09, 0.09, 0.05))
    response = harness.candidates('box_1')
    assert response.outcome.code == Outcome.OK
    assert len(response.candidates) == 0


def test_debug_topic_shows_the_latest_grasp_poses(harness):
    harness.world.add('box_1')
    response = harness.candidates('box_1')

    def latest() -> PoseArray | None:
        with harness._debug_lock:
            return harness.debug[-1] if harness.debug else None

    wait_until(lambda: latest() is not None and len(latest().poses) == 4)
    shown = latest()
    assert shown.header.frame_id == 'base'
    assert [p.position.z for p in shown.poses] == pytest.approx(
        [c.grasp_pose.pose.position.z for c in response.candidates])


def test_world_model_not_answering():
    h = Harness(with_world_model=False)
    try:
        assert h.candidates('box_1').outcome.code == Outcome.TIMEOUT
        assert h.places('box_1').outcome.code == Outcome.TIMEOUT
    finally:
        h.close()


def test_place_candidates_for_a_held_object(harness):
    harness.world.add('box_1', status=WorldObject.GRASPED)
    response = harness.places('box_1')
    assert response.outcome.code == Outcome.OK
    assert len(response.candidates) == 2
    for c in response.candidates:
        for pose in (c.preplace_pose, c.place_pose, c.retreat_pose):
            assert pose.header.frame_id == 'base'
        place = c.place_pose.pose.position
        assert (place.x, place.y, place.z) == pytest.approx((0.35, 0.30, 0.03))
        assert c.preplace_pose.pose.position.z == pytest.approx(0.13)
        assert c.retreat_pose.pose.position.z == pytest.approx(0.13)


def test_place_target_in_another_frame(harness):
    harness.world.add('box_1', status=WorldObject.GRASPED)
    response = harness.places('box_1', x=0.2, y=0.1, frame='place_frame')
    assert response.outcome.code == Outcome.OK
    for c in response.candidates:
        assert c.place_pose.header.frame_id == 'base'
        place = c.place_pose.pose.position
        assert (place.x, place.y, place.z) == pytest.approx((0.0, 0.2, 0.03))


def test_place_target_frame_unknown(harness):
    harness.world.add('box_1', status=WorldObject.GRASPED)
    assert harness.places('box_1', frame='nowhere').outcome.code == Outcome.INVALID_GOAL


def test_place_unknown_object(harness):
    assert harness.places('box_9').outcome.code == Outcome.NOT_FOUND


@pytest.mark.parametrize('status', [WorldObject.FREE, WorldObject.LOST])
def test_place_object_not_grasped(harness, status):
    harness.world.add('box_1', status=status)
    assert harness.places('box_1').outcome.code == Outcome.INVALID_STATE


def test_place_object_held_by_another_frame(harness):
    harness.world.add('box_1', status=WorldObject.GRASPED, held_by='other_hand')
    assert harness.places('box_1').outcome.code == Outcome.INVALID_STATE
