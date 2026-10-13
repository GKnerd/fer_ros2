"""
Contract test: the world-model node against the fer_interfaces rules.

The node runs in-process with fake detections from a camera 1 m above `base`.
"""
import threading
import time

from fer_interfaces.action import DetectObjects, RefineObject
from fer_interfaces.msg import Outcome, WorldObject, WorldObjectArray
from fer_interfaces.srv import QueryObjects, SetObjectStatus
from fer_world_model.world_model_server import build_world_model, WorldModelServer
from geometry_msgs.msg import TransformStamped, Vector3
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from shape_msgs.msg import SolidPrimitive
from tf2_ros import StaticTransformBroadcaster
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

CAMERA_HEIGHT = 1.0
FIXTURES = """
fixtures:
  - id: table
    size: [1.0, 1.0, 0.02]
    pose:
      position: {x: 0.3, y: 0.0, z: -0.01}
"""


def wait(future, timeout: float = 5.0):
    deadline = time.monotonic() + timeout
    while not future.done():
        assert time.monotonic() < deadline, 'timed out'
        time.sleep(0.01)
    return future.result()


class Harness:

    def __init__(self, tmp_path) -> None:
        fixtures = tmp_path / 'fixtures.yaml'
        fixtures.write_text(FIXTURES)
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.server_node = Node(
            'fer_world_model',
            context=self.context,
            parameter_overrides=[
                Parameter('fixtures_file', value=str(fixtures)),
                Parameter('detection_timeout', value=1.0),
            ])
        self.server = WorldModelServer(self.server_node, build_world_model(self.server_node))
        self.node = Node('contract_client', context=self.context)

        # (class_id, x, y, z) in the base frame, published from the camera frame.
        self.detections: list[tuple[str, float, float, float]] = []
        self.publishing = True
        self._pub = self.node.create_publisher(Detection3DArray, '/perception/detections', 1)
        self.node.create_timer(0.05, self._publish)
        camera = TransformStamped()
        camera.header.frame_id = 'base'
        camera.child_frame_id = 'camera'
        camera.transform.translation.z = CAMERA_HEIGHT
        camera.transform.rotation.w = 1.0
        self._tf = StaticTransformBroadcaster(self.node)
        self._tf.sendTransform(camera)

        self.detect_client = ActionClient(
            self.node, DetectObjects, '/world_model/detect_objects')
        self.refine_client = ActionClient(
            self.node, RefineObject, '/world_model/refine_object')
        self.query_client = self.node.create_client(
            QueryObjects, '/world_model/query_objects')
        self.set_status_client = self.node.create_client(
            SetObjectStatus, '/world_model/set_object_status')

        self.executor = MultiThreadedExecutor(context=self.context)
        self.executor.add_node(self.server_node)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()
        assert self.detect_client.wait_for_server(timeout_sec=5.0)
        assert self.refine_client.wait_for_server(timeout_sec=5.0)
        assert self.query_client.wait_for_service(timeout_sec=5.0)
        assert self.set_status_client.wait_for_service(timeout_sec=5.0)

    def close(self) -> None:
        self.executor.shutdown()
        self.server_node.destroy_node()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)
        self._thread.join(timeout=5.0)

    def _publish(self) -> None:
        if not self.publishing:
            return
        msg = Detection3DArray()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        for class_id, x, y, z in self.detections:
            detection = Detection3D()
            detection.header = msg.header
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_id
            hypothesis.hypothesis.score = 0.9
            detection.results = [hypothesis]
            detection.bbox.center.position.x = x
            detection.bbox.center.position.y = y
            detection.bbox.center.position.z = z - CAMERA_HEIGHT
            detection.bbox.center.orientation.w = 1.0
            detection.bbox.size = Vector3(x=0.05, y=0.05, z=0.05)
            msg.detections.append(detection)
        self._pub.publish(msg)

    def send_goal(self, client: ActionClient, goal):
        goal_handle = wait(client.send_goal_async(goal))
        assert goal_handle.accepted
        return goal_handle

    def run(self, client: ActionClient, goal):
        return wait(self.send_goal(client, goal).get_result_async()).result

    def detect(self, class_ids: list[str] | None = None) -> DetectObjects.Result:
        return self.run(self.detect_client, DetectObjects.Goal(class_ids=class_ids or []))

    def refine(self, object_id: str) -> RefineObject.Result:
        return self.run(self.refine_client, RefineObject.Goal(object_id=object_id))

    def query(self, **fields) -> QueryObjects.Response:
        return wait(self.query_client.call_async(QueryObjects.Request(**fields)))

    def set_status(self, object_id: str, status: int, held_by: str, frame: str,
                   x: float = 0.0) -> SetObjectStatus.Response:
        request = SetObjectStatus.Request(id=object_id, status=status, held_by=held_by)
        request.pose.header.frame_id = frame
        request.pose.pose.position.x = x
        request.pose.pose.orientation.w = 1.0
        return wait(self.set_status_client.call_async(request))


@pytest.fixture
def harness(tmp_path):
    h = Harness(tmp_path)
    yield h
    h.close()


def test_detect_adds_then_updates(harness):
    harness.detections = [('box', 0.4, 0.1, 0.025), ('cylinder', 0.5, -0.25, 0.05)]
    first = harness.detect()
    assert first.outcome.code == Outcome.OK
    assert sorted(first.added) == ['box_1', 'cylinder_1']

    second = harness.detect()
    assert second.outcome.code == Outcome.OK
    assert second.added == [] and sorted(second.updated) == ['box_1', 'cylinder_1']
    assert second.revision > first.revision

    objects = {o.id: o for o in harness.query().objects}
    box = objects['box_1']
    assert box.pose.header.frame_id == 'base'
    assert box.pose.pose.position.z == pytest.approx(0.025)
    assert box.shape.type == SolidPrimitive.BOX
    assert list(box.shape.dimensions) == pytest.approx([0.05, 0.05, 0.05])
    assert box.status == WorldObject.FREE and not box.fixed
    assert len(harness.query(include_fixed=True).objects) == 3


def test_detect_reports_not_seen(harness):
    harness.detections = [('box', 0.4, 0.1, 0.025)]
    harness.detect()
    harness.detections = []
    result = harness.detect()
    assert result.outcome.code == Outcome.OK and result.not_seen == ['box_1']
    assert [o.id for o in harness.query().objects] == ['box_1']


def test_set_status_contract(harness):
    harness.detections = [('box', 0.4, 0.1, 0.025)]
    harness.detect()

    assert harness.set_status('nope', WorldObject.FREE, '', 'base').outcome.code \
        == Outcome.NOT_FOUND
    assert harness.set_status('box_1', 7, '', 'base').outcome.code == Outcome.INVALID_GOAL
    assert harness.set_status('box_1', WorldObject.GRASPED, '', 'fer_hand_tcp').outcome.code \
        == Outcome.INVALID_GOAL
    assert harness.set_status('table', WorldObject.LOST, '', 'base').outcome.code \
        == Outcome.INVALID_GOAL
    assert harness.set_status('box_1', WorldObject.FREE, '', 'no_such_frame').outcome.code \
        == Outcome.INVALID_GOAL

    grasped = harness.set_status('box_1', WorldObject.GRASPED, 'fer_hand_tcp', 'fer_hand_tcp')
    assert grasped.outcome.code == Outcome.OK
    assert harness.refine('box_1').outcome.code == Outcome.INVALID_STATE

    released = harness.set_status('box_1', WorldObject.FREE, '', 'camera', x=0.6)
    assert released.outcome.code == Outcome.OK and released.revision > grasped.revision
    box = harness.query(ids=['box_1']).objects[0]
    assert box.status == WorldObject.FREE and box.source == 'release_estimate'
    assert box.pose.header.frame_id == 'base'
    assert box.pose.pose.position.z == pytest.approx(CAMERA_HEIGHT)


def test_refine(harness):
    harness.detections = [('box', 0.4, 0.1, 0.025)]
    harness.detect()
    assert harness.refine('nope').outcome.code == Outcome.NOT_FOUND
    assert harness.refine('table').outcome.code == Outcome.INVALID_STATE

    harness.detections = [('box', 0.43, 0.1, 0.025)]
    assert harness.refine('box_1').outcome.code == Outcome.OK
    box = harness.query(ids=['box_1']).objects[0]
    assert box.pose.pose.position.x == pytest.approx(0.43)

    harness.detections = []
    assert harness.refine('box_1').outcome.code == Outcome.NOT_FOUND


def test_detect_times_out_without_detections(harness):
    harness.publishing = False
    assert harness.detect().outcome.code == Outcome.TIMEOUT


def test_new_goal_replaces_the_running_one(harness):
    harness.publishing = False
    first = harness.send_goal(harness.detect_client, DetectObjects.Goal())
    second = harness.send_goal(harness.refine_client, RefineObject.Goal(object_id='table'))
    assert wait(first.get_result_async()).result.outcome.code == Outcome.CANCELLED
    assert wait(second.get_result_async()).result.outcome.code == Outcome.INVALID_STATE

    third = harness.send_goal(harness.detect_client, DetectObjects.Goal())
    fourth = harness.send_goal(harness.detect_client, DetectObjects.Goal())
    harness.publishing = True
    assert wait(third.get_result_async()).result.outcome.code == Outcome.CANCELLED
    assert wait(fourth.get_result_async()).result.outcome.code == Outcome.OK


def test_client_cancel(harness):
    harness.publishing = False
    goal_handle = harness.send_goal(harness.detect_client, DetectObjects.Goal())
    wait(goal_handle.cancel_goal_async())
    assert wait(goal_handle.get_result_async()).result.outcome.code == Outcome.CANCELLED


def test_objects_topic_is_latched(harness):
    harness.detections = [('box', 0.4, 0.1, 0.025)]
    revision = harness.detect().revision
    received = []
    got = threading.Event()

    def on_objects(msg: WorldObjectArray) -> None:
        received.append(msg)
        got.set()

    harness.node.create_subscription(
        WorldObjectArray, '/world_model/objects', on_objects,
        QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
    assert got.wait(timeout=5.0)
    assert received[-1].revision == revision
    assert {o.id for o in received[-1].objects} == {'box_1', 'table'}
