"""
Contract test: the gripper server with the MuJoCo adapter against the fer_interfaces rules.

The server runs in-process with a simulated hand (active GripperCommand controller,
/joint_states), a fake world model and a static base -> fer_hand_tcp transform.
"""
import threading
import time

from control_msgs.action import GripperCommand
from fer_gripper_server.adapters.gripper_command import GripperCommandGripper
from fer_gripper_server.adapters.world_model_client import WorldModelClient
from fer_gripper_server.gripper_server import GripperServer
from fer_interfaces.action import Grasp, MoveGripper, Release
from fer_interfaces.msg import Outcome, WorldObject
from fer_interfaces.srv import QueryObjects, SetObjectStatus
from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster

HAND_Z = 0.5
OBJECT_Z = 0.4


def wait(future, timeout: float = 5.0):
    deadline = time.monotonic() + timeout
    while not future.done():
        assert time.monotonic() < deadline, 'timed out'
        time.sleep(0.01)
    return future.result()


def wait_until(condition, timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while not condition():
        assert time.monotonic() < deadline, 'timed out'
        time.sleep(0.01)


class FakeHand:
    """Active GripperCommand controller and /joint_states of a simulated hand."""

    FINGER_SPEED = 0.05  # m/s per finger

    def __init__(self, node: Node) -> None:
        group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self.finger = 0.04
        self.target = 0.04
        self.object_width: float | None = None
        ActionServer(
            node, GripperCommand, '/gripper_effort_controller/gripper_cmd', self._execute,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        self._pub = node.create_publisher(JointState, '/joint_states', 10)
        node.create_timer(0.01, self._step, callback_group=group)

    def width(self) -> float:
        with self._lock:
            return 2.0 * self.finger

    def _goal_locked(self) -> float:
        if self.object_width is not None and self.target < self.object_width / 2.0 \
                <= self.finger + 1e-9:
            return self.object_width / 2.0
        return self.target

    def _step(self) -> None:
        with self._lock:
            delta = self._goal_locked() - self.finger
            step = self.FINGER_SPEED * 0.01
            self.finger += max(-step, min(step, delta))
            finger = self.finger
        self._pub.publish(JointState(name=['fer_finger_joint1'], position=[finger]))

    def _execute(self, goal_handle) -> GripperCommand.Result:
        with self._lock:
            self.target = goal_handle.request.command.position
        while True:
            if goal_handle.is_cancel_requested:
                with self._lock:
                    self.target = self.finger
                goal_handle.canceled()
                return GripperCommand.Result()
            with self._lock:
                settled = abs(self._goal_locked() - self.finger) < 1e-6
            if settled:
                goal_handle.succeed()
                return GripperCommand.Result(position=self.finger, reached_goal=True)
            time.sleep(0.01)


class FakeWorldModel:

    def __init__(self, node: Node) -> None:
        group = ReentrantCallbackGroup()
        self.objects: dict[str, WorldObject] = {}
        self.calls: list[SetObjectStatus.Request] = []
        node.create_service(
            QueryObjects, '/world_model/query_objects', self._query, callback_group=group)
        node.create_service(
            SetObjectStatus, '/world_model/set_object_status', self._set, callback_group=group)

    def add(self, object_id: str, status: int = WorldObject.FREE) -> None:
        obj = WorldObject(id=object_id, class_id='box', status=status)
        obj.pose.header.frame_id = 'base'
        obj.pose.pose.position.x = 0.3
        obj.pose.pose.position.z = OBJECT_Z
        obj.pose.pose.orientation.w = 1.0
        self.objects[object_id] = obj

    def _query(self, request, response):
        response.objects = [
            o for o in self.objects.values()
            if (not request.ids or o.id in request.ids)
            and (not request.statuses or o.status in request.statuses)]
        response.outcome = Outcome(code=Outcome.OK)
        return response

    def _set(self, request, response):
        obj = self.objects.get(request.id)
        if obj is None:
            response.outcome = Outcome(code=Outcome.NOT_FOUND)
            return response
        obj.status = request.status
        obj.held_by = request.held_by
        obj.pose = request.pose
        self.calls.append(request)
        response.outcome = Outcome(code=Outcome.OK)
        return response


class Harness:

    def __init__(self) -> None:
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.server_node = Node(
            'fer_gripper_server', context=self.context,
            parameter_overrides=[
                Parameter('max_force', value=20.0),
                Parameter('watch_rate', value=20.0),
                Parameter('startup_timeout', value=5.0),
                Parameter('action_timeout', value=5.0),
            ])
        group = ReentrantCallbackGroup()
        self.server = GripperServer(
            self.server_node, GripperCommandGripper(self.server_node, group),
            WorldModelClient(self.server_node, group), group)

        self.node = Node('contract_client', context=self.context)
        self.hand = FakeHand(self.node)
        self.world = FakeWorldModel(self.node)
        hand = TransformStamped()
        hand.header.frame_id = 'base'
        hand.child_frame_id = 'fer_hand_tcp'
        hand.transform.translation.x = 0.3
        hand.transform.translation.z = HAND_Z
        hand.transform.rotation.w = 1.0
        self._tf = StaticTransformBroadcaster(self.node)
        self._tf.sendTransform(hand)
        self.move_client = ActionClient(self.node, MoveGripper, '/gripper/move')
        self.grasp_client = ActionClient(self.node, Grasp, '/gripper/grasp')
        self.release_client = ActionClient(self.node, Release, '/gripper/release')

        self.executor = MultiThreadedExecutor(context=self.context)
        self.executor.add_node(self.server_node)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()
        wait_until(lambda: self.server.ready)
        for client in (self.move_client, self.grasp_client, self.release_client):
            assert client.wait_for_server(timeout_sec=5.0)

    def close(self) -> None:
        self.executor.shutdown()
        self.server_node.destroy_node()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)
        self._thread.join(timeout=5.0)

    def send(self, client: ActionClient, goal):
        goal_handle = wait(client.send_goal_async(goal))
        assert goal_handle.accepted
        return goal_handle

    def result(self, goal_handle):
        return wait(goal_handle.get_result_async(), timeout=10.0).result

    def run(self, client: ActionClient, goal):
        return self.result(self.send(client, goal))

    def move(self, width: float) -> MoveGripper.Result:
        return self.run(self.move_client, MoveGripper.Goal(width=width))

    def grasp(self, object_id: str, width: float = 0.05, force: float = 10.0) -> Grasp.Result:
        return self.run(
            self.grasp_client, Grasp.Goal(object_id=object_id, width=width, force=force))

    def release(self, object_id: str) -> Release.Result:
        return self.run(self.release_client, Release.Goal(object_id=object_id))

    def statuses(self) -> list[int]:
        return [call.status for call in self.world.calls]


@pytest.fixture
def harness():
    h = Harness()
    yield h
    h.close()


def test_move(harness):
    result = harness.move(0.02)
    assert result.outcome.code == Outcome.OK
    assert result.measured_width == pytest.approx(0.02, abs=0.003)
    assert harness.move(0.08).outcome.code == Outcome.OK
    assert harness.move(0.09).outcome.code == Outcome.INVALID_GOAL


def test_grasp_on_nothing_reopens_and_leaves_the_world_model(harness):
    harness.world.add('box_1')
    result = harness.grasp('box_1')
    assert result.outcome.code == Outcome.GRASP_FAILED
    assert result.measured_width == pytest.approx(0.0, abs=0.002)
    assert harness.hand.width() == pytest.approx(0.08, abs=0.003)
    assert harness.world.calls == []


def test_grasp_and_release(harness):
    harness.world.add('box_1')
    harness.hand.object_width = 0.05
    grasped = harness.grasp('box_1')
    assert grasped.outcome.code == Outcome.OK
    assert grasped.measured_width == pytest.approx(0.05, abs=0.002)
    call = harness.world.calls[-1]
    assert call.status == WorldObject.GRASPED and call.held_by == 'fer_hand_tcp'
    assert call.pose.header.frame_id == 'fer_hand_tcp'
    assert call.pose.pose.position.z == pytest.approx(OBJECT_Z - HAND_Z)

    released = harness.release('box_1')
    assert released.outcome.code == Outcome.OK
    assert released.released_pose.header.frame_id == 'base'
    assert released.released_pose.pose.position.z == pytest.approx(OBJECT_Z)
    assert harness.statuses() == [WorldObject.GRASPED, WorldObject.FREE]


def test_validation(harness):
    harness.world.add('box_1')
    harness.world.add('held', status=WorldObject.GRASPED)
    assert harness.grasp('nope').outcome.code == Outcome.NOT_FOUND
    assert harness.grasp('held').outcome.code == Outcome.INVALID_STATE
    assert harness.grasp('box_1', force=0.0).outcome.code == Outcome.INVALID_GOAL
    assert harness.grasp('box_1', force=25.0).outcome.code == Outcome.INVALID_GOAL
    assert harness.grasp('box_1', width=0.2).outcome.code == Outcome.INVALID_GOAL
    assert harness.release('nope').outcome.code == Outcome.NOT_FOUND
    assert harness.release('box_1').outcome.code == Outcome.INVALID_STATE
    assert harness.world.calls == []


def test_object_taken_out_of_the_hand_becomes_lost(harness):
    harness.world.add('box_1')
    harness.hand.object_width = 0.05
    assert harness.grasp('box_1').outcome.code == Outcome.OK
    harness.hand.object_width = None
    wait_until(lambda: harness.statuses()[-1] == WorldObject.LOST)
    lost = harness.world.calls[-1]
    assert lost.pose.header.frame_id == 'base'
    assert lost.pose.pose.position.z == pytest.approx(OBJECT_Z)


def test_opening_while_holding_becomes_lost(harness):
    harness.world.add('box_1')
    harness.hand.object_width = 0.05
    assert harness.grasp('box_1').outcome.code == Outcome.OK
    assert harness.move(0.08).outcome.code == Outcome.OK
    wait_until(lambda: harness.statuses()[-1] == WorldObject.LOST)


def test_new_goal_replaces_the_running_one(harness):
    first = harness.send(harness.move_client, MoveGripper.Goal(width=0.0))
    second = harness.send(harness.move_client, MoveGripper.Goal(width=0.08))
    assert harness.result(first).outcome.code == Outcome.CANCELLED
    assert harness.result(second).outcome.code == Outcome.OK


def test_client_cancel(harness):
    goal_handle = harness.send(harness.move_client, MoveGripper.Goal(width=0.0))
    time.sleep(0.2)
    wait(goal_handle.cancel_goal_async())
    assert harness.result(goal_handle).outcome.code == Outcome.CANCELLED
    assert harness.hand.width() > 0.01
