"""
The franka adapter against fake franka_gripper actions and gripper_state.

Runs only where franka_msgs with GripperState exists (real profile).
"""
import threading
import time

from fer_gripper_server.core.grasp import GripperError
import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

try:
    from fer_gripper_server.adapters.franka_gripper import FrankaGripper
    from franka_msgs.action import Grasp, Move
    from franka_msgs.msg import GripperState
except ImportError:
    GripperState = None

# A module-level pytest.skip would end the whole session under the launch_testing plugin.
pytestmark = pytest.mark.skipif(
    GripperState is None, reason='franka_msgs with GripperState not available')


class FakeFrankaGripper:

    def __init__(self, node: Node) -> None:
        group = ReentrantCallbackGroup()
        self.goals = []
        self.error = ''
        self.state = GripperState(width=0.08, max_width=0.08, is_grasped=False)
        ActionServer(node, Move, '/fer_gripper/move', self._execute, callback_group=group)
        ActionServer(node, Grasp, '/fer_gripper/grasp', self._execute, callback_group=group)
        self._pub = node.create_publisher(GripperState, '/fer_gripper/gripper_state', 10)
        node.create_timer(0.02, lambda: self._pub.publish(self.state), callback_group=group)

    def _execute(self, goal_handle):
        self.goals.append(goal_handle.request)
        action = Grasp if hasattr(goal_handle.request, 'epsilon') else Move
        result = action.Result(success=not self.error, error=self.error)
        if self.error:
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return result


@pytest.fixture
def setup():
    context = rclpy.Context()
    rclpy.init(context=context)
    adapter_node = Node(
        'fer_gripper_server', context=context,
        parameter_overrides=[Parameter('speed', value=0.05)])
    fake_node = Node('fake_franka_gripper', context=context)
    fake = FakeFrankaGripper(fake_node)
    adapter = FrankaGripper(adapter_node, ReentrantCallbackGroup())
    executor = MultiThreadedExecutor(context=context)
    executor.add_node(adapter_node)
    executor.add_node(fake_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    adapter.prepare(5.0)
    yield adapter, fake
    executor.shutdown()
    adapter_node.destroy_node()
    fake_node.destroy_node()
    rclpy.shutdown(context=context)
    thread.join(timeout=5.0)


def never() -> bool:
    return False


def test_grasp_passes_tolerance_as_epsilon(setup):
    adapter, fake = setup
    adapter.grasp(0.05, 30.0, 0.004, never)
    goal = fake.goals[-1]
    assert goal.width == 0.05 and goal.force == 30.0 and goal.speed == 0.05
    assert goal.epsilon.inner == 0.004 and goal.epsilon.outer == 0.004


def test_gripper_error_is_raised(setup):
    adapter, fake = setup
    fake.error = 'gripper not homed'
    with pytest.raises(GripperError, match='not homed'):
        adapter.move(0.08, never)


def test_state_gives_width_and_is_grasped(setup):
    adapter, fake = setup
    fake.state = GripperState(width=0.05, max_width=0.08, is_grasped=True)
    time.sleep(0.1)
    assert adapter.fresh_width() == pytest.approx(0.05)
    assert adapter.holding() is True
