#!/usr/bin/env python3
"""Control-mode switch check for the FER. Moves nothing, but the arm is under control.

  ros2 run fer_bringup fer_mode_switch_check.py [--cycles 3]

Switching effort <-> velocity is ATOMIC at the ros2_control level but a genuine
stop/restart at the libfranka level: perform_command_mode_switch calls stopRobot()
(join the control thread, robot_->stop() over the network) and then initializes the new
loop. This measures how long the arm has no running control loop -- the number that
matters when MoveIt hands off between controllers.

It also asserts the guard: activating a controller WITHOUT deactivating the running one
must be rejected, because prepare_command_mode_switch refuses a start while
control_mode_ != None. Both lists therefore have to travel in ONE request:

  ros2 control switch_controllers --deactivate <old> --activate <new>
"""

import argparse
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState

EFFORT = 'effort_joint_trajectory_controller'
VELOCITY = 'vel_joint_trajectory_controller'
SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=200,
                        durability=QoSDurabilityPolicy.VOLATILE)
GREEN, RED, YELLOW, DIM, RESET = '\033[32m', '\033[31m', '\033[33m', '\033[2m', '\033[0m'


class Results:
    def __init__(self, title):
        self.entries = []
        print(f'\n=== {title} ===\n')

    def _add(self, status, name, detail):
        self.entries.append(status)
        colour = {'PASS': GREEN, 'FAIL': RED, 'SKIP': YELLOW, 'INFO': DIM}[status]
        print(f'  {colour}{status:<4}{RESET}  {name}')
        for line in str(detail).splitlines():
            if line:
                print(f'          {DIM}{line}{RESET}')

    def ok(self, n, d=''):
        self._add('PASS', n, d)

    def fail(self, n, d=''):
        self._add('FAIL', n, d)

    def skip(self, n, d=''):
        self._add('SKIP', n, d)

    def info(self, n, d=''):
        self._add('INFO', n, d)

    def check(self, cond, n, d=''):
        (self.ok if cond else self.fail)(n, d)
        return cond

    def verdict(self):
        failed = self.entries.count('FAIL')
        print(f'\n  {self.entries.count("PASS")} passed, {failed} failed')
        print(f'  {RED}VERDICT: FAIL{RESET}\n' if failed else f'  {GREEN}VERDICT: PASS{RESET}\n')
        return 1 if failed else 0


class GapProbe:
    """Largest interval between consecutive joint_states stamps -- the dead window."""

    def __init__(self, node, topic):
        self.stamps = []
        self.sub = node.create_subscription(JointState, topic, self._cb, SENSOR_QOS)

    def _cb(self, msg):
        self.stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def max_gap(self):
        if len(self.stamps) < 2:
            return None
        return max(b - a for a, b in zip(self.stamps, self.stamps[1:]))


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)


class ControllerManager:
    def __init__(self, node, timeout=5.0):
        self.node = node
        self._list = node.create_client(ListControllers, '/controller_manager/list_controllers')
        self._switch = node.create_client(SwitchController, '/controller_manager/switch_controller')
        for client, name in ((self._list, 'list_controllers'), (self._switch, 'switch_controller')):
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f'/controller_manager/{name} unavailable -- '
                                   'is the stack running?')

    def states(self):
        future = self._list.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        if future.result() is None:
            raise RuntimeError('list_controllers timed out')
        return {c.name: c.state for c in future.result().controller}

    def switch(self, activate=(), deactivate=(), timeout=10.0):
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = False
        future = self._switch.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if future.result() is None:
            raise RuntimeError('switch_controller timed out')
        return future.result().ok


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--arm-id', default='fer')
    parser.add_argument('--cycles', type=int, default=3)
    parser.add_argument('--settle', type=float, default=2.0,
                        help='seconds to hold in each mode (default: 2.0)')
    args = parser.parse_args()

    rclpy.init()
    node = Node('fer_mode_switch_check')
    res = Results(f'FER control mode switch ({args.cycles} cycles)')
    topic = f'/{args.arm_id}/joint_states'

    try:
        cm = ControllerManager(node)
        states = cm.states()
        for name in (EFFORT, VELOCITY):
            if states.get(name) is None:
                raise RuntimeError(f'{name} is not loaded; check the bringup spawners')

        running = [c for c in (EFFORT, VELOCITY) if states.get(c) == 'active']
        if not running:
            if not cm.switch(activate=[EFFORT]):
                raise RuntimeError(f'could not activate {EFFORT} to begin')
            running = [EFFORT]
            res.info(f'activated {EFFORT} to begin')

        current = running[0]

        # The guard: a start while control_mode_ != None must be refused.
        other = VELOCITY if current == EFFORT else EFFORT
        refused = not cm.switch(activate=[other])
        res.check(refused, 'activating a second mode without deactivating the first is refused',
                  'prepare_command_mode_switch: "Switching between control modes without '
                  'stopping it first is not supported."')
        if not refused:
            cm.switch(activate=[current], deactivate=[other])  # leave the arm sane

        gaps = []
        for cycle in range(args.cycles):
            target = VELOCITY if current == EFFORT else EFFORT
            probe = GapProbe(node, topic)
            spin_for(node, 0.5)
            ok = cm.switch(activate=[target], deactivate=[current])
            spin_for(node, args.settle)

            if not res.check(ok, f'cycle {cycle + 1}: atomic switch {current} -> {target}'):
                break
            gap = probe.max_gap()
            if gap is not None:
                gaps.append(gap)
                res.info(f'cycle {cycle + 1}: largest joint_states gap {gap * 1000:.1f} ms',
                         'the window where the libfranka loop was torn down and restarted')
            res.check(cm.states().get(target) == 'active',
                      f'cycle {cycle + 1}: {target} active after switch')
            node.destroy_subscription(probe.sub)
            current = target

        if gaps:
            res.info(f'dead window: min {min(gaps) * 1000:.1f} ms, '
                     f'max {max(gaps) * 1000:.1f} ms, n={len(gaps)}',
                     'budget this when planning controller handoffs under MoveIt')
        res.info(f'left active: {current}')
        return res.verdict()
    except KeyboardInterrupt:
        return 130
    except RuntimeError as exc:
        print(f'\n{RED}ERROR: {exc}{RESET}\n')
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
