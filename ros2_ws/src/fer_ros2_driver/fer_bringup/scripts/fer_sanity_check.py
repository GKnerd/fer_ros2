#!/usr/bin/env python3
"""Sanity check for the FER lower-level stack. Moves nothing.

  ros2 run fer_bringup fer_sanity_check.py [--duration 30]

Checks topic rates, joint validity, the TF chain, and that controllers STAY active --
plus whether a libfranka control loop is actually running.

That last check is the one worth understanding. An "active" controller is NOT evidence
that the arm is commandable. On 2026-09-09 the controller read active, all 7 effort
interfaces read [claimed], and joint_states streamed at 1 kHz, while no control loop
existed and the arm silently ignored commands for 102 s. Only FrankaState.robot_mode and
control_command_success_rate reveal that. See "Information" in the fer_ros2 README.
"""

import argparse
import math
import statistics
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from franka_msgs.msg import FrankaState
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

EXPECTED_ARM_HZ = 1000.0
EXPECTED_GRIPPER_HZ = 15.0
RATE_TOLERANCE = 0.25
ROBOT_MODE_NAMES = {0: 'OTHER', 1: 'IDLE', 2: 'MOVE', 3: 'GUIDING',
                    4: 'REFLEX', 5: 'USER_STOPPED', 6: 'AUTOMATIC_ERROR_RECOVERY'}
ROBOT_MODE_MOVE = 2
TRAJECTORY_CONTROLLERS = ('effort_joint_trajectory_controller',
                          'vel_joint_trajectory_controller')

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


class RateProbe:
    """Rate from header stamps. mean is the assertion; median-interval is a diagnostic
    that separates a bursty publisher from rclpy dropping messages at 1 kHz."""

    def __init__(self, node, topic, msg_type):
        self.topic, self.stamps, self.count = topic, [], 0
        node.create_subscription(msg_type, topic, self._cb, SENSOR_QOS)

    def _cb(self, msg):
        self.count += 1
        self.stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def mean_rate(self):
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return (len(self.stamps) - 1) / span if span > 0 else 0.0

    def median_rate(self):
        deltas = [b - a for a, b in zip(self.stamps, self.stamps[1:]) if b > a]
        median = statistics.median(deltas) if deltas else 0
        return 1.0 / median if median > 0 else 0.0

    def describe(self):
        return (f'{self.count} msgs, mean {self.mean_rate():.0f} Hz, '
                f'median-interval {self.median_rate():.0f} Hz')


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)


def latest(node, topic, msg_type, timeout=5.0):
    holder = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: holder.__setitem__('m', m), SENSOR_QOS)
    left = timeout
    while 'm' not in holder and left > 0 and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        left -= 0.05
    node.destroy_subscription(sub)
    return holder.get('m')


def control_loop_live(state):
    """A commanding loop shows robot_mode MOVE with a non-zero success rate. IDLE with
    rate 0.0 is continuous-reading mode: state streams, commands reach no one."""
    if state is None:
        return False, 'no FrankaState received'
    mode = ROBOT_MODE_NAMES.get(state.robot_mode, f'UNKNOWN({state.robot_mode})')
    rate = state.control_command_success_rate
    if state.robot_mode == ROBOT_MODE_MOVE and rate > 0.0:
        return True, f'robot_mode {mode}, command success rate {rate:.2f}'
    return False, f'robot_mode {mode}, command success rate {rate:.2f} -- no commanding loop'


def controller_states(node, timeout=5.0):
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError('/controller_manager/list_controllers unavailable -- '
                           'is the stack running?')
    future = client.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if future.result() is None:
        raise RuntimeError('list_controllers timed out')
    return {c.name: c.state for c in future.result().controller}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--arm-id', default='fer')
    parser.add_argument('--duration', type=float, default=30.0,
                        help='seconds to sample and re-verify controller state')
    parser.add_argument('--base-frame', default='base')
    parser.add_argument('--tip-frame', default=None, help='default: <arm_id>_hand_tcp')
    parser.add_argument('--no-gripper', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = Node('fer_sanity_check')
    res = Results(f'FER sanity check (arm_id={args.arm_id}, {args.duration:.0f}s)')
    names = [f'{args.arm_id}_joint{i}' for i in range(1, 8)]
    tip = args.tip_frame or f'{args.arm_id}_hand_tcp'

    try:
        before = controller_states(node)
        for name, state in sorted(before.items()):
            res.info(name, f'state: {state}')
        for name in ('joint_state_broadcaster', 'franka_robot_state_broadcaster'):
            res.check(before.get(name) == 'active', f'{name} active at start',
                      f'state: {before.get(name, "NOT LOADED")}')

        arm = RateProbe(node, f'/{args.arm_id}/joint_states', JointState)
        robot_state = RateProbe(node, '/franka_robot_state_broadcaster/robot_state', FrankaState)
        gripper = None if args.no_gripper else RateProbe(
            node, f'/{args.arm_id}_gripper/joint_states', JointState)
        tf_buffer = Buffer()
        TransformListener(tf_buffer, node)

        print(f'  ... sampling for {args.duration:.0f}s\n')
        spin_for(node, args.duration)

        res.check(abs(arm.mean_rate() - EXPECTED_ARM_HZ) <= EXPECTED_ARM_HZ * RATE_TOLERANCE,
                  f'{arm.topic} at ~{EXPECTED_ARM_HZ:.0f} Hz', arm.describe())
        res.check(robot_state.count > 0, f'{robot_state.topic} publishing',
                  robot_state.describe())
        if gripper is not None:
            res.check(abs(gripper.mean_rate() - EXPECTED_GRIPPER_HZ)
                      <= EXPECTED_GRIPPER_HZ * RATE_TOLERANCE,
                      f'{gripper.topic} at ~{EXPECTED_GRIPPER_HZ:.0f} Hz', gripper.describe())

        js = latest(node, f'/{args.arm_id}/joint_states', JointState)
        if js is None:
            res.fail('arm joint states', 'no message received')
        else:
            missing = [n for n in names if n not in js.name]
            res.check(not missing, 'all 7 arm joints present',
                      f'missing: {missing}' if missing else ', '.join(names))
            bad = [js.name[i] for i in range(len(js.name))
                   if i < len(js.position) and not math.isfinite(js.position[i])]
            res.check(not bad, 'all joint positions finite',
                      f'non-finite: {bad}' if bad else 'no NaN/inf')

        # base->link0 and link7->hand->hand_tcp are FIXED joints and live on /tf_static.
        # Echoing /tf alone shows only the revolute chain and misses both ends.
        try:
            res.check(tf_buffer.can_transform(args.base_frame, tip, rclpy.time.Time()),
                      f'TF {args.base_frame} -> {tip} resolves',
                      'spans /tf_static (fixed) and /tf (revolute)')
        except Exception as exc:  # noqa: BLE001 - tf2 raises a family of lookup errors
            res.fail(f'TF {args.base_frame} -> {tip} resolves', str(exc))

        live, why = control_loop_live(
            latest(node, '/franka_robot_state_broadcaster/robot_state', FrankaState))
        driving = [c for c in TRAJECTORY_CONTROLLERS if before.get(c) == 'active']
        if driving:
            res.check(live, f'control loop running for {driving[0]}',
                      why if live else why +
                      '\nSee "Recovering from a fault" in the fer_ros2 README.')
        else:
            res.info('no motion controller active',
                     why + '\nIDLE is expected with only broadcasters running.')

        after = controller_states(node)
        changed = {k: (before.get(k), v) for k, v in after.items() if before.get(k) != v}
        res.check(not changed, f'no controller changed state over {args.duration:.0f}s',
                  f'changed: {changed}' if changed else 'all states stable')
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
