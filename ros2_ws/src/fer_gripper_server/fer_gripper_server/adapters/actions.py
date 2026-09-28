"""Blocking helpers for futures and action goals, called from executor worker threads."""
from __future__ import annotations

import threading
import time
from typing import Any, Callable

from fer_gripper_server.core.grasp import GripperError, Interrupted
from rclpy.action import ActionClient
from rclpy.task import Future


def wait_for(
    future: Future, timeout: float, interrupted: Callable[[], bool] | None = None
) -> bool:
    """
    Wait until `future` is done; False on timeout.

    Raises Interrupted as soon as `interrupted()` returns True.
    """
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    deadline = time.monotonic() + timeout
    while not done.is_set():
        if interrupted is not None and interrupted():
            raise Interrupted()
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        done.wait(min(remaining, 0.05))
    return True


def run_goal(
    client: ActionClient,
    name: str,
    goal: Any,
    timeout: float,
    interrupted: Callable[[], bool],
) -> Any:
    """
    Send `goal`, wait for its result and return the wrapped result (status and result).

    An interrupted goal is cancelled on the server and awaited, so the hardware has stopped
    before the next command. Raises GripperError when the goal is not accepted or not
    finished within `timeout`, Interrupted when stopped, also if that happens just as the
    goal finishes.
    """
    if interrupted():
        raise Interrupted()
    sent = client.send_goal_async(goal)
    if not wait_for(sent, timeout):
        raise GripperError(f"no answer from '{name}'")
    handle = sent.result()
    if not handle.accepted:
        raise GripperError(f"'{name}' rejected the goal")
    result = handle.get_result_async()
    try:
        finished = wait_for(result, timeout, interrupted)
    except Interrupted:
        wait_for(handle.cancel_goal_async(), timeout)
        wait_for(result, timeout)
        raise
    if not finished:
        handle.cancel_goal_async()
        raise GripperError(f"'{name}' did not finish within {timeout:.1f} s")
    if interrupted():
        raise Interrupted()
    return result.result()


class WidthMonitor:
    """Latest measured width and when it arrived."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._width: float | None = None
        self._received = 0.0

    def update(self, width: float) -> None:
        with self._condition:
            self._width = width
            self._received = time.monotonic()
            self._condition.notify_all()

    def latest(self) -> float | None:
        with self._condition:
            return self._width

    def fresh(self, timeout: float) -> float:
        """Return the first width received after this call; raise GripperError on timeout."""
        start = time.monotonic()
        with self._condition:
            if not self._condition.wait_for(lambda: self._received > start, timeout):
                raise GripperError(f'no gripper state within {timeout:.1f} s')
            return self._width
