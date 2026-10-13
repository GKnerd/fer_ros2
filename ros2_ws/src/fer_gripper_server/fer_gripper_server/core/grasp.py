"""Grasp decisions from measured widths."""
from __future__ import annotations

from typing import Callable, Protocol


class GripperError(RuntimeError):
    """The gripper did not carry out a command: error, timeout or not ready."""


class Interrupted(Exception):
    """The running command was cancelled or replaced by a newer goal."""


class Gripper(Protocol):
    """What the server needs from a gripper; one adapter per hardware."""

    def prepare(self, timeout: float) -> None: ...

    def move(self, width: float, interrupted: Callable[[], bool]) -> None: ...

    def grasp(
        self, width: float, force: float, tolerance: float, interrupted: Callable[[], bool]
    ) -> None: ...

    def width(self) -> float | None: ...

    def fresh_width(self) -> float: ...

    def holding(self) -> bool | None: ...


def width_valid(width: float, max_width: float) -> bool:
    return (0.0 <= width) and (width <= max_width)


def force_valid(force: float, max_force: float) -> bool:
    return (0.0 < force) and (force <= max_force)


def grasp_succeeded(
    measured: float, width: float, tolerance: float, min_hold_width: float
) -> bool:
    return (abs(measured - width) <= tolerance) and (measured > min_hold_width)


def release_confirmed(measured: float, open_width: float, tolerance: float) -> bool:
    return measured >= open_width - tolerance


def still_holding(measured: float, grasp_width: float, tolerance: float) -> bool:
    return abs(measured - grasp_width) <= tolerance
