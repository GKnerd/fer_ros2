"""World-model data types: objects, poses and detections."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum


@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class Pose:
    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)


@dataclass(frozen=True)
class Size:
    """Bounding-box extent in m, along the axes of the object's pose."""

    x: float
    y: float
    z: float


class ObjectStatus(IntEnum):
    """Values match the constants of fer_interfaces/WorldObject."""

    FREE = 0
    GRASPED = 1
    LOST = 2


@dataclass
class WorldObject:
    """One object known to the world model, represented by its bounding box."""

    object_id: str
    class_id: str
    size: Size
    pose: Pose
    frame: str
    status: ObjectStatus = ObjectStatus.FREE
    held_by: str = ''
    fixed: bool = False
    score: float = 1.0
    source: str = ''
    last_observed: float | None = None
    # First snapshot that missed the object; cleared when it is matched again.
    missing_since: float | None = None

    def validate(self, base_frame: str) -> None:
        """
        Check the status/frame invariants. raise ValueError if one is violated.

        :param base_frame: Frame every FREE or LOST pose is stored in.
        """
        if self.status is ObjectStatus.GRASPED:
            if not self.held_by:
                raise ValueError(f"'{self.object_id}': GRASPED needs held_by")
            if self.frame != self.held_by:
                raise ValueError(
                    f"'{self.object_id}': GRASPED pose must be in '{self.held_by}', "
                    f"got '{self.frame}'")
        else:
            if self.held_by:
                raise ValueError(f"'{self.object_id}': held_by is only allowed when GRASPED")
            if self.frame != base_frame:
                raise ValueError(
                    f"'{self.object_id}': pose must be in '{base_frame}', got '{self.frame}'")


@dataclass(frozen=True)
class Detection:
    """One detection, already transformed into the base frame."""

    class_id: str
    score: float
    pose: Pose
    size: Size
