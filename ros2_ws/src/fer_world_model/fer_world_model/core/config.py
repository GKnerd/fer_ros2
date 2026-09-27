"""Read poses, sizes and fixtures from parsed YAML."""
from __future__ import annotations

from typing import Dict, List

from fer_world_model.core.world_object import (
    Point,
    Pose,
    Quaternion,
    Size,
    WorldObject,
)


def pose_from_config(entry: Dict) -> Pose:
    """
    Build a Pose from `{position: {x, y, z}, orientation: {x, y, z, w}}`.

    A missing orientation is the identity.
    """
    position = entry['position']
    orientation = entry.get('orientation', {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0})
    return Pose(
        position=Point(
            x=float(position['x']), y=float(position['y']), z=float(position['z'])),
        orientation=Quaternion(
            x=float(orientation['x']), y=float(orientation['y']),
            z=float(orientation['z']), w=float(orientation['w'])),
    )


def size_from_config(values: List) -> Size:
    """
    Build a Size from `[x, y, z]`.

    Raises ValueError unless there are exactly three positive values.
    """
    if len(values) != 3 or any(float(v) <= 0.0 for v in values):
        raise ValueError(f'size must be three positive values [x, y, z], got {values}')
    return Size(*(float(v) for v in values))


def fixtures_from_config(config: Dict, base_frame: str) -> List[WorldObject]:
    """
    Build fixed objects from `{fixtures: [{id, class_id, size, pose}]}`.

    Poses are in the base frame. Raises KeyError or ValueError on a malformed entry.
    """
    return [
        WorldObject(
            object_id=entry['id'],
            class_id=entry.get('class_id', entry['id']),
            size=size_from_config(entry['size']),
            pose=pose_from_config(entry['pose']),
            frame=base_frame,
            fixed=True,
            score=1.0,
            source='manual',
        )
        for entry in (config or {}).get('fixtures') or []
    ]
