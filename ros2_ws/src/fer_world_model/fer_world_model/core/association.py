"""Match detections to known objects."""
from __future__ import annotations

import math
from typing import Sequence

from fer_world_model.core.world_object import Detection, Point, WorldObject


def distance(a: Point, b: Point) -> float:
    return math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))


def associate(
    detections: Sequence[Detection],
    objects: Sequence[WorldObject],
    gate: float,
) -> list[tuple[int, str]]:
    """
    Pair detections with objects of the same class, nearest pairs first.

    Every pair closer than `gate` is a candidate. Candidates are taken in order of
    increasing distance; each detection and each object is used at most once.
    Returns (detection index, object id) for every match.

    :param detections: Detections in the base frame.
    :param objects: Objects that may be matched.
    :param gate: Largest center distance in m that still counts as the same object.
    """
    candidates = []
    for index, detection in enumerate(detections):
        for obj in objects:
            if obj.class_id != detection.class_id:
                continue
            d = distance(detection.pose.position, obj.pose.position)
            if d <= gate:
                candidates.append((d, index, obj.object_id))
    candidates.sort()

    used_detections: set[int] = set()
    used_objects: set[str] = set()
    matches = []
    for _, index, object_id in candidates:
        if index in used_detections or object_id in used_objects:
            continue
        used_detections.add(index)
        used_objects.add(object_id)
        matches.append((index, object_id))
    return matches
