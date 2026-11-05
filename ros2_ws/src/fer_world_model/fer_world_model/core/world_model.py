"""
World model: objects built from detections and gripper reports.

Writers are detection (`apply_detections`, `refine`), the gripper server (`set_status`)
and the removal sweep. Every change raises `revision`.
"""
from __future__ import annotations

import copy
import dataclasses
from dataclasses import dataclass
from typing import (
    Dict,
    Iterable,
    List,
    Sequence,
)

from fer_world_model.core.association import associate, distance
from fer_world_model.core.result import WMResult, WMStatus
from fer_world_model.core.world_object import (
    Detection,
    ObjectStatus,
    Point,
    Pose,
    WorldObject,
)


@dataclass(frozen=True)
class DetectionUpdate:
    added: List[str]
    updated: List[str]
    not_seen: List[str]


class WorldModel:

    def __init__(
        self,
        base_frame: str = 'base',
        association_gate: float = 0.05,
        refine_gate: float = 0.10,
        min_score: float = 0.0,
        lost_after_misses: int = 1,
    ) -> None:
        """
        Create an empty world model.

        :param lost_after_misses: A FREE object missed by this many snapshots in a row
            (detection or refine) becomes LOST: it leaves the planning scene and an
            unmatched detection of its class can take it over. 1 for mock perception, which
            sees every object; more for a real camera, so one bad frame does not drop a real
            object. 0: never LOST from misses; a missed object stays FREE until the sweep.
        """
        self.base_frame = base_frame
        self.association_gate = association_gate
        self.refine_gate = refine_gate
        self.min_score = min_score
        self.lost_after_misses = lost_after_misses
        self._objects: Dict[str, WorldObject] = {}
        self._next_index: Dict[str, int] = {}
        self._revision = 0

    @property
    def revision(self) -> int:
        return self._revision

    def load_fixtures(self, fixtures: Iterable[WorldObject]) -> None:
        """
        Add fixed objects; they are never matched, changed or removed.

        Raises ValueError on an invalid fixture or a duplicate id.
        """
        for fixture in fixtures:
            fixture.fixed = True
            fixture.validate(self.base_frame)
            if fixture.object_id in self._objects:
                raise ValueError(f"duplicate fixture id '{fixture.object_id}'")
            self._objects[fixture.object_id] = fixture
            self._revision += 1

    def apply_detections(
        self,
        detections: Sequence[Detection],
        class_ids: Sequence[str],
        grasped_positions: Sequence[Point],
        stamp: float,
        source: str,
    ) -> DetectionUpdate:
        """
        Fold one detection snapshot into the model.

        Returns the ids added, updated (matched or adopted from LOST) and not seen. A FREE
        object not seen counts a miss and becomes LOST after `lost_after_misses`; it can
        then be taken over by an unmatched detection of the same snapshot (reported as
        updated, not as not seen).

        :param detections: Detections in the base frame.
        :param class_ids: Only these classes are considered; empty means all.
        :param grasped_positions: Current base-frame centers of GRASPED objects. Detections
            this close are the held object seen in the hand and are dropped.
        :param stamp: Capture time of the snapshot, s.
        :param source: Recorded as the `source` of every object the snapshot touches.
        """
        usable = [
            d for d in self._usable(detections, class_ids)
            if all(distance(d.pose.position, p) > self.association_gate
                   for p in grasped_positions)
        ]
        candidates = [
            o for o in self._objects.values()
            if o.status is ObjectStatus.FREE and not o.fixed
            and self._in_classes(o.class_id, class_ids)
        ]
        matches = associate(usable, candidates, self.association_gate)
        matched_detections = {index for index, _ in matches}
        matched_ids = {object_id for _, object_id in matches}

        updated = []
        for index, object_id in matches:
            self._observe(self._objects[object_id], usable[index], stamp, source)
            updated.append(object_id)

        # Before the takeover below, so an object found elsewhere in this snapshot keeps its id.
        not_seen = []
        went_lost = False
        for obj in candidates:
            if obj.object_id in matched_ids:
                continue
            not_seen.append(obj.object_id)
            went_lost = self._miss(obj, stamp) or went_lost

        lost = sorted(
            (o for o in self._objects.values()
             if o.status is ObjectStatus.LOST and self._in_classes(o.class_id, class_ids)),
            key=lambda o: o.missing_since if o.missing_since is not None else float('inf'),
        )
        added = []
        for index, detection in enumerate(usable):
            if index in matched_detections:
                continue
            adopted = next((o for o in lost if o.class_id == detection.class_id), None)
            if adopted is not None:
                lost.remove(adopted)
                adopted.status = ObjectStatus.FREE
                self._observe(adopted, detection, stamp, source)
                updated.append(adopted.object_id)
                continue
            obj = WorldObject(
                object_id=self._new_id(detection.class_id),
                class_id=detection.class_id,
                size=detection.size,
                pose=Pose(),
                frame=self.base_frame,
            )
            self._observe(obj, detection, stamp, source)
            self._objects[obj.object_id] = obj
            added.append(obj.object_id)

        not_seen = [object_id for object_id in not_seen if object_id not in updated]
        if added or updated or went_lost:
            self._revision += 1
        return DetectionUpdate(added=added, updated=updated, not_seen=not_seen)

    def refine(
        self,
        object_id: str,
        detections: Sequence[Detection],
        stamp: float,
        source: str,
    ) -> WMResult:
        """
        Update one object from a close-up snapshot.

        A miss counts like a miss in `apply_detections`: it starts the removal clock and
        counts towards `lost_after_misses`.
        Returns OK; NOT_FOUND for an unknown or unseen object; CONFLICT for a fixed or
        GRASPED one.

        :param object_id: Object to refine.
        :param detections: Detections in the base frame.
        :param stamp: Capture time of the snapshot, s.
        :param source: Recorded as the object's `source` on success.
        """
        obj = self._objects.get(object_id)
        if obj is None:
            return WMResult.not_found(object_id)
        if obj.fixed or obj.status is ObjectStatus.GRASPED:
            return WMResult.conflict(
                f"'{object_id}' is {'fixed' if obj.fixed else 'GRASPED'}", object_id)

        same_class = [
            d for d in self._usable(detections, [obj.class_id])
            if distance(d.pose.position, obj.pose.position) <= self.refine_gate
        ]
        if not same_class:
            if self._miss(obj, stamp):
                self._revision += 1
                return WMResult(
                    WMStatus.NOT_FOUND, f"'{object_id}' not seen, now LOST", object_id)
            return WMResult(WMStatus.NOT_FOUND, f"'{object_id}' not seen", object_id)

        nearest = min(same_class, key=lambda d: distance(d.pose.position, obj.pose.position))
        obj.status = ObjectStatus.FREE
        self._observe(obj, nearest, stamp, source)
        self._revision += 1
        return WMResult.success(object_id, f"'{object_id}' refined")

    def query(
        self,
        ids: Sequence[str],
        class_ids: Sequence[str],
        statuses: Sequence[int],
        include_fixed: bool,
    ) -> list[WorldObject]:
        """
        Return copies of the matching objects.

        Filters combine with AND, entries within one filter with OR; an empty filter
        does not restrict.
        """
        return [
            copy.deepcopy(o) for o in self._objects.values()
            if (include_fixed or not o.fixed)
            and (not ids or o.object_id in ids)
            and (not class_ids or o.class_id in class_ids)
            and (not statuses or int(o.status) in statuses)
        ]

    def set_status(
        self,
        object_id: str,
        status: ObjectStatus,
        held_by: str,
        pose: Pose,
        frame: str,
        stamp: float,
    ) -> WMResult:
        """
        Set status and pose, as reported by the gripper server.

        Returns OK; NOT_FOUND for an unknown id; INVALID for a fixed object or a
        status/frame combination that violates the invariants.

        :param object_id: Object to change.
        :param status: New status.
        :param held_by: Link holding the object; required for GRASPED, empty otherwise.
        :param pose: New pose; relative to `held_by` when GRASPED, else in the base frame.
        :param frame: Frame of `pose`.
        :param stamp: Current time, s; starts the removal clock for LOST.
        """
        obj = self._objects.get(object_id)
        if obj is None:
            return WMResult.not_found(object_id)
        if obj.fixed:
            return WMResult.invalid(f"'{object_id}' is fixed", object_id)

        changed = dataclasses.replace(
            obj, status=status, held_by=held_by, pose=copy.deepcopy(pose), frame=frame)
        try:
            changed.validate(self.base_frame)
        except ValueError as exc:
            return WMResult.invalid(str(exc), object_id)

        if status is ObjectStatus.FREE:
            changed.source = 'release_estimate'
            changed.missing_since = None
            changed.misses = 0
        elif status is ObjectStatus.LOST:
            changed.missing_since = stamp
        else:
            changed.missing_since = None
            changed.misses = 0

        self._objects[object_id] = changed
        self._revision += 1
        return WMResult.success(object_id, f"'{object_id}' is {status.name}")

    def sweep(self, now: float, removal_timeout: float) -> List[str]:
        """Remove FREE and LOST objects missing for longer than `removal_timeout` s."""
        removed = [
            o.object_id for o in self._objects.values()
            if not o.fixed
            and o.status in (ObjectStatus.FREE, ObjectStatus.LOST)
            and o.missing_since is not None
            and now - o.missing_since > removal_timeout
        ]
        for object_id in removed:
            del self._objects[object_id]
        if removed:
            self._revision += 1
        return removed

    def snapshot(self) -> list[WorldObject]:
        return [copy.deepcopy(o) for o in self._objects.values()]

    def _usable(
        self, detections: Iterable[Detection], class_ids: Sequence[str]
    ) -> List[Detection]:
        return [
            d for d in detections
            if d.score >= self.min_score and self._in_classes(d.class_id, class_ids)
        ]

    @staticmethod
    def _in_classes(class_id: str, class_ids: Sequence[str]) -> bool:
        return not class_ids or class_id in class_ids

    def _observe(
        self, obj: WorldObject, detection: Detection, stamp: float, source: str
    ) -> None:
        obj.pose = copy.deepcopy(detection.pose)
        obj.size = detection.size
        obj.frame = self.base_frame
        obj.score = detection.score
        obj.source = source
        obj.last_observed = stamp
        obj.missing_since = None
        obj.misses = 0

    def _miss(self, obj: WorldObject, stamp: float) -> bool:
        """Count a snapshot that missed `obj`; return True if it became LOST."""
        if obj.missing_since is None:
            obj.missing_since = stamp
        obj.misses += 1
        if (self.lost_after_misses > 0 and obj.misses >= self.lost_after_misses
                and obj.status is ObjectStatus.FREE):
            obj.status = ObjectStatus.LOST
            return True
        return False

    def _new_id(self, class_id: str) -> str:
        index = self._next_index.get(class_id, 0)
        while True:
            index += 1
            object_id = f'{class_id}_{index}'
            if object_id not in self._objects:
                break
        self._next_index[class_id] = index
        return object_id
