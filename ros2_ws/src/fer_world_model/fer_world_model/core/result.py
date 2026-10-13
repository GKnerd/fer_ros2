"""
Return type of the WorldModel operations.

Every mutating WorldModel method returns a WMResult instead of raising or returning a
bare bool, so callers get a machine-readable status and a human message. The node maps
it onto fer_interfaces/Outcome.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class WMStatus(Enum):
    OK = 'ok'
    NOT_FOUND = 'not_found'            # no object with that id
    INVALID = 'invalid'                # malformed request (bad frame, missing held_by, ...)
    CONFLICT = 'conflict'              # op contradicts current state (e.g. refine a GRASPED)


@dataclass(frozen=True)
class WMResult:
    status: WMStatus
    message: str = ''
    key: str | None = None  # object id the operation concerned, when relevant

    @property
    def ok(self) -> bool:
        return self.status is WMStatus.OK

    def __bool__(self) -> bool:
        return self.ok

    @classmethod
    def success(cls, key: str | None = None, message: str = '') -> WMResult:
        return cls(WMStatus.OK, message or 'ok', key)

    @classmethod
    def not_found(cls, key: str) -> WMResult:
        return cls(WMStatus.NOT_FOUND, f"no object with id '{key}'", key)

    @classmethod
    def invalid(cls, message: str, key: str | None = None) -> WMResult:
        return cls(WMStatus.INVALID, message, key)

    @classmethod
    def conflict(cls, message: str, key: str | None = None) -> WMResult:
        return cls(WMStatus.CONFLICT, message, key)
