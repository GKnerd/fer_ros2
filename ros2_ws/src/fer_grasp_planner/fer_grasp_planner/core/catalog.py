"""Grasp force per object class, with a default for classes not listed."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


class CatalogError(ValueError):
    """The object catalog is malformed."""


@dataclass(frozen=True)
class Catalog:
    default_force: float
    forces: dict[str, float] = field(default_factory=dict)

    def force_for(self, class_id: str) -> float:
        return self.forces.get(class_id, self.default_force)


def catalog_from_dict(data: Any) -> Catalog:
    """
    Build a Catalog from the parsed object_catalog.yaml.

    Raises CatalogError when `default` is missing or a force is not positive.
    """
    if not isinstance(data, dict) or not isinstance(data.get('default'), dict):
        raise CatalogError("catalog needs a 'default' entry")
    default_force = _force('default', data['default'])
    classes = data.get('classes') or {}
    if not isinstance(classes, dict):
        raise CatalogError("'classes' must map class ids to entries")
    return Catalog(
        default_force=default_force,
        forces={str(name): _force(name, entry) for name, entry in classes.items()})


def _force(name: str, entry: Any) -> float:
    try:
        force = float(entry['force'])
    except (TypeError, KeyError, ValueError):
        raise CatalogError(f"'{name}' needs a numeric 'force'") from None
    if force <= 0.0:
        raise CatalogError(f"'{name}': force must be positive, got {force}")
    return force
