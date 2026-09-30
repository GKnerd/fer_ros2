"""Object catalog parsing and lookup."""
from fer_grasp_planner.core.catalog import catalog_from_dict, CatalogError
import pytest


def test_known_class_and_default():
    catalog = catalog_from_dict(
        {'default': {'force': 15.0}, 'classes': {'sphere': {'force': 10.0}}})
    assert catalog.force_for('sphere') == 10.0
    assert catalog.force_for('mug') == 15.0


def test_classes_are_optional():
    assert catalog_from_dict({'default': {'force': 12.0}}).force_for('box') == 12.0


@pytest.mark.parametrize('data', [
    None,
    {},
    {'classes': {'box': {'force': 15.0}}},
    {'default': {'force': 0.0}},
    {'default': {}},
    {'default': {'force': 15.0}, 'classes': {'box': {'force': -1.0}}},
    {'default': {'force': 15.0}, 'classes': {'box': {'force': 'strong'}}},
    {'default': {'force': 15.0}, 'classes': ['box']},
])
def test_malformed_catalog_is_rejected(data):
    with pytest.raises(CatalogError):
        catalog_from_dict(data)
