from fer_world_model.core.association import associate
from fer_world_model.core.world_object import Detection, Point, Pose, Size, WorldObject

SIZE = Size(0.05, 0.05, 0.05)


def pose(x: float, y: float = 0.0, z: float = 0.0) -> Pose:
    return Pose(position=Point(x, y, z))


def detection(class_id: str, x: float) -> Detection:
    return Detection(class_id=class_id, score=1.0, pose=pose(x), size=SIZE)


def obj(object_id: str, class_id: str, x: float) -> WorldObject:
    return WorldObject(
        object_id=object_id, class_id=class_id, size=SIZE, pose=pose(x), frame='base')


def test_nearest_pairs_win():
    detections = [detection('box', 0.02), detection('box', 0.0)]
    objects = [obj('box_1', 'box', 0.0), obj('box_2', 'box', 0.04)]
    assert sorted(associate(detections, objects, gate=0.05)) == [(0, 'box_2'), (1, 'box_1')]


def test_class_must_match():
    assert associate([detection('sphere', 0.0)], [obj('box_1', 'box', 0.0)], gate=0.05) == []


def test_gate_excludes_far_detections():
    assert associate([detection('box', 0.06)], [obj('box_1', 'box', 0.0)], gate=0.05) == []


def test_each_object_used_once():
    detections = [detection('box', 0.0), detection('box', 0.01)]
    assert associate(detections, [obj('box_1', 'box', 0.0)], gate=0.05) == [(0, 'box_1')]
