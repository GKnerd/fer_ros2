from fer_world_model.core.config import fixtures_from_config
from fer_world_model.core.result import WMStatus
from fer_world_model.core.world_model import WorldModel
from fer_world_model.core.world_object import (
    Detection,
    ObjectStatus,
    Point,
    Pose,
    Size,
)
import pytest

SIZE = Size(0.05, 0.05, 0.05)
TABLE = {
    'fixtures': [{
        'id': 'table', 'size': [1.0, 1.0, 0.02],
        'pose': {'position': {'x': 0.3, 'y': 0.0, 'z': -0.01}},
    }],
}


def pose(x: float, y: float = 0.0, z: float = 0.0) -> Pose:
    return Pose(position=Point(x, y, z))


def detection(class_id: str, x: float, y: float = 0.0, score: float = 0.9) -> Detection:
    return Detection(class_id=class_id, score=score, pose=pose(x, y), size=SIZE)


def detect(model: WorldModel, detections, stamp: float = 1.0, classes=(), grasped=()):
    return model.apply_detections(detections, list(classes), list(grasped), stamp, 'test')


@pytest.fixture
def model() -> WorldModel:
    world_model = WorldModel(min_score=0.5)
    world_model.load_fixtures(fixtures_from_config(TABLE, 'base'))
    return world_model


def test_new_detections_get_class_ids(model):
    update = detect(model, [detection('box', 0.4), detection('box', 0.6), detection('ball', 0.5)])
    assert sorted(update.added) == ['ball_1', 'box_1', 'box_2']
    assert update.updated == [] and update.not_seen == []


def test_ids_are_never_reused(model):
    detect(model, [detection('box', 0.4)], stamp=1.0)
    detect(model, [], stamp=2.0)
    assert model.sweep(now=40.0, removal_timeout=30.0) == ['box_1']
    assert detect(model, [detection('box', 0.4)], stamp=41.0).added == ['box_2']


def test_repeated_detection_updates_pose_size_and_stamp(model):
    detect(model, [detection('box', 0.40)], stamp=1.0)
    moved = Detection(class_id='box', score=0.8, pose=pose(0.43), size=Size(0.06, 0.05, 0.05))
    update = detect(model, [moved], stamp=2.0)
    assert update.added == [] and update.updated == ['box_1']
    box = model.query(['box_1'], [], [], False)[0]
    assert box.pose.position.x == pytest.approx(0.43)
    assert box.size == Size(0.06, 0.05, 0.05)
    assert box.last_observed == 2.0 and box.source == 'test' and box.score == 0.8


def test_low_scores_are_ignored(model):
    assert detect(model, [detection('box', 0.4, score=0.3)]).added == []


def test_class_filter(model):
    detect(model, [detection('box', 0.4), detection('ball', 0.6)], stamp=1.0)
    update = detect(model, [detection('ball', 0.6)], stamp=2.0, classes=['ball'])
    assert update.updated == ['ball_1'] and update.not_seen == []


def test_not_seen_keeps_the_object_unchanged(model):
    detect(model, [detection('box', 0.4)], stamp=1.0)
    revision = model.revision
    update = detect(model, [], stamp=2.0)
    assert update.not_seen == ['box_1']
    box = model.query(['box_1'], [], [], False)[0]
    assert box.status is ObjectStatus.FREE and box.pose.position.x == pytest.approx(0.4)
    assert model.revision == revision


def test_fixtures_are_never_matched_or_reported(model):
    update = detect(model, [detection('table', 0.3)])
    assert update.added == ['table_1']
    assert detect(model, [], stamp=2.0).not_seen == ['table_1']
    table = model.query(['table'], [], [], True)[0]
    assert table.fixed and table.source == 'manual'


def test_held_object_seen_in_hand_is_ignored(model):
    detect(model, [detection('box', 0.4)], stamp=1.0)
    grasp = model.set_status('box_1', ObjectStatus.GRASPED, 'fer_hand_tcp', pose(0.0),
                             'fer_hand_tcp', 2.0)
    assert grasp.ok
    in_hand = Point(0.3, 0.2, 0.3)
    seen_in_hand = Detection(class_id='box', score=0.9, pose=pose(0.3, 0.2, 0.31), size=SIZE)
    update = detect(model, [seen_in_hand], stamp=3.0, grasped=[in_hand])
    assert update.added == []
    elsewhere = detect(model, [detection('box', 0.3, 0.2)], stamp=4.0, grasped=[in_hand])
    assert elsewhere.added == ['box_2']


def test_grasped_objects_are_never_changed_by_detection(model):
    detect(model, [detection('box', 0.4)], stamp=1.0)
    model.set_status('box_1', ObjectStatus.GRASPED, 'fer_hand_tcp', pose(0.0),
                     'fer_hand_tcp', 2.0)
    update = detect(model, [detection('box', 0.4)], stamp=3.0)
    assert update.added == ['box_2'] and update.not_seen == []
    held = model.query(['box_1'], [], [], False)[0]
    assert held.status is ObjectStatus.GRASPED and held.frame == 'fer_hand_tcp'


def test_lost_object_is_adopted_by_an_unmatched_detection(model):
    detect(model, [detection('box', 0.4)], stamp=1.0)
    model.set_status('box_1', ObjectStatus.LOST, '', pose(0.4), 'base', 2.0)
    update = detect(model, [detection('box', 0.7)], stamp=3.0)
    assert update.added == [] and update.updated == ['box_1']
    box = model.query(['box_1'], [], [], False)[0]
    assert box.status is ObjectStatus.FREE and box.pose.position.x == pytest.approx(0.7)


def test_set_status_invariants(model):
    detect(model, [detection('box', 0.4)], stamp=1.0)
    assert model.set_status('nope', ObjectStatus.FREE, '', pose(0.4), 'base', 2.0).status \
        is WMStatus.NOT_FOUND
    assert model.set_status('table', ObjectStatus.LOST, '', pose(0.4), 'base', 2.0).status \
        is WMStatus.INVALID
    assert model.set_status('box_1', ObjectStatus.GRASPED, '', pose(0.0), 'base', 2.0).status \
        is WMStatus.INVALID
    assert model.set_status('box_1', ObjectStatus.GRASPED, 'fer_hand_tcp', pose(0.0), 'base',
                            2.0).status is WMStatus.INVALID
    assert model.set_status('box_1', ObjectStatus.FREE, 'fer_hand_tcp', pose(0.4), 'base',
                            2.0).status is WMStatus.INVALID


def test_release_marks_the_estimate(model):
    detect(model, [detection('box', 0.4)], stamp=1.0)
    model.set_status('box_1', ObjectStatus.GRASPED, 'fer_hand_tcp', pose(0.0),
                     'fer_hand_tcp', 2.0)
    assert model.set_status('box_1', ObjectStatus.FREE, '', pose(0.6), 'base', 3.0).ok
    box = model.query(['box_1'], [], [], False)[0]
    assert box.source == 'release_estimate' and box.last_observed == 1.0


def test_every_change_raises_the_revision(model):
    start = model.revision
    detect(model, [detection('box', 0.4)], stamp=1.0)
    after_detect = model.revision
    model.set_status('box_1', ObjectStatus.LOST, '', pose(0.4), 'base', 2.0)
    assert start < after_detect < model.revision


def test_refine(model):
    detect(model, [detection('box', 0.4)], stamp=1.0)
    assert model.refine('nope', [], 2.0, 'test').status is WMStatus.NOT_FOUND
    assert model.refine('table', [], 2.0, 'test').status is WMStatus.CONFLICT
    refined = model.refine('box_1', [detection('box', 0.47), detection('box', 0.43)], 2.0, 'test')
    assert refined.ok
    assert model.query(['box_1'], [], [], False)[0].pose.position.x == pytest.approx(0.43)


def test_refine_miss_starts_the_removal_clock(model):
    detect(model, [detection('box', 0.4)], stamp=1.0)
    assert model.refine('box_1', [detection('box', 0.6)], 2.0, 'test').status \
        is WMStatus.NOT_FOUND
    assert model.sweep(now=33.0, removal_timeout=30.0) == ['box_1']


def test_refine_on_grasped_is_a_conflict(model):
    detect(model, [detection('box', 0.4)], stamp=1.0)
    model.set_status('box_1', ObjectStatus.GRASPED, 'fer_hand_tcp', pose(0.0),
                     'fer_hand_tcp', 2.0)
    assert model.refine('box_1', [], 3.0, 'test').status is WMStatus.CONFLICT


def test_no_removal_without_a_miss(model):
    detect(model, [detection('box', 0.4)], stamp=1.0)
    assert model.sweep(now=1000.0, removal_timeout=30.0) == []


def test_removal_after_timeout_only(model):
    detect(model, [detection('box', 0.4)], stamp=1.0)
    detect(model, [], stamp=10.0)
    assert model.sweep(now=39.0, removal_timeout=30.0) == []
    revision = model.revision
    assert model.sweep(now=41.0, removal_timeout=30.0) == ['box_1']
    assert model.query(['box_1'], [], [], False) == []
    assert model.revision == revision + 1


def test_seen_again_stops_the_clock(model):
    detect(model, [detection('box', 0.4)], stamp=1.0)
    detect(model, [], stamp=10.0)
    detect(model, [detection('box', 0.4)], stamp=20.0)
    assert model.sweep(now=100.0, removal_timeout=30.0) == []


def test_lost_objects_are_removed_after_timeout(model):
    detect(model, [detection('box', 0.4)], stamp=1.0)
    model.set_status('box_1', ObjectStatus.LOST, '', pose(0.4), 'base', 5.0)
    assert model.sweep(now=36.0, removal_timeout=30.0) == ['box_1']


def test_grasped_and_fixed_are_never_removed(model):
    detect(model, [detection('box', 0.4)], stamp=1.0)
    detect(model, [], stamp=2.0)
    model.set_status('box_1', ObjectStatus.GRASPED, 'fer_hand_tcp', pose(0.0),
                     'fer_hand_tcp', 3.0)
    assert model.sweep(now=1000.0, removal_timeout=30.0) == []
    assert len(model.query([], [], [], True)) == 2


def test_query_filters(model):
    detect(model, [detection('box', 0.4), detection('ball', 0.6)], stamp=1.0)
    model.set_status('ball_1', ObjectStatus.LOST, '', pose(0.6), 'base', 2.0)
    assert [o.object_id for o in model.query([], ['box'], [], False)] == ['box_1']
    lost = model.query([], [], [int(ObjectStatus.LOST)], False)
    assert [o.object_id for o in lost] == ['ball_1']
    assert model.query(['box_1'], ['ball'], [], False) == []
    assert len(model.query([], [], [], False)) == 2
    assert len(model.query([], [], [], True)) == 3
