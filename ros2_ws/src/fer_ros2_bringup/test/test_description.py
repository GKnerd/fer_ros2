"""Description invariants for fer_ros2_bringup.

fer.urdf.xacro must produce exactly the upstream franka_description model for
every hardware backend, plus exactly one backend overlay whose joints exist in
that model. The controller configuration must only name joints of the model.
"""
import os

import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from lxml import etree

BRINGUP = get_package_share_directory("fer_ros2_bringup")
UPSTREAM = os.path.join(
    get_package_share_directory("franka_description"), "robots", "fer", "fer.urdf.xacro"
)
WRAPPER = os.path.join(BRINGUP, "urdf", "fer.urdf.xacro")
ROBOT_IP = "192.0.2.1"

OVERLAY_TAGS = ("ros2_control", "mujoco_inputs")
PLUGINS = {
    "real": "franka_hardware/FrankaMultiHardwareInterface",
    "mujoco": "mujoco_ros2_control/MujocoSystemInterface",
}


def expand(path, **mappings):
    mappings = {k: str(v) for k, v in mappings.items()}
    return etree.fromstring(xacro.process_file(path, mappings=mappings).toxml().encode())


def model_only(root):
    """Canonical form of the kinematic/dynamic model: overlays, comments and whitespace removed."""
    root = etree.fromstring(etree.tostring(root))
    for tag in OVERLAY_TAGS:
        for element in root.findall(tag):
            root.remove(element)
    for comment in root.xpath("//comment()"):
        comment.getparent().remove(comment)
    for element in root.iter():
        if element.text is not None and not element.text.strip():
            element.text = None
        if element.tail is not None and not element.tail.strip():
            element.tail = None
    return etree.tostring(root, method="c14n")


def model_joints(root):
    return {joint.get("name") for joint in root.findall("joint")}


@pytest.fixture(scope="module")
def upstream():
    return expand(UPSTREAM, robot_ip=ROBOT_IP)


@pytest.mark.parametrize("hardware", ["none", "real", "mujoco"])
def test_model_matches_upstream(upstream, hardware):
    wrapped = expand(WRAPPER, hardware=hardware, robot_ip=ROBOT_IP)
    assert model_only(wrapped) == model_only(upstream)


@pytest.mark.parametrize("hand", ["true", "false"])
@pytest.mark.parametrize("hardware", ["real", "mujoco"])
def test_model_matches_upstream_hand(hardware, hand):
    upstream = expand(UPSTREAM, robot_ip=ROBOT_IP, hand=hand)
    wrapped = expand(WRAPPER, hardware=hardware, robot_ip=ROBOT_IP, hand=hand)
    assert model_only(wrapped) == model_only(upstream)


def test_none_has_no_overlay():
    root = expand(WRAPPER, hardware="none")
    for tag in OVERLAY_TAGS:
        assert root.findall(tag) == []


@pytest.mark.parametrize("hardware", ["real", "mujoco"])
def test_exactly_one_backend(hardware):
    root = expand(WRAPPER, hardware=hardware, robot_ip=ROBOT_IP)
    blocks = root.findall("ros2_control")
    assert len(blocks) == 1
    assert blocks[0].get("name") == "fer_hardware"
    assert blocks[0].findtext("hardware/plugin").strip() == PLUGINS[hardware]
    assert len(root.findall("mujoco_inputs")) == (1 if hardware == "mujoco" else 0)


@pytest.mark.parametrize("hardware", ["real", "mujoco"])
def test_overlay_joints_exist(hardware):
    root = expand(WRAPPER, hardware=hardware, robot_ip=ROBOT_IP)
    joints = model_joints(root)
    for joint in root.findall("ros2_control/joint"):
        assert joint.get("name") in joints


def test_real_overlay_contract():
    root = expand(WRAPPER, hardware="real", robot_ip=ROBOT_IP)
    block = root.find("ros2_control")
    params = {p.get("name"): (p.text or "").strip() for p in block.findall("hardware/param")}
    assert params == {"robot_count": "1", "ns1": "fer", "robot_ip1": ROBOT_IP}
    joints = block.findall("joint")
    assert [j.get("name") for j in joints] == [f"fer_joint{i}" for i in range(1, 8)]
    for joint in joints:
        assert [c.get("name") for c in joint.findall("command_interface")] == ["effort", "position", "velocity"]
        assert [s.get("name") for s in joint.findall("state_interface")] == ["position", "velocity", "effort"]


@pytest.mark.parametrize("control_type", ["effort", "position"])
@pytest.mark.parametrize("hand_control_type", ["effort", "position"])
def test_mujoco_actuators_unique(control_type, hand_control_type):
    root = expand(
        WRAPPER,
        hardware="mujoco",
        mujoco_control_type=control_type,
        hand_control_type=hand_control_type,
    )
    joints = model_joints(root)
    actuators = root.findall("mujoco_inputs/raw_inputs/actuator/*")
    actuated = [a.get("joint") for a in actuators]
    assert len(actuated) == len(set(actuated)) == 8
    assert set(actuated) <= joints
    assert len(root.findall("mujoco_inputs/raw_inputs/equality/joint")) == 1
    for joint in root.findall("ros2_control/joint"):
        commands = [c.get("name") for c in joint.findall("command_interface")]
        expected = hand_control_type if "finger" in joint.get("name") else control_type
        assert commands == [expected]


@pytest.mark.parametrize("mappings", [{"hardware": "gazebo"}, {"hardware": "real", "ros2_control": "true"}])
def test_invalid_arguments_rejected(mappings):
    with pytest.raises(Exception):
        expand(WRAPPER, robot_ip=ROBOT_IP, **mappings)


@pytest.mark.parametrize(
    "files",
    [
        ["fer_controllers.yaml", "fer_controllers_real.yaml"],
        ["fer_controllers.yaml", "fer_controllers_gripper.yaml"],
    ],
)
def test_controller_joints_exist(files):
    joints = model_joints(expand(WRAPPER, hardware="none"))
    declared = set()
    for name in files:
        with open(os.path.join(BRINGUP, "config", name)) as f:
            config = yaml.safe_load(f)
        declared |= set(config["controller_manager"]["ros__parameters"]) - {"update_rate"}
        for controller, section in config.items():
            if controller == "controller_manager":
                continue
            params = section["ros__parameters"]
            for joint in params.get("joints", []) + ([params["joint"]] if "joint" in params else []):
                assert joint in joints, f"{controller}: unknown joint {joint}"
            assert controller in declared, f"{controller} has parameters but no type"
