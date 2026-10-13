"""
Mock perception: publishes the detections of a YAML file at a fixed rate.

Stands in for a detector. Each message is stamped with the current time; the detections
carry class, score, center pose and size, never an object id.
"""
from __future__ import annotations

from pathlib import Path

from fer_world_model.adapters.ros_conversions import pose_to_msg
from fer_world_model.core.config import pose_from_config, size_from_config
from geometry_msgs.msg import Vector3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
import yaml


class MockPerception:

    def __init__(self, node: Node) -> None:

        self._node: Node = node
        self._node.declare_parameter('objects_file', '')
        self._node.declare_parameter('frame_id', 'base')
        self._node.declare_parameter('rate', 5.0)
        self._node.declare_parameter('detection_topic', '/perception/detections')

        self._frame_id = self._node.get_parameter('frame_id').value
        objects_file = self._node.get_parameter('objects_file').value
        if not objects_file:
            raise ValueError('parameter objects_file is required')
        with Path(objects_file).open() as f:
            self._detections = [
                self._to_detection(entry)
                for entry in (yaml.safe_load(f) or {}).get('detections') or []
            ]

        self._pub = self._node.create_publisher(
            Detection3DArray, self._node.get_parameter('detection_topic').value, 1)
        self._node.create_timer(1.0 / float(self._node.get_parameter('rate').value), self._publish)
        self._node.get_logger().info(
            f'publishing {len(self._detections)} detection(s) from {objects_file}')

    def _to_detection(self, entry: dict) -> Detection3D:
        detection = Detection3D()
        detection.header.frame_id = self._frame_id
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = entry['class_id']
        hypothesis.hypothesis.score = float(entry.get('score', 1.0))
        detection.results = [hypothesis]
        detection.bbox.center = pose_to_msg(pose_from_config(entry['pose']))
        size = size_from_config(entry['size'])
        detection.bbox.size = Vector3(x=size.x, y=size.y, z=size.z)
        return detection

    def _publish(self) -> None:
        msg = Detection3DArray()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        for detection in self._detections:
            detection.header.stamp = msg.header.stamp
        msg.detections = self._detections
        self._pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    node = Node('fer_mock_perception')
    MockPerception(node=node)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
