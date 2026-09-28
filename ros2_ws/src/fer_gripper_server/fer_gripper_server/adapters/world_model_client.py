"""Blocking client for the world model services of fer_interfaces."""
from __future__ import annotations

from fer_gripper_server.adapters.actions import wait_for
from fer_interfaces.msg import Outcome, WorldObject
from fer_interfaces.srv import QueryObjects, SetObjectStatus
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.node import Node


class WorldModelError(RuntimeError):
    """The world model did not answer."""


class WorldModelClient:

    def __init__(self, node: Node, callback_group: CallbackGroup) -> None:
        node.declare_parameter('world_model_timeout', 2.0)
        self._timeout = float(node.get_parameter('world_model_timeout').value)
        self._query = node.create_client(
            QueryObjects, '/world_model/query_objects', callback_group=callback_group)
        self._set_status = node.create_client(
            SetObjectStatus, '/world_model/set_object_status', callback_group=callback_group)

    def get(self, object_id: str) -> WorldObject | None:
        response = self._call(
            self._query, QueryObjects.Request(ids=[object_id], include_fixed=True))
        return response.objects[0] if response.objects else None

    def held_by(self, link: str) -> list[WorldObject]:
        response = self._call(
            self._query, QueryObjects.Request(statuses=[WorldObject.GRASPED]))
        return [o for o in response.objects if o.held_by == link]

    def set_status(
        self, object_id: str, status: int, held_by: str, pose: PoseStamped
    ) -> Outcome:
        request = SetObjectStatus.Request(id=object_id, status=status, held_by=held_by, pose=pose)
        return self._call(self._set_status, request).outcome

    def _call(self, client: Client, request):
        if not client.service_is_ready():
            raise WorldModelError(f"world model service '{client.srv_name}' not available")
        future = client.call_async(request)
        if not wait_for(future, self._timeout):
            raise WorldModelError(f"no answer from '{client.srv_name}'")
        return future.result()
