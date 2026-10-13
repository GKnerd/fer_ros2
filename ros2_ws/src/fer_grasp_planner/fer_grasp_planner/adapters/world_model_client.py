"""Blocking client for the world model's QueryObjects service."""
from __future__ import annotations

import threading

from fer_interfaces.msg import WorldObject
from fer_interfaces.srv import QueryObjects
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from rclpy.task import Future


class WorldModelError(RuntimeError):
    """The world model did not answer."""


class WorldModelClient:

    def __init__(self, node: Node, callback_group: CallbackGroup) -> None:
        node.declare_parameter('world_model_timeout', 2.0)
        self._timeout = float(node.get_parameter('world_model_timeout').value)
        self._query = node.create_client(
            QueryObjects, '/world_model/query_objects', callback_group=callback_group)

    def get(self, object_id: str) -> WorldObject | None:
        response = self._call(
            self._query, QueryObjects.Request(ids=[object_id], include_fixed=True))
        return response.objects[0] if response.objects else None

    def _call(
        self, client: Client, request: QueryObjects.Request
    ) -> QueryObjects.Response:
        if not client.service_is_ready():
            raise WorldModelError(f"world model service '{client.srv_name}' not available")
        future = client.call_async(request)
        if not _wait(future, self._timeout):
            raise WorldModelError(f"no answer from '{client.srv_name}'")
        return future.result()


def _wait(future: Future, timeout: float) -> bool:
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    return done.wait(timeout)
