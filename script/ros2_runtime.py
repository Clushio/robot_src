#!/usr/bin/env python3
"""Small ROS2/Qt runtime helpers shared by the operator utilities."""

import os
import re
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions


def init_rclpy_once(args=None):
    if not rclpy.ok():
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)


def wait_future(future, timeout):
    deadline = time.monotonic() + max(0.0, float(timeout))
    while not future.done() and rclpy.ok() and time.monotonic() < deadline:
        time.sleep(0.01)
    if not future.done():
        return None
    return future.result()


def service_is_ready(node, service_name):
    return any(
        name == service_name
        for name, _types in node.get_service_names_and_types()
    )


def node_names(node):
    return {
        f'{namespace.rstrip("/")}/{name}'.replace('//', '/')
        for name, namespace in node.get_node_names_and_namespaces()
    }


def parse_service_success(output):
    match = re.search(
        r'\bsuccess\s*[:=]\s*(true|false|True|False|1|0)\b',
        output or '',
    )
    return bool(match and match.group(1).lower() in ('true', '1'))


def parse_service_message(output):
    match = re.search(
        r"\bmessage\s*[:=]\s*(['\"])(.*?)\1",
        output or '', re.DOTALL,
    )
    return match.group(2) if match else ''


class Ros2Runtime:
    def __init__(self, node_name='upmachine_publisher'):
        init_rclpy_once()
        unique_name = f'{node_name}_{os.getpid()}'
        self.node = rclpy.create_node(unique_name)
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(
            target=self.executor.spin,
            name=f'{unique_name}_executor',
            daemon=True,
        )
        self.thread.start()

    def shutdown(self):
        self.executor.shutdown(timeout_sec=1.0)
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
