#!/usr/bin/env python3

import json
import re
import socket
import subprocess
import threading
import time


class TCPServer:
    def __init__(self, host="192.168.3.216", port=12345):
        self.host = host
        self.port = port
        self.server_socket = None
        self.clients = []

    @staticmethod
    def parse_service_success(output):
        text = output.decode("utf-8", errors="replace")
        match = re.search(
            r"(?i)\bsuccess\s*[:=]\s*(true|false|1|0)\b",
            text,
        )
        if not match:
            print(f"Cannot parse service success from output:\n{text}")
            return False
        return match.group(1).lower() in ("true", "1")

    @staticmethod
    def wait_for_service(service_name, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            check = subprocess.run(
                ["ros2", "service", "type", service_name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
            if check.returncode == 0:
                return True
            time.sleep(0.2)
        print(f"Service {service_name} is not available after {timeout:.1f}s")
        return False

    def call_service(self, service_name, service_type, request, retries=3):
        if not self.wait_for_service(service_name):
            return None

        command = [
            "ros2",
            "service",
            "call",
            service_name,
            service_type,
            request,
        ]
        for attempt in range(1, retries + 1):
            try:
                return subprocess.check_output(command, stderr=subprocess.STDOUT)
            except subprocess.CalledProcessError as error:
                detail = error.output.decode("utf-8", errors="replace").strip()
                print(
                    f"Service call {service_name} failed "
                    f"(attempt {attempt}/{retries}, "
                    f"returncode={error.returncode}):\n{detail}"
                )
            except OSError as error:
                print(
                    f"Cannot execute ros2 service for {service_name} "
                    f"(attempt {attempt}/{retries}): {error}"
                )

            if attempt < retries:
                time.sleep(0.5)
                if not self.wait_for_service(service_name, timeout=3.0):
                    break
        return None

    def start(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        print(f"Server started on {self.host}:{self.port}")
        while True:
            client_socket, address = self.server_socket.accept()
            print(f"Connection from {address}")
            client_thread = threading.Thread(
                target=self.handle_client, args=(client_socket,)
            )
            client_thread.start()

    def handle_client(self, client_socket):
        try:
            while True:
                data = client_socket.recv(1024).decode("utf-8")
                if not data:
                    break
                print(f"Received: {data}")
                request = json.loads(data)
                if request["cmd"] == 1:
                    self.handle_cmd_1(client_socket, request)
                elif request["cmd"] == 2:
                    self.handle_cmd_2(client_socket, request)
        except Exception as error:
            print(f"Error handling client: {error}")
        finally:
            client_socket.close()

    def handle_cmd_1(self, client_socket, request):
        point_id = request["id"]
        current_id = request["cid"]

        def execute():
            start_time = time.time()
            result = False

            def call_service():
                nonlocal result
                service_request = (
                    f"{{data: {int(point_id)}, current_id: {int(current_id)}, run: 1}}"
                )
                output = self.call_service(
                    "/plan_path_and_go",
                    "x2bot_teleop/srv/SetInt",
                    service_request,
                )
                if output is not None:
                    result = self.parse_service_success(output)

            service_thread = threading.Thread(target=call_service)
            service_thread.start()
            while service_thread.is_alive():
                elapsed_time = int(time.time() - start_time)
                response = {
                    "cmd": 1,
                    "id": point_id,
                    "cid": current_id,
                    "execute": elapsed_time,
                }
                client_socket.send(json.dumps(response).encode("utf-8"))
                time.sleep(0.5)

            response = {
                "cmd": 1,
                "id": point_id,
                "cid": current_id,
                "result": result,
            }
            client_socket.send(json.dumps(response).encode("utf-8"))

        threading.Thread(target=execute).start()

    def handle_cmd_2(self, client_socket, request):
        target_x = float(request.get("x", 0.0))
        target_y = float(request.get("y", request.get("id", 0.0)))
        target_angle = float(request.get("angle", request.get("angel", 0.0)))
        current_id = request.get("cid", 0)

        def execute():
            start_time = time.time()
            result = False

            def call_service():
                nonlocal result
                service_request = (
                    f"{{target_x: {target_x / 100}, "
                    f"target_y: {target_y / 100}, "
                    f"target_angle: {target_angle}}}"
                )
                output = self.call_service(
                    "/set_target_y",
                    "x2bot_teleop/srv/SetTagY",
                    service_request,
                )
                if output is not None:
                    result = self.parse_service_success(output)

            service_thread = threading.Thread(target=call_service)
            service_thread.start()
            while service_thread.is_alive():
                elapsed_time = int(time.time() - start_time)
                response = {
                    "cmd": 2,
                    "x": target_x,
                    "y": target_y,
                    "angle": target_angle,
                    "cid": current_id,
                    "execute": elapsed_time,
                }
                client_socket.send(json.dumps(response).encode("utf-8"))
                time.sleep(0.5)

            response = {
                "cmd": 2,
                "x": target_x,
                "y": target_y,
                "angle": target_angle,
                "cid": current_id,
                "result": result,
            }
            client_socket.send(json.dumps(response).encode("utf-8"))

        threading.Thread(target=execute).start()


def main():
    TCPServer().start()


if __name__ == "__main__":
    main()
