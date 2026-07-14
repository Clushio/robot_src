import socket
import threading
import time
import json
import subprocess
import re


class TCPServer:
    def __init__(self, host='192.168.3.216', port=12345):
        # 192.168.1.216 172.20.103.212
        self.host = host
        self.port = port
        self.server_socket = None
        self.clients = []

    def parse_rosservice_success(self, output):
        text = output.decode('utf-8', errors='replace')
        match = re.search(
            r'(?m)^\s*success:\s*(true|false|True|False|1|0)\s*$',
            text
        )
        if not match:
            print(f"Cannot parse service success from output:\n{text}")
            return False
        return match.group(1).lower() in ('true', '1')

    def wait_for_service(self, service_name, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            check = subprocess.run(
                ["rosservice", "info", service_name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            if check.returncode == 0:
                return True
            time.sleep(0.2)
        print(f"Service {service_name} is not available after {timeout:.1f}s")
        return False

    def call_rosservice(self, service_name, request_args, retries=3):
        if not self.wait_for_service(service_name):
            return None

        command = ["rosservice", "call", service_name] + list(request_args)
        for attempt in range(1, retries + 1):
            try:
                return subprocess.check_output(command, stderr=subprocess.STDOUT)
            except subprocess.CalledProcessError as e:
                detail = e.output.decode('utf-8', errors='replace').strip()
                print(
                    f"Service call {service_name} failed "
                    f"(attempt {attempt}/{retries}, returncode={e.returncode}):\n{detail}"
                )
            except OSError as e:
                print(
                    f"Cannot execute rosservice for {service_name} "
                    f"(attempt {attempt}/{retries}): {e}"
                )

            if attempt < retries:
                time.sleep(0.5)
                if not self.wait_for_service(service_name, timeout=3.0):
                    break
        return None

    def start(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许端口复用
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        print(f"Server started on {self.host}:{self.port}")
        while True:
            client_socket, addr = self.server_socket.accept()
            print(f"Connection from {addr}")
            client_thread = threading.Thread(target=self.handle_client, args=(client_socket,))
            client_thread.start()

    def handle_client(self, client_socket):
        try:
            while True:
                data = client_socket.recv(1024).decode('utf-8')
                if not data:
                    break
                print(f"Received: {data}")
                request = json.loads(data)

                if request['cmd'] == 1:
                    self.handle_cmd_1(client_socket, request)
                elif request['cmd'] == 2:
                    self.handle_cmd_2(client_socket, request)
        except Exception as e:
            print(f"Error handling client: {e}")
        finally:
            client_socket.close()

    def handle_cmd_1(self, client_socket, request):
        id = request['id']
        cid = request['cid']

        # Start a thread to call rosservice and monitor execution
        def execute():
            start_time = time.time()
            result = False

            # Call rosservice asynchronously
            def call_service():
                nonlocal result
                service_request = (
                    f"{{data: {int(id)}, currentID: {int(cid)}, run: 1}}"
                )
                output = self.call_rosservice(
                    "/plan_path_and_go", [service_request]
                )
                if output is not None:
                    result = self.parse_rosservice_success(output)

            service_thread = threading.Thread(target=call_service)
            service_thread.start()

            # Send execution status every 0.5 seconds
            while service_thread.is_alive():
                elapsed_time = int(time.time() - start_time)
                response = {"cmd": 1, "id": id, "cid": cid, "execute": elapsed_time}
                client_socket.send(json.dumps(response).encode('utf-8'))
                time.sleep(0.5)

            # Send final result
            response = {"cmd": 1, "id": id, "cid": cid, "result": result}
            client_socket.send(json.dumps(response).encode('utf-8'))

        threading.Thread(target=execute).start()

    def handle_cmd_2(self, client_socket, request):
        target_x = float(request.get('x', 0.0))
        target_y = float(request.get('y', request.get('id', 0.0)))
        target_angle = float(request.get('angle', request.get('angel', 0.0)))
        cid = request.get('cid', 0)

        # Start a thread to call rosservice and monitor execution
        def execute():
            start_time = time.time()
            result = False

            # Call rosservice asynchronously
            def call_service():
                nonlocal result
                service_request = (
                    f"{{target_x: {target_x/100}, target_y: {target_y/100}, "
                    f"target_angle: {target_angle}}}"
                )
                output = self.call_rosservice(
                    "/set_target_y", [service_request]
                )
                if output is not None:
                    result = self.parse_rosservice_success(output)

            service_thread = threading.Thread(target=call_service)
            service_thread.start()

            # Send execution status every 0.5 seconds
            while service_thread.is_alive():
                elapsed_time = int(time.time() - start_time)
                response = {
                    "cmd": 2,
                    "x": target_x,
                    "y": target_y,
                    "angle": target_angle,
                    "cid": cid,
                    "execute": elapsed_time
                }
                client_socket.send(json.dumps(response).encode('utf-8'))
                time.sleep(0.5)

            # Send final result
            response = {
                "cmd": 2,
                "x": target_x,
                "y": target_y,
                "angle": target_angle,
                "cid": cid,
                "result": result
            }
            client_socket.send(json.dumps(response).encode('utf-8'))

        threading.Thread(target=execute).start()


if __name__ == "__main__":
    server = TCPServer()
    server.start()
