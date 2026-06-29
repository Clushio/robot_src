import socket
import threading
import time
import json
import subprocess


class TCPServer:
    def __init__(self, host='192.168.3.216', port=12345):
        # 192.168.1.216 172.20.103.212
        self.host = host
        self.port = port
        self.server_socket = None
        self.clients = []

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
                try:
                    request_msg = f"{{data: {id}, currentID: {cid}, run: 1}}"
                    output = subprocess.check_output(
                        ["rosservice", "call", "/plan_path_and_go", request_msg],
                        stderr=subprocess.STDOUT
                    )
                    result = "success" in output.decode('utf-8').lower()
                except Exception as e:
                    print(f"Service call failed: {e}")

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
                try:
                    service_request = (
                        f"{{target_x: {target_x/100}, target_y: {target_y/100}, "
                        f"target_angle: {target_angle}}}"
                    )
                    output = subprocess.check_output(
                        ["rosservice", "call", "/set_target_y", service_request],
                        stderr=subprocess.STDOUT
                    )
                    result = "success" in output.decode('utf-8').lower()
                except Exception as e:
                    print(f"Service call failed: {e}")

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