import socket

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
    server_socket.connect(('127.0.0.1', 6666))
    server_socket.sendall(b'Hello World!')
