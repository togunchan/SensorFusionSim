import socket

HOST = "127.0.0.1"
PORT = 5555

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((HOST, PORT))
sock.listen(1)

print(f"[PythonServer] Listening on {HOST}:{PORT}")

conn, addr = sock.accept()
print(f"[PythonServer] Connected from {addr}")

with conn:
    while True:
        data = conn.recv(4096)
        if not data:
            break
        print("[PythonServer] Received:", data.decode().strip())