import socket

# Define server host and port
HOST = '192.168.100.248'  # Listen on all interfaces
PORT = 12345      # Port to listen on (non-privileged ports are > 1023)

# Create socket
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f"Server listening on {HOST}:{PORT}")

    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        while True:
            data = conn.recv(1024)
            if not data:
                break
            print(f"Received {data.decode()}")

# To run the server, use:
# python3 server.py
