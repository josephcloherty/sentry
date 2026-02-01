import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 5000))

while True:
    data, _ = sock.recvfrom(1024)
    roll, pitch, yaw = data.decode().split(',')
    print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")