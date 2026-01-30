import math
import socket
from pymavlink import mavutil

mav = mavutil.mavlink_connection('udp:127.0.0.1:14550')
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

while True:
    msg = mav.recv_match(type='ATTITUDE', blocking=True)
    if msg:
        roll = math.degrees(msg.roll)
        pitch = math.degrees(msg.pitch)
        yaw = math.degrees(msg.yaw)
        data = f"{roll:.2f},{pitch:.2f},{yaw:.2f}".encode()
        sock.sendto(data, ('<broadcast>', 5000))