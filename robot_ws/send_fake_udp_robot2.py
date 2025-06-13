import socket
import json
import time

MULTICAST_GROUP = '224.0.0.1'
PORT = 5007

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)

while True:
    message = json.dumps({
        "name": "robot2",
        "distance": 10.0

    })
    sock.sendto(message.encode(), (MULTICAST_GROUP, PORT))
    print("📡 Distance simulée envoyée par robot2 : 10.0 m")
    time.sleep(1)

