#!/usr/bin/env python
import socket, traceback
host = '192.168.0.86'
port = 5555
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind((host, port))
while 1:
    try:
        message, address = s.recvfrom(8192)
        print message
    except (KeyboardInterrupt, SystemExit):
        raise