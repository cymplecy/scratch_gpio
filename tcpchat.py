#!/usr/bin/env python

import socket


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('127.0.0.1', 2002))
s.listen(1)

while True:
    c,addr = s.accept()
    rcvdata = c.recv(512)
    print rcvdata
    splitdata = rcvdata.split ("\n")
    print splitdata[1]
    value = ord(splitdata[4][38])
    #print value
    print splitdata[4][42: 42 + value]
    
    c.close()