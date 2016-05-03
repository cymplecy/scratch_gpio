import sys
import socket

dweetname = sys.argv[1] #Name of dweet channel to use
key = sys.argv[2]
value = sys.argv[3]


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("dweet.io", 80))
sent = s.send("POST /dweet/for/" + dweetname + "?" + key + "=" + value + " HTTP/1.1\r\nHost: dweet.io\r\nConnection: close\r\nAccept: */*\r\n\r\n".encode('utf-8'))
