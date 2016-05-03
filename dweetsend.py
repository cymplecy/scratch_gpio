import sys
import socket

dweetname = sys.argv[1] #Name of dweet channel to use
key = sys.argv[2]       #key name
value = sys.argv[3]     #key value


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)   #Alocate a socket
s.connect(("dweet.io", 80))                             #Connect to dweet.io

#Send a simple HTTP post command to Dweet
sent = s.send("POST /dweet/for/" + dweetname + "?" + key + "=" + value + " HTTP/1.1\r\nHost: dweet.io\r\nConnection: close\r\nAccept: */*\r\n\r\n".encode('utf-8'))

