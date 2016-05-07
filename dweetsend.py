import sys
import socket

dweetname = sys.argv[1] #Name of dweet channel to use
key = sys.argv[2]       #key name
value = sys.argv[3]     #key value


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)   #Alocate a socket
s.connect(("dweet.io", 80))                             #Connect to dweet.io

#Send a simple HTTP post command to Dweet
sent = s.send("POST /dweet/for/" + dweetname + "?" + key + "=" + value + " HTTP/1.1\r\nHost: dweet.io\r\nConnection: close\r\nAccept: */*\r\n\r\n".encode('utf-8'))
s.close()

#request = b"GET / HTTP/1.1\nHost: dweet.io/get/latest/dweet/for/cycy42\n\n"
request = b"GET / HTTP/1.1\nHost: dweet.io/get/dweets/for/cycy42\n\n"
#request = "http://dweet.io/get/dweets/for/cycy42\n\n"
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)   #Alocate a socket
s.connect(("dweet.io", 80))                             #Connect to dweet.io
#s.send(request)
#s.send("GET /get/latest/dweet/for/cycy42 HTTP/1.0\n\n")
s.send("GET /get/dweets/for/cycy42 HTTP/1.0\n\n")
result = s.recv(50000)
print(result)

