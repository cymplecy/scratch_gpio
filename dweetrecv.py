import sys
import socket

dweetname = sys.argv[1] #Name of dweet channel to use
try:
    key = sys.argv[2]       #key name
    value = sys.argv[3]     #key value
except:
    pass


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)   #Alocate a socket
s.connect(("dweet.io", 80))                             #Connect to dweet.io

#s.send("GET /get/latest/dweet/for/cycy42 HTTP/1.0\n\n")
s.send("GET /get/dweets/for/cycy42 HTTP/1.0\n\n")
result = s.recv(50000)
print(result)

