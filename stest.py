import array
import itertools
import socket
import struct
import sys
import time
        
dataOut = 'broadcast "All Off"'

s = socket.socket() #Create a socket object
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#host = socket.gethostname() #Get the local machine name
port = 42001 # Reserve a port for your service
s.bind(('127.0.0.1',port)) #Bind to the port

s.listen(5) #Wait for the client connection
print "listening"
c,addr = s.accept() #Establish a connection with the client
print "Got connection from", addr
n = len(dataOut)
b = (chr((n >> 24) & 0xFF)) + (chr((n >> 16) & 0xFF)) + (chr((n >> 8) & 0xFF)) + (
    chr(n & 0xFF))
c.send(b + dataOut)
print "sensor data to socket2", dataOut
time.sleep(2)
c.close()
#s.close() 