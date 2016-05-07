import sys
import socket
import time
from array import array

def sendScratchCommand(cmd):
   n = len(cmd)
   a = array('c')
   a.append(chr((n >> 24) & 0xFF))
   a.append(chr((n >> 16) & 0xFF))
   a.append(chr((n >>  8) & 0xFF))
   a.append(chr(n & 0xFF))
   try:
      PORT = 42001
      HOST = '127.0.0.1'

      scratchSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      scratchSock.connect((HOST, PORT))
      #print "data len" , n
      scratchSock.sendall(a.tostring() + cmd)
      time.sleep(0.1)
      scratchSock.shutdown(socket.SHUT_RDWR)
      scratchSock.close()
   except:
      print sys.exc_info()[0]
      print ("Failed to send to Scratch")
      
dweetname = sys.argv[1] #Name of dweet channel to use
try:
    key = sys.argv[2]       #key name
    value = sys.argv[3]     #key value
except:
    pass


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)   #Alocate a socket
s.connect(("dweet.io", 80))                             #Connect to dweet.io

s.send("GET /get/latest/dweet/for/cycy42 HTTP/1.0\n\n")
#s.send("GET /get/dweets/for/cycy42 HTTP/1.0\n\n")
result = s.recv(50000)

#print(result)

res2 = result.split('"content"')
#print
#print res2[1]

res3 = res2[1].split('"')
#print
print res3[1],res3[3]

sendScratchCommand('sensor-update ' + res3[1] + ' ' + res3[3])
