#!/usr/bin/env python
# S3GPIOserver - listen for GET requests from Scratch 3 translate extension
# Copyright (C) 2013-2019 by Simon Walters with help from David Ferguson

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

# This code hosted on Github thanks to Ben Nuttall who taught me how to be a git(ter)


# Tidied up a lot
Version = "1.0.1_11Mar2019" #Added in direct read of sensordict

#import BaseHTTPServer, SimpleHTTPServer
import ssl
import socket
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
import SocketServer
import json
import urlparse
import subprocess
import time
import array
import struct
import threading
import urllib

sensorDict = {"s3gpioversion":Version}

def broadcast_to_sgh(dataOut):
    #dataOut = 'broadcast "'  + command + '"'
    n = len(dataOut)
    b = (chr((n >> 24) & 0xFF)) + (chr((n >> 16) & 0xFF)) + (chr((n >> 8) & 0xFF)) + (chr(n & 0xFF))
    sghConnection.send(b + dataOut)
    print "Data sent to sgh", dataOut
    
def isJSON(data):
    try:
        json.loads(data)
        return True
    except ValueError as error:
        print("invalid json: %s" % error)
        return False


def rcv_from_sgh():
    global sghSocket,sghConnection,server,sensorDict
    dataPrevious = ""
    while True:
        #print "listening for data from sgh"
        # time.sleep(1)
        data = dataPrevious + sghConnection.recv(8192)
        if data != "":
            #print "Data received from sgh", data
            #print ("datalen: %s", len(data))
            if len(data) > 0:  # Connection still valid so process the data received
                dataIn = data
                datawithCAPS = data
                # dataOut = ""
                dataList = []  # used to hold series of broadcasts or sensor updates
                dataPrefix = ""  # data to be re-added onto front of incoming data
                while len(dataIn) > 0:  # loop thru data
                    if len(dataIn) < 4:  # If whole length not received then break out of loop
                        # print "<4 chrs received"
                        dataPrevious = dataIn  # store data and tag it onto next data read
                        break
                    sizeInfo = dataIn[0:4]
                    size = struct.unpack(">L", sizeInfo)[0]  # get size of Scratch msg
                    # print "size:", size
                    if size > 0:
                        # print dataIn[4:size + 4]
                        dataMsg = dataIn[4:size + 4].lower()  # turn msg into lower case
                        if len(dataMsg) < size:  # if msg recieved is too small
                            # print "half msg found"
                            # print size, len(dataMsg)
                            dataPrevious = dataIn  # store data and tag it onto next data read
                            break
                        #print "msg:",dataMsg
                        dataList.append(dataMsg)
                        dataIn = dataIn[size + 4:]  # cut data down that's been processed
                        # print "previous:", dataPrevious
            #print "datalist:",dataList
            for msg in dataList:
                #print "msg:",msg
                if msg[0:13] == 'sensor-update':
                    msgsplit = msg[14:].replace('"','').split(' ')
                    #print "split",msgsplit
                    print "SENSORDICT:" + str(sensorDict)
                    for loop in range(int(len(msgsplit) / 2)):
                        sensorDict[msgsplit[loop * 2]] = msgsplit[(loop * 2) + 1]
        else:
            time.sleep(0.1)

class S(BaseHTTPRequestHandler):

    def _set_headers(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin','*')
        self.end_headers()

    def do_GET(self):
        self._set_headers()
        parsed_path = urlparse.urlparse(self.path)

        print "GET: ", self.path
        if self.path == "/favicon.ico":
            return
        splitData = urllib.unquote(self.path).split('&text=',1)
        print "split: " , splitData
        message = ""
        if len(splitData) > 1:
            message = splitData[1]
        else:
            response = {"result":"invalid request made"}
            self.wfile.write(json.dumps(response))
            return

        print "isJSON", isJSON(message)
        messageKey = None
        messageValue = None
        if isJSON(message):
            jsonMessage = json.loads(message)
            print "jdict", jsonMessage
            messageKey = jsonMessage.keys()[0].lower().replace(" ","")
            messageValue= jsonMessage[messageKey]
            print "messageKey", messageKey
            print "messageValue", messageValue
        else:
            print "doing own parsing"
            if len(message) > 0:
                if (message[0] == "[") and (message[-1] == "]"):
                    message = message[1:-1]
                elif (message[0] == "[") and (message[-1] <> "]"):
                    message = message[1:]
                split1 = message.split(":",1)
                print "split1: ", split1
                if len(split1) == 2:
                    messageKey = split1[0].lower().replace(" ","")
                    messageValue = split1[1]
                    print "messageKey", messageKey
                    print "messageValue", messageValue
                elif len(split1) == 1:
                    print("no : in message")
                    messageKey = "s3gpioread"
                    messageValue = message.lower().replace(" ","")

        if messageKey != None:
            sghMessage = None
            if messageKey == "s3gpiocommand":
                sghMessage = 'broadcast "'  + messageValue + '"'
                broadcast_to_sgh(sghMessage)
                response = {"result":"Command processed"}
                self.wfile.write(json.dumps(response))
                return
            elif messageKey == "s3gpioset":
                sMessage = messageValue.split(':',1)
                if len(sMessage) > 1:
                    sghMessage = 'sensor-update "' + sMessage[0] +'" "' + sMessage[1] + '"'
                else:
                    sghMessage = 'sensor-update "' + message + '"'
                broadcast_to_sgh(sghMessage)
                response = {"result":"Command processed"}
                self.wfile.write(json.dumps(response))
                return
            elif messageKey == "s3gpioread":
                print "SENSORDICT:" + str(sensorDict)
                messageValue = messageValue.lower().replace(" ","")
                if messageValue in sensorDict:
                    response = '{"result":"' + sensorDict[messageValue] +'"}'
                    print "Response to Scratch:" + response
                    self.wfile.write(response)
                    return
                elif messageValue == "all":
                    response = '{"result":' + str(sensorDict) +'}'
                    print "Response to Scratch:" + response
                    self.wfile.write(response)
                    return
                else:
                    response = '{"result":"' + 'read value not recognised' +'"}'
                    print "Response to Scratch:" + response
                    self.wfile.write(response)
                    return
            #Add in extra processing code here to handle other mesageKeys
            #elif messageKey == "xxxxxxx"
            #    response = '{"result":"' + 'your return value' +'"}'
            #    print "Response to Scratch:" + response
            #    self.wfile.write(response)
            #    return
            else:
                response = '{"result":"' + 'command not recognised' +'"}'
                print "Response to Scratch:" + response
                self.wfile.write(response)
                return
        else:
            response = '{"result":"' + 'no value received' +'"}'
            print "Response to Scratch:" + response
            self.wfile.write(response)
            return



    def do_POST(self):
        #print "POST RCVD"
        self._set_headers()
        parsed_path = urlparse.urlparse(self.path)
        request_id = parsed_path.path


    def do_HEAD(self):
        self._set_headers()
        
    def do_OPTIONS(self):
        self.sendResponse(200)
        self.processRequest()

def run(server_class=HTTPServer, handler_class=S, port=443):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    httpd.socket = ssl.wrap_socket (httpd.socket, certfile='./ca.pem', server_side=True)
    print 'Starting webserver...'
    httpd.serve_forever()

#Setup communication to scratchgpio_handler
sghSocket = socket.socket() #Create a socket object
sghSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#host = socket.gethostname() #Get the local machine name
port = 42001 # Scratch 1.4 standard port
sghSocket.bind(('127.0.0.1',port)) #Bind to the port
sghSocket.listen(5) #Wait for the client connection
print "trying to listen to scratchGPIO_handler"
sghConnection,addr = sghSocket.accept() #Establish a connection with the client
print "Got connection from ScratchGPIOHandler", addr

#pre-seed some sensor values
broadcast_to_sgh('broadcast "get ip"')
broadcast_to_sgh('broadcast "version"')


#Start thread to listen to data coming from scratchgpio_handler
listen_to_sgh = threading.Thread(name='rcv_from_sgh', target=rcv_from_sgh)
listen_to_sgh.setDaemon(True)
listen_to_sgh.start()

#Start HTTPS webserver
run()
