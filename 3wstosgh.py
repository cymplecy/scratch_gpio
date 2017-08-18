import array
import itertools
import socket
import struct
import sys

import threading
import time

def rcv_from_sgh():
    global s,c,server
    dataPrevious = ""
    while True:
        print "listening for data from sgh"
        # time.sleep(1)
        data = dataPrevious + c.recv(8192)
        if data != "":
            print "Data received from sgh", data
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
                        # if len(dataMsg) == size:  # if msg recieved is correct
                        #     if "alloff" in dataMsg:
                        #         allSplit = dataMsg.find("alloff")
                        #         logging.debug("not sure why this code is here Whole message:%s", dataIn)
                        #
                        # dataPrevious = ""  # no data needs tagging to next read
                        # if ("alloff" in dataMsg) or ("allon" in dataMsg):
                        #     dataList.append(dataMsg)
                        # else:
                        #     if dataMsg[0:2] == "br":  # removed redundant "broadcast" and "sensor-update" txt
                        #         if dataPrefix == "br":
                        #             dataList[-1] = dataList[-1] + " " + dataMsg[10:]
                        #         else:
                        #             dataList.append(dataMsg)
                        #             dataPrefix = "br"
                        #     else:
                        #         if dataPrefix == "se":
                        #             dataList[-1] += dataMsg[13:]  # changr from 10 to 13
                        #         else:
                        #             dataList.append(dataMsg)
                        #             dataPrefix = "se"

                        dataIn = dataIn[size + 4:]  # cut data down that's been processed

                        # print "previous:", dataPrevious
            print "datalist:",dataList
            for msg in dataList:
                msgsplit = msg.split('"')
                server.send_message_to_all(msgsplit[1] + ':' + msgsplit[2][1:])
        else:
            time.sleep(0.1)
            
            #connB, addrB = self.scratch_socketB.accept()
            # reply = 'OK...' + data
            # if not data:



# Called for every client connecting (after handshake)
def new_client(client, server):
    print("New client connected and was given id %d" % client['id'])
    server.send_message_to_all("Hey all, a new client has joined us")


# Called for every client disconnecting
def client_left(client, server):
    print("Client(%d) disconnected" % client['id'])


# Called when a client sends a message
def message_received(client, server, message):
    if len(message) > 200:
        message = message[:200]+'..'
    print("Client(%d) said: %s" % (client['id'], message))
    dataOut = message

    n = len(dataOut)
    b = (chr((n >> 24) & 0xFF)) + (chr((n >> 16) & 0xFF)) + (chr((n >> 8) & 0xFF)) + (
        chr(n & 0xFF))
    c.send(b + dataOut)
    print "Data sent to sgh", dataOut


       

# For Scratch 3 handle long as int 
if sys.version > '3':
    long = int

class ScratchError(Exception): pass
class ScratchConnectionError(ScratchError): pass        

class Scratch(object):

    prefix_len = 4
    broadcast_prefix_len = prefix_len + len('broadcast ')
    sensorupdate_prefix_len = prefix_len + len('sensor-update ')

    msg_types = set(['broadcast', 'sensor-update'])

    def __init__(self, host='localhost', port=42001):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        self.connect()
        

        
    def __repr__(self):
        return "Scratch(host=%r, port=%r)" % (self.host, self.port)

    def _pack(self, msg):
        """
        Packages msg according to Scratch message specification (encodes and 
        appends length prefix to msg). Credit to chalkmarrow from the 
        scratch.mit.edu forums for the prefix encoding code.
        """
        n = len(msg)
        packstr = chr((n >> 24) & 0xFF)
        packstr += chr ((n >> 16) & 0xFF)
        packstr += chr  ((n >>  8) & 0xFF)
        packstr += chr (n & 0xFF)
        return packstr + msg

    def _extract_len(self, prefix):
        """
        Extracts the length of a Scratch message from the given message prefix. 
        """
        return struct.unpack(">L", prefix)[0]

    def _get_type(self, s):
        """
        Converts a string from Scratch to its proper type in Python. Expects a
        string with its delimiting quotes in place. Returns either a string, 
        int or float. 
        """
        # TODO: what if the number is bigger than an int or float?
        
        # convert to string (rather than bytes)
        s = s.decode(encoding='UTF-8')
        if s.startswith("\"") and s.endswith("\""):
            return s[1:-1]
        elif s.find('.') != -1: 
            return float(s) 
        else: 
            return int(s)

    def _escape(self, msg):
        """
        Escapes double quotes by adding another double quote as per the Scratch
        protocol. Expects a string without its delimiting quotes. Returns a new
        escaped string. 
        """
        escaped = ''        
        for c in msg: 
            escaped += c
            if c == '"':
                escaped += '"'
        return escaped

    def _unescape(self, msg):
        """
        Removes double quotes that were used to escape double quotes. Expects
        a string without its delimiting quotes, or a number. Returns a new
        unescaped string.
        """
        if isinstance(msg, (int, float, long)):
            return msg

        unescaped = ''
        i = 0
        while i < len(msg):
            unescaped += msg[i]
            if msg[i] == '"':
                i+=1
            i+=1
        return unescaped     

    def _is_msg(self, msg):
        """
        Returns True if message is a proper Scratch message, else return False.
        """
        if not msg or len(msg) < self.prefix_len:
            return False
        length = self._extract_len(msg[:self.prefix_len])
        msg_type = msg[self.prefix_len:].decode(encoding='UTF-8').split(' ', 1)[0]
        if length == len(msg[self.prefix_len:]) and msg_type in self.msg_types:
            return True
        return False

    def _parse_broadcast(self, msg):
        """
        Given a broacast message, returns the message that was broadcast.
        """
        # get message, remove surrounding quotes, and unescape
        return self._unescape(self._get_type(msg[self.broadcast_prefix_len:]))

    def _parse_sensorupdate(self, msg):
        """
        Given a sensor-update message, returns the sensors/variables that were
        updated as a dict that maps sensors/variables to their updated values.
        """
        update = msg[self.sensorupdate_prefix_len:].decode(encoding='UTF-8')
        parsed = [] # each element is either a sensor (key) or a sensor value
        curr_seg = '' # current segment (i.e. key or value) being formed
        numq = 0 # number of double quotes in current segment
        for seg in update.split(' ')[:-1]: # last char in update is a space
            numq += seg.count('"')
            curr_seg += seg
            # even number of quotes means we've finished parsing a segment
            if numq % 2 == 0: 
                parsed.append(curr_seg)
                curr_seg = ''
                numq = 0
            else: # segment has a space inside, so add back it in
                curr_seg += ' '
        unescaped = [self._unescape(self._get_type(x)) for x in parsed]
        # combine into a dict using iterators (both elements in the list
        # inputted to izip have a reference to the same iterator). even 
        # elements are keys, odd are values
        return dict(itertools.izip(*[iter(unescaped)]*2)) 

    def _parse(self, msg):
        """
        Parses a Scratch message and returns a tuple with the first element
        as the message type, and the second element as the message payload. The 
        payload for a 'broadcast' message is a string, and the payload for a 
        'sensor-update' message is a dict whose keys are variables, and values
        are updated variable values. Returns None if msg is not a message.
        """
        if not self._is_msg(msg):
            return None
        msg_type = msg[self.prefix_len:].decode(encoding='UTF-8').split(' ')[0]
        if msg_type == 'broadcast':
            return ('broadcast', self._parse_broadcast(msg))
        else:
            return ('sensor-update', self._parse_sensorupdate(msg))

    def _write(self, data):
        """
        Writes string data out to Scratch
        """
        total_sent = 0
        length = len(data)
        while total_sent < length:
            try:
                sent = self.socket.send(data[total_sent:].encode('UTF-8'))
            except socket.error as e:
                self.connected = False
                raise ScratchError(e)
            if sent == 0:
                self.connected = False
                raise ScratchConnectionError("Connection broken")
            total_sent += sent

    def _send(self, data):
        """
        Sends a message to Scratch
        """
        self._write(self._pack(data))

    def _read(self, size):
        """
        Reads size number of bytes from Scratch and returns data as a string
        """
        data = b''
        while len(data) < size:
            try:
                chunk = self.socket.recv(size-len(data))
            except socket.error as e:
                self.connected = False
                raise ScratchError(e)
            if chunk == '':
                self.connected = False
                raise ScratchConnectionError("Connection broken")
            data += chunk
        return data

    def _recv(self):
        """
        Receives and returns a message from Scratch
        """
        prefix = self._read(self.prefix_len)
        msg = self._read(self._extract_len(prefix))
        return prefix + msg

    def connect(self):
        """
        Connects to Scratch.
        """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.socket.connect((self.host, self.port))
        except socket.error as e:
            self.connected = False
            raise ScratchError(e)
        self.connected = True
                 
    def disconnect(self):
        """
        Closes connection to Scratch
        """
        try: # connection may already be disconnected, so catch exceptions
            self.socket.shutdown(socket.SHUT_RDWR) # a proper disconnect
        except socket.error:
            pass
        self.socket.close()
        self.connected = False

    def sensorupdate(self, data):
        """
        Given a dict of sensors and values, updates those sensors with the 
        values in Scratch.
        """
        if not isinstance(data, dict):
            raise TypeError('Expected a dict')
        msg = 'sensor-update '
        for key in data.keys():
            msg += '"%s" "%s" ' % (self._escape(str(key)), 
                                   self._escape(str(data[key])))
        self._send(msg)

    def broadcast(self, msg):
        """
        Broadcasts msg to Scratch. msg can be a single message or an iterable 
        (list, tuple, set, generator, etc.) of messages.
        """
        #if getattr(msg, '__iter__', False): # iterable
        if isinstance(msg, (tuple, list, set)):
            for m in msg:
                self._send('broadcast "%s"' % self._escape(str(m)))
        else: # probably a string or number
            self._send('broadcast "%s"' % self._escape(str(msg)))

    def receive(self):
        """
        Receives broadcasts and sensor updates from Scratch. Returns a tuple
        with the first element as the message type and the second element 
        as the message payload. broadcast messages have a string as payload, 
        and the sensor-update messages have a dict as payload. Returns None if 
        message received could not be parsed. Raises exceptions on connection
        errors.
        """
        return self._parse(self._recv())

       
        
s = socket.socket() #Create a socket object
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#host = socket.gethostname() #Get the local machine name
port = 42001 # Reserve a port for your service
s.bind(('127.0.0.1',port)) #Bind to the port

s.listen(5) #Wait for the client connection
print "wstosgh listening to scratchGPIO_handler"
c,addr = s.accept() #Establish a connection with the client
print "Got connection from ScratchGPIOHandler", addr







       

from websocket_server import WebsocketServer

scr= Scratch()

PORT=8000
server = WebsocketServer(PORT)
server.set_fn_new_client(new_client)
server.set_fn_client_left(client_left)
server.set_fn_message_received(message_received)
d = threading.Thread(name='rcv_from_sgh', target=rcv_from_sgh)
d.setDaemon(True)
d.start()
server.run_forever()