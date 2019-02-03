#!/usr/bin/env python
# SPython - listen for GET requests from Scratch 3 translate extension
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

# Last Modified
# 30Jan19 18:00
# Start

import ssl
import socket
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
import SocketServer
import json
import urlparse
import urllib
import time
import struct



class S(BaseHTTPRequestHandler):

    def _set_headers(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin','*')
        self.end_headers()

    def do_GET(self):
        self._set_headers()
        print "GET: ", self.path
        if self.path == "/favicon.ico":
            return
        splitData = urllib.unquote(self.path).split('&text=[')
        print "split: " , splitData
        message = splitData[1]
        if message[-1] == "]":
            message = message[:-1]
        
        print "MESSAGE: ", message
                
        if message[0:7] == "letters":
            message = message[7:]
            sMessage = message.split('.to.')
            start = int(sMessage[0])
            sMessage = sMessage[1].split('.of.')
            end = int(sMessage[0])
            print "start end", start, end
            print "len:", len(sMessage[1])
            if end > len(sMessage[1]):
                end = len(sMessage[1])
            if start < 1:
                start = 1
            swapped = False
            if start > end:
                temp = end
                end = start
                start = temp
                swapped = True
            print "start end", start, end
            result = sMessage[1][int(start)-1:int(end)]
            if swapped:
                result = result[::-1]
            print "result", result
            response = {"result":result}
            self.wfile.write(json.dumps(response))
            
        elif message[0:11] == "position of":
            message = message[11:]
            sMessage = message.split('.in.')
            print str(sMessage)
            subString = sMessage[0]
            mainString = sMessage[1]
            result = mainString.find(subString)
            response = {"result":(int(result) + 1)}
            self.wfile.write(json.dumps(response))

        else:
            response = {"result":"no result found"}
            self.wfile.write(json.dumps(response))


    def do_POST(self):
        #print "POST RCVD"
        self._set_headers()
        parsed_path = urlparse.urlparse(self.path)
        request_id = parsed_path.path
        #response = subprocess.check_output(["python", request_id])
        #self.wfile.write(json.dumps(response))

    def do_HEAD(self):
        self._set_headers()
        
    def do_OPTIONS(self):
        self.sendResponse(200)
        self.processRequest()

def run(server_class=HTTPServer, handler_class=S, port=443):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    httpd.socket = ssl.wrap_socket (httpd.socket, certfile='./ca.pem', server_side=True)
    print 'Starting https webserver...'
    httpd.serve_forever()


#Start HTTPS webserver
run()
