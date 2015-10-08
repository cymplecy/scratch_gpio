#!/usr/bin/env python
from bottle import route, run, template, request
import socket

@route('/')
def index():
    return '''
        <form action="/send" method="post">
            Command: <input name="scratchgpio" type="text" />
            <input value="Send" type="submit" />
        </form>
    '''

@route('/send', method='POST')
def get_sendtext():
    sendText = request.forms.get('scratchgpio')
    sendToScratch(sendText)
    return "<p>Your command<p>" + sendText + "<p>was sent</p>"    
    
def sendToScratch(stext):
    scratch_socket2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    scratch_socket2.connect(('127.0.0.1', 42001))

    totalcmd = ''
    cmd = 'broadcast "' + stext + '"'
    n = len(cmd)
    b = (chr((n >> 24) & 0xFF)) + (chr((n >> 16) & 0xFF)) + (chr((n >> 8) & 0xFF)) + (
        chr(n & 0xFF))
    totalcmd = b + cmd

    scratch_socket2.send(totalcmd)    
    scratch_socket2.close()

run(host='0.0.0.0', port=80)
