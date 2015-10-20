import socket
s = socket.socket(
    socket.AF_INET, socket.SOCK_STREAM)
#now connect to the web server on port 80
# - the normal http port
s.connect(("dweet.io", 80))
sent = s.send("POST /dweet/for/cycy42?test=Hello_World HTTP/1.1\r\nHost: dweet.io\r\nConnection: close\r\nAccept: */*\r\n\r\n".encode('utf-8'))