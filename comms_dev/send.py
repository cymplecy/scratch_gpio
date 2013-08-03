#Original Code Martin Bateman 2013
#Modified by Simon Walters
#GPLv2 applies

import sys, time
from socket import *

s = socket(AF_INET, SOCK_DGRAM)
s.bind(('', 0))
s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

if len(sys.argv) > 1:
    message = sys.argv[1]
else:
    message = "Hi, I'm your Pi"

for i in range(0,30):
    s.sendto(message, ('<broadcast>', 50000))
    time.sleep(2)