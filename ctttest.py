import sgh_captouch
import time

CT = sgh_captouch.Cap1208()

def handle(a,b):
    print "handle",a,b

for loop in range(0,8):
    CT.on(loop,"press",handle)

while True:
        time.sleep(1)