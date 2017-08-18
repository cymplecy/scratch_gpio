#!/usr/bin/env python
import time
import sys

import SimpleMFRC522
reader = SimpleMFRC522.SimpleMFRC522()

tick = 0
lastTimeSinceLastSleep = time.time()
sleepTime = 0.1

while True:
    loopTime = time.time() - lastTimeSinceLastSleep
    if loopTime < sleepTime:
        sleepdelay = sleepTime - loopTime
        time.sleep(sleepdelay)  # be kind to cpu  :)
    tick += 1
    print "tick count",time.time(),tick
    if tick == sys.maxint:
        tick = 0
    lastTimeSinceLastSleep = time.time()

    if reader is not None:
        if tick % 10 == 0:   
            cardid,cardtext = reader.read_no_block()
            print "cardid",cardid
               
