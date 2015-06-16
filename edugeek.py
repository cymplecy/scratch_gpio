import pibrella
import time

while True:

    if pibrella.button.read() == 1:
        #get time when button pressed
        starttime = time.clock()
        # wait until button released
        while pibrella.button.read() == 1:
            endtime = time.clock()
        # check if button held down for less than 0.5 sec
        if endtime-starttime < 0.5:
            pibrella.light.red.on()
            time.sleep(2)
            pibrella.light.red.off()
        else:
            pibrella.light.yellow.on()
            time.sleep(2)
            pibrella.light.yellow.off()