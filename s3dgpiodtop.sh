#!/bin/bash
#Version 0.2 - add in & to allow simulatenous running of handler and Scratch
#Version 0.3 - change sp launches rsc.sb from "/home/pi/Documents/Scratch Projects"
#Version 0.4 - 20Mar13 meltwater - change to use provided name for home
#Version 1.0 - 29Oct13 sw - change to cd into simplesi_scratch_handler to run servods OK
#V 20Aug19 - modify for Scratch 3 Desktop
sudo pkill -f scratchgpio_handler
sudo pkill -f S3GPIOServer
cd /opt/scratchgpio8
sudo python scratchgpio_handler8.py &
sudo python S3GPIOServer.py &
cd ~/Scratch
scratch3
sudo pkill -f scratchgpio_handler
sudo pkill -f S3GPIOServer

