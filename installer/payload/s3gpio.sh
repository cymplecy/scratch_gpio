#!/bin/bash
#Version 0.2 - add in & to allow simulatenous running of handler and Scratch
#Version 0.3 - change sp launches rsc.sb from "/home/pi/Documents/Scratch Projects"
#Version 0.4 - 20Mar13 meltwater - change to use provided name for home
#Version 1.0 - 29Oct13 sw - change to cd into simplesi_scratch_handler to run servods OK
sudo pkill -f scratchgpio_handler
sudo pkill -f S3GPIOServer
cd /opt/scratchgpio8
sudo python scratchgpio_handler8.py &
sudo python S3GPIOServer.py &
chromium-browser --start-maximized --load-extension=S3GPIOExtension https://scratch.mit.edu/projects/285979180
sudo pkill -f scratchgpio_handler
sudo pkill -f S3GPIOServer

