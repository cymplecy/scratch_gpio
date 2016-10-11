# findPort.py  12/05/2014  D.J.Whale
#
# Run this program to scan your system for serial ports.
# It will prompt you to remove and then insert your Arduino board.
# When it has finished, it stores the name of the found port in a file.
# This file is called portscan.cache
# 
# If you add any other usb devices to your system, you might need to re run
# this program again if the port number changes.

import arduino.portscan as portscan
portscan.main()

# END

