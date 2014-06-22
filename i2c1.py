#!/usr/bin/python3
import os

i2c = os.popen("sudo i2cdetect -y 1").read().split()

for i in range(len(i2c)):
   if ((i2c[i][1:3] != "0:") & (i2c[i][1:2] != "")):
      if (i2c[i] != "--"):
         print (i2c[i])
