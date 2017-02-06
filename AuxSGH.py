#!/usr/bin/env python
import scratch
import os as os
import paho.mqtt.client as mqtt
import time as time
from sgh_GetJSONFromURL import GetJSONFromURL

getjsonfromurl = GetJSONFromURL()


    
def listen():
    while True:
        try:
           yield s.receive()
        except scratch.ScratchError:
           raise StopIteration

s = scratch.Scratch()


result = False
while result is False:
    try:
        # connect
        s.connect()
        print ("connected")
        result = True
    except:
        print "waiting"
        pass
    time.sleep(1)

for msg in listen():
    print msg
    if msg[0] == 'broadcast':
        if msg[1][:10] == "getweather":
            data = getjsonfromurl.getJSON("http://api.openweathermap.org/data/2.5/weather?q=Chorley,uk&appid=4041655e60abaea9a9134b6e78ca864f")
            print data
            s.sensorupdate({"main_temp" : data.get("main_temp")})
        # elif msg[1][:4] == "mqtt":
            # print "mqtt"
            # if msg[1][:10] == "mqttbroker":
                # client.connect("192.168.0.23", 1883)      
                # print ("Connected to broker")
        
    # elif msg[0] == 'sensor-update':
    #     code to handle sensor updates


#s.disconnect()