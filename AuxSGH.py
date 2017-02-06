#!/usr/bin/env python
import scratch
import os as os
import paho.mqtt.client as mqtt
import time as time


def on_connect(client, userdata, rc):
    print("Connected with result code "+str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("cycy42/+")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print
    print time.asctime(), " Topic: ", msg.topic+'\nMessage: '+str(msg.payload), " received over MQTT"

    
def listen():
    while True:
        try:
           yield s.receive()
        except scratch.ScratchError:
           raise StopIteration

s = scratch.Scratch()
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

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
    # if msg[0] == 'broadcast':
        # if msg[1][:4] == "gpio":
            # os.system(msg[1])
        # elif msg[1][:4] == "mqtt":
            # print "mqtt"
            # if msg[1][:10] == "mqttbroker":
                # client.connect("192.168.0.23", 1883)      
                # print ("Connected to broker")
        
    # elif msg[0] == 'sensor-update':
    #     code to handle sensor updates


#s.disconnect()