#!/usr/bin/env python
import RPi.GPIO as GPIO
import time

devices = [
    {
        "name": "mainPwr",
        "state": 0,
        "pin": 4
    },
    {
        "name": "circPump",
        "state": 0,
        "pin": 10
    },
    {
        "name": "heaterGas",
        "state": 0,
        "pin": 12
    },
    {
        "name": "heaterIgnite",
        "state": 0,
        "pin": 13
    },
    {
        "name": "speakerIn",
        "state": 0,
        "pin": 16
    },
    {
        "name": "speakerOut",
        "state": 0,
        "pin": 17
    },
    {
        "name": "stereoPower",
        "state": 0,
        "pin": 18
    },
    {
        "name": "lightDeck",
        "state": 0,
        "pin": 19
    },
    {
        "name": "lightsTub",
        "state": 0,
        "pin": 20
    },
    {
        "name": "lightsCover",
        "state": 0,
        "pin": 21
    },
    {
        "name": "aux",
        "state": 0,
        "pin": 23
    },
    {
        "name": "aux",
        "state": 0,
        "pin": 24
    },
    {
        "name": "aux1",
        "state": 0,
        "pin": 24
    },
    {
        "name": "aux2",
        "state": 0,
        "pin": 25
    },
    {
        "name": "volUp",
        "state": 0,
        "pin": 26
    },
    {
        "name": "volDown",
        "state": 0,
        "pin": 27
    },
]

modbtn = {
    "name": "modifier",
    "state": 0,
    "pin": 23
}


GPIO.setmode(GPIO.BCM)
GPIO.setup(modbtn['pin'], GPIO.IN)
for device in devices:
    GPIO.setup(device['pin'], GPIO.IN)

def checkDevice(device):
    prev_state = device['state']
    input = GPIO.input(device['pin'])
    if prev_state != input:
        mosquitto_pub - t sensors / temperature - m 32 - q 1

for device in devices:
    checkDevice(device)