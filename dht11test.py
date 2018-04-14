import pigpio
import dht11
pi = pigpio.pi()
sensor = dht11.DHT11(pi, 15) # 4 is the data GPIO pin connected to your sensor
print sensor.read()
print sensor.temperature
#print sensor.temperature()
