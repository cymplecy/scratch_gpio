import smbus
import time
bus = smbus.SMBus(1)

bus.write_byte_data(0x52,0x40,0x00)
time.sleep(0.1)
while True:
 try:
  bus.write_byte(0x52,0x00)
  time.sleep(0.2)
  data0 = bus.read_byte(0x52)
  data1 = bus.read_byte(0x52)
  data2 = bus.read_byte(0x52)
  data3 = bus.read_byte(0x52)
  data4 = bus.read_byte(0x52)
  data5 = bus.read_byte(0x52)
  joy_x = data0
  joy_y = data1
  accel_x = (data2 << 2) + ((data5 & 0x0c) >> 2)
  accel_y = (data3 << 2) + ((data5 & 0x30) >> 4)
  accel_z = (data4 << 2) + ((data5 & 0xc0) >> 6)
  buttons = data5 & 0x03
  button_c = (buttons == 1) or (buttons == 2)
  button_z = (buttons == 0) or (buttons == 2)
  print 'Jx: %s Jy: %s Ax: %s Ay: %s Az: %s Bc: %s Bz: %s' % (joy_x, joy_y, accel_x, accel_y, accel_z, button_c, button_z)
 except IOError as e:
  print e

