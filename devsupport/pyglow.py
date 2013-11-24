import time
from smbus import SMBus

CMD_ENABLE_OUTPUT = 0x00
CMD_ENABLE_LEDS = 0x13
CMD_SET_PWM_VALUES = 0x01
CMD_UPDATE = 0x16

class PiGlow:
	i2c_addr = 0x54 # fixed i2c address of SN3218 ic
	bus = None

	def __init__(self, i2c_bus=1):
		print "init"
		self.bus = SMBus(i2c_bus)
		self.enable_output()
		self.enable_leds()

	def enable_output(self):
		self.write_i2c(CMD_ENABLE_OUTPUT, 0x01)

	def enable_leds(self):
		self.write_i2c(CMD_ENABLE_LEDS, [0xFF, 0xFF, 0xFF])

	def update_pwm_values(self, values):
		print "update pwm"
		self.write_i2c(CMD_SET_PWM_VALUES, values)
		self.write_i2c(CMD_UPDATE, 0xFF)
		
	def write_i2c(self, reg_addr, value):
		if not isinstance(value, list):
			value = [value];
		self.bus.write_i2c_block_data(self.i2c_addr, reg_addr, value)

# setup a fade across the 18 LEDs of values ranging from 0 - 255
values = [0x01,0x02,0x04,0x08,0x10,0x18,0x20,0x30,0x40,0x50,0x60,0x70,0x80,0x90,0xA0,0xC0,0xE0,0xFF]

# 1 is the I2C bus, should be 0 for old old old Pis
piglow = PiGlow(0)

# loop forever, i mean why would we ever want to stop now the party has started?
while True:
	# pop the first value off then drop it back on again - this just cycles the values around
	val = values.pop(0)
	values.append(val)
	
	# update the piglow with current values
	piglow.update_pwm_values(values)

	# sleep for a bit, don't go too fast!
	time.sleep(0.01)