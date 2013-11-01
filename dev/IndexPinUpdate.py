#!/usr/bin/env python
global INVERT,PIN_USE,PIN_NUM,GPIO
def mod_index_pin_update(pin_index, value):
	global INVERT,PIN_USE,PIN_NUM,GPIO
	print pin_index, value
	if INVERT == True:
		if PIN_USE[pin_index] == 1:
			value = abs(value - 1)
	if (PIN_USE[pin_index] == 0):
		PIN_USE[pin_index] = 1
		GPIO.setup(PIN_NUM[pin_index],GPIO.OUT)
		print 'pin' , PIN_NUM[pin_index] , ' changed to digital out from input'
	if (PIN_USE[pin_index] == 2):
		PIN_USE[pin_index] = 1
		PWM_OUT[pin_index].stop()
		GPIO.setup(PIN_NUM[pin_index],GPIO.OUT)
		print 'pin' , PIN_NUM[pin_index] , ' changed to digital out from PWM'
	if (PIN_USE[pin_index] == 1):
		#print 'setting gpio %d (physical pin %d) to %d' % (GPIO_NUM[pin_index],PIN_NUM[pin_index],value)
		GPIO.output(PIN_NUM[pin_index], value)
	
