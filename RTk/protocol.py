# anyio/protocol.py  20/04/2014  D.J.Whale

# Eventually there will be sub modules to the protocol
# to include I2C, SPI, Uart, PWM, code load, and other stuff.
# For now, it only supports the GPIO module and it is loaded
# as the default context


def trace(msg):
  print(msg)

def error(msg):
  trace("error:" + str(msg))

IN  = 0
OUT = 1

GPIO_MODE_INPUT  = "I"
GPIO_MODE_OUTPUT = "O"

GPIO_READ       = "?"
GPIO_ANALOG       = "A"
GPIO_VALUE_HIGH = "1"
GPIO_VALUE_LOW  = "0"

PUD_DOWN = 21
PUD_UP = 22
PUD_OFF = 20
GPIO_PULL_DOWN  = "D"
GPIO_PULL_UP  = "U"
GPIO_PULL_NONE = "N"
BOARDMODE = 0

boardpins = {
3 : 2,
5 : 3,
7 : 4,
8 : 14,
10 : 15,
11 : 17,
12 : 18,
13 : 27,
15 : 22,
16 : 23,
18 : 24,
19 : 10,
21 : 9,
22 : 25,
23 : 11,
24 : 8,
26 : 7,
27 : 0,
28 : 1,
29 : 5,
31 : 6,
32 : 12,
33 : 13,
35 : 19,
36 : 16,
37 : 26,
38 : 20,
40 : 21}

def _pinch(channel):
    global BOARDMODE
    if(BOARDMODE==0):
        return chr(channel+ord('a'))
    else:
        channel = boardpins[channel]
        return chr(channel+ord('a'))

def _valuech(value):
  if value == None or value == 0 or value == False:
    return GPIO_VALUE_LOW
  return GPIO_VALUE_HIGH

def _modech(mode):
  if mode == None or mode == IN:
    return GPIO_MODE_INPUT
  return GPIO_MODE_OUTPUT

def _pudch(pud):
  if pud == PUD_DOWN:
    return GPIO_PULL_DOWN
  elif pud == PUD_UP:
    return GPIO_PULL_UP
  else:
    return GPIO_PULL_NONE

# Needed for Server later
#def _parse_pinch(ch):
#  pass #TODO inverse of _pinch
#
def _parse_valuech(ch):
  if ch == GPIO_VALUE_LOW:
    return False
  if ch == GPIO_VALUE_HIGH:
    return True
  error("Unknown value ch:" + ch)
  return GPIO_VALUE_HIGH

def _parse_avalue(ch):
  self.trace(ch)
  if ch == GPIO_VALUE_LOW:
    return False
  if ch == GPIO_VALUE_HIGH:
    return True
  error("Unknown value ch:" + str(ch))
  return GPIO_VALUE_HIGH

#def _parse_modech(ch):
#  pass #TODO inverse of _modech


# CLIENT ===============================================================
# Client will be constructed like: g = GPIOClient(Serial("/dev/ttyAMC0"))
# Client will be called via an interface just like RPi.GPIO

class GPIOClient:
  """ The GPIO command set
      Assumes the wire protocol is already in the GPIO mode.
      As we only support the GPIO module at the moment,
      that's a simple assumption to make.
  """
  IN = 0
  OUT = 1
  DEBUG = False
  PUD_DOWN = 21
  PUD_UP = 22
  BOARDMODE = 0


  def trace(self, msg):
    if self.DEBUG:
      trace(msg)

  def __init__(self, wire, debug=False):
    self.wire = wire
    self.DEBUG = debug

  def setmode(self, mode):
    global BOARDMODE

    if(mode == 0):
        BOARDMODE=0
    else:
        #Mode is Board Mode so we need to set the BOARDMODE to 1
        BOARDMODE=1

  def setwarnings(self, mode):
    #RTk.GPIO doesn't output warnings so this is just pass
    pass

  def analogueRead(self, channel):
  #RTk.GPIO doesn't output warnings so this is just pass

    pinch = _pinch(channel)
    self._write(pinch)
    self._write(GPIO_ANALOG)
    while True:
      v = self._read(10, termset="\r\n")

      if len(v) == 10:
        break
      self.trace("retrying")

    self.trace("input read back:" + v + " len:" + str(len(v)))
    if len(v) == 1:
      self.trace("single returned char is ord:" + str(ord(v[0])))

    v = v.split("i")[1]

    value = float(v)
    #rint(value)
    return (value)
    #return _parse_valuech("moo")

  def setup(self, channel, mode,pull_up_down=None):
    #TODO outer wrapper needs to do validation
    #if channel < self.MIN_PIN or channel > self.MAX_PIN:
    #  raise ValueError("Invalid pin")

    pinch = _pinch(channel)
    self.trace(pinch)

    modech = _modech(mode)
    #print(modech)
    self._write(pinch + modech)
    #TODO read and verify echoback
    #Pullupdown
    if(pull_up_down!=None):
      pudch = _pudch(pull_up_down)
      self._write(pinch + pudch)


  def input(self, channel):
    #self.trace("READ")
    #TODO outer wrapper needs to do validation
    #if channel < self.MIN_PIN or channel > self.MAX_PIN:
    #  raise ValueError("Invalid pin")
    pinch = _pinch(channel)
    self._write(pinch + GPIO_READ + "\n")
    while True:
      v = self._read(3, termset="\r\n")
      if len(v) == 3:
        break
      self.trace("retrying")

    self.trace("input read back:" + v + " len:" + str(len(v)))
    if len(v) == 1:
      self.trace("single returned char is ord:" + str(ord(v[0])))
    valuech = v[1]
    return _parse_valuech(valuech)



  def output(self, channel, value):
    #TODO outer wrapper needs to do validation
    #if channel < self.MIN_PIN or channel > self.MAX_PIN:
    #  raise ValueError("Invalid pin")
    ch = _pinch(channel)
    v = _valuech(value)
   # print(ch)
    #print(v)
    if value == None or value == 0 or value == False:
      self._write(ch)
      self._write(GPIO_VALUE_LOW)
    else:
      self._write(ch)
      self._write(GPIO_VALUE_HIGH)
    #TODO read and verify echoback

  def cleanup(self):
    pass


  # redirector to wrapped comms link
  def _open(self, *args, **kwargs):
    self.trace("open")
    self.wire.open(*args, **kwargs)

  def _write(self, *args, **kwargs):
    self.trace("write:" + str(*args) + " " + str(**kwargs))
    self.wire.write(*args, **kwargs)

  def _read(self, *args, **kwargs):
    self.trace("read")
    return self.wire.read(*args, **kwargs)

  def _close(self):
    self.trace("close")
    self.wire.close()


# SERVER ===============================================================
#
# The server is not needed yet, this is just a placeholder for later.
# We will use this to build a server listening on a network port,
# that accepts protocol commands and reroutes them to a local GPIO
# instance (allowing remote IO over the net or any other links)
#
# This is the server.
# Commands sent in are parsed and dispatched to methods
# server will be constructed like:
#   s = GPIOServer(Serial("/dev/ttyAMA0"), RPi.GPIO)
# or
#   s = GPIOServer(Net("localhost", 8888), sim.GPIO)
# (auto starts)
#   ...
#  s.stop()
# Do we open and close the port in here?
# How does Net wrap network.py, does it do the start() and stop()


#class GPIOServer:
#  """ A server that accepts wire protocol commands
#      and dispatches those to a real GPIO implementation.
#  """
#  def __init__(self, wire, gpio):
#    self.wire = wire
#    self.gpio = gpio
#
#  # The wrapping server class calls this function when it receives data
#
#  def receive(self, msg, reply):
#    msg = msg.trim()
#    #Might be multiple commands in a single message, all 2 chars long
#    #TODO change this into a resumable state machine,
#    #otherwise they must always come in together in the same call,
#    #which they might not do over serial
#    #Just push this into a _machine() handler and pump chars to it
#    #from here.
#
#    while len(msg) >= 2:
#      # Consume and parse 2 chars at a time
#      msgpart = msg[:2]
#      msg = msg[2:]
#
#  def _process(self, msg):
#    # This is always called with 2 chars
#    pinch = msg[0]
#    valuech = msg[1]
#
#    channel = _parse_pinch(pinch)
#    if   valuech == GPIO_MODE_INPUT:
#      self._setup(channel, IN)
#
#    elif valuech == GPIO_MODE_OUTPUT:
#      self._setup(channel, OUT)
#
#    elif valuech == GPIO_VALUE_HIGH:
#      self._output(channel, True)
#
#    elif valuech == GPIO_VALUE_LOW:
#      self._output(channel, False)
#
#    elif valuech == GPIO_READ:
#      pinval = self._read(channel)
#      reply(pinval)
#
#    #elif valuech == GPIO_CLEANUP:
#    #  _cleanup()
#
#    else:
#      error("Invalid valuech:" + valuech)
#      #TODO raise exception?
#
#
#  def _setmode(self, mode):
#    self.gpio.setmode(mode)
#
#  def _setup(self, channel, mode):
#    self.gpio.setup(channel, mode)
#
#  def _input(self, channel):
#    return self.gpio.input(channel)
#
#  def _output(self, channel, value):
#    self.gpio.output(channel, value)
#
#  # how does this get called? Need a cleanup wire command!
#  def _cleanup(self):
#    self.gpio.cleanup()
#
#  def _stop(self):
#    self.wire.stop()
#


#END
