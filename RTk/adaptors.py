# anyio/adaptors.py  22/04/2014  D.J.Whale
#
# Some adaptors to make the inner connectivity classes all work
# in a standard way

  
# adapt a pyserial (or anything else) to our common interface
class SerialAdaptor:
  
  def __init__(self, serial):
    self.serial = serial
    
  def open(self, *args):
    self.serial.open(*args)
    
  def close(self, *args):
    self.serial.close(*args)
    
  # read data between minsize and maxsize chars.
  # optional terminator set to mark end of packet.
  # timeout if any one read takes longer than timeout period.
  # i.e. whole packet may take longer to come in
  
  def read(self, maxsize=1, minsize=None, termset=None, timeout=None):
    if minsize == None:
      minsize = maxsize
      
    remaining = maxsize
    if termset != None:
      readsz = 1
    else:
      readsz = remaining

    buf = b''
      
    while len(buf) < minsize:
      data = self.serial.read(readsz)
      if (len(data) == 0):
        time.sleep(0.1) # prevent CPU hogging
      else:
        #print("just read:" + data)
        buf = buf + data
        remaining -= len(data)
        if termset != None:
          if data[0] in termset:
            break # terminator seen
        
    return buf
    
  def write(self, str):
    self.serial.write(str.encode('ascii'))


#TODO adapt a network.py connection to our normal connection scheme
class NetAdaptor:
  def __init__(self, net):
    pass # TODO
    #self.net = net
    
  def open(self, *args):
    pass # TODO
    # self.net.open(*args)
    
  def close(self, *args):
    pass # TODO
    # self.net.close(*args)
    
  # read data between minsize and maxsize chars.
  # optional terminator set to mark end of packet.
  # timeout if any one read takes longer than timeout period.
  # i.e. whole packet may take longer to come in
  
  def read(self, maxsize=1, minsize=None, termset=None, timeout=None):
    if minsize == None:
      minsize = maxsize
      
    remaining = maxsize
    if termset != None:
      readsz = 1
    else:
      readsz = remaining

    buf = ''
      
    while len(buf) < minsize:
      #TODO ####
      data = self.net.read(readsz)
      if (len(data) == 0):
        time.sleep(0.1) # prevent CPU hogging
      else:
        #print("just read:" + data)
        buf = buf + data
        remaining -= len(data)
        if termset != None:
          if data[0] in termset:
            break # terminator seen
        
    return buf
    
  def write(self, str):
    pass # TODO
    #self.serial.write(str)
  
# END

