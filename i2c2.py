import smbus

def getDevicesOnBus(busNo):
    devices = []
    bus = SMBus(busNo)
    for addr in range(3, 178):
        try:
            bus.write_quick(addr)
            devices += [addr]
        except IOError:
            pass
    return devices
