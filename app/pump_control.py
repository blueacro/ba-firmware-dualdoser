import usb.core
import usb.util
import time
import struct

# find our device
dev = usb.core.find(idVendor=0x726c, idProduct=0x3101)

# was it found?
if dev is None:
    raise ValueError('Device not found')

# set the active configuration. With no arguments, the first
# configuration will be the active one
dev.reset()

dev.set_configuration()

# get an endpoint instance
cfg = dev.get_active_configuration()
intf = cfg[(0,0)]
usb.util.claim_interface(dev, intf)


ep = usb.util.find_descriptor(
    intf,
    # match the first OUT endpoint
    custom_match = \
    lambda e: \
        usb.util.endpoint_direction(e.bEndpointAddress) == \
        usb.util.ENDPOINT_OUT)

assert ep is not None


epin = usb.util.find_descriptor(
    intf,
    # match the first OUT endpoint
    custom_match = \
    lambda e: \
        usb.util.endpoint_direction(e.bEndpointAddress) == \
        usb.util.ENDPOINT_IN)

assert epin is not None

import datetime

while True:
# write the data
    print(datetime.datetime.now())
    for p in range (0, 2):

        for x in range(1, 100, 5):
            print(p, x)
            if p == 1:
                cmd = struct.pack("<bbb", 1, 0, x)
            else:
                cmd = struct.pack("<bbb", 1, x, 0)
            ep.write(cmd)
            print(epin.read(64))
            time.sleep(1)
    ep.write(b'\x01\x00\x00')
    print("wrote both pumps off")
    print(epin.read(64))
    time.sleep(1)

usb.util.release_interface(dev, intf)
