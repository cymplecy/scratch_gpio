#!/bin/python
import struct

# This example demonstrates how to read a binary file, by reading the width and
# height information from a bitmap file. First, the bytes are read, and then
# they are converted to integers.

# When reading a binary file, always add a 'b' to the file open mode
with open('test.bmp', 'rb') as f:
    # BMP files store their width and height statring at byte 18 (12h), so seek
    # to that position
    f.seek(10)

    # The width and height are 4 bytes each, so read 8 bytes to get both of them
    bytes = f.read(4)

    # Here, we decode the byte array from the last step. The width and height
    # are each unsigned, little endian, 4 byte integers, so they have the format
    # code '<II'. See http://docs.python.org/3/library/struct.html for more info
    bmpdata = int(struct.unpack('<I', bytes)[0])
    print bmpdata

    # Print the width and height of the image
    print('Data starts at:  ' + str(bmpdata))
    #print('Image height: ' + str(size[1]))
    f.seek(bmpdata)
    for i in range(0,64):
        bytes = f.read(3)
        pixel = struct.unpack('BBB',  bytes)#[0]
        #pixel = struct.unpack('<I', bytes)[0]
        print i,pixel
        