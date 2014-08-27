#!/usr/bin/python
import IPython
from dynamixel_driver import dynamixel_io
from dynamixel_driver import dynamixel_const
from dynamixel_driver.dynamixel_const import  *

port = "/dev/robot/ttyArm"
baud = 400000
id_max = 30

print "Dynamixel shell, opening %s @%d baud..." % (port, baud)

try:
    D = dynamixel_io.DynamixelIO(port, baud, True)
except e:
    print "Failed to open port."
    exit(1)

print "Pinging motors 0..%d..." % (id_max)
for id in range(id_max):   #  max: range(253)
    if D.ping(id):
        pos = D.get_position(id)
        print "ID %3d found, position %4d" % (id, pos)




# r = D.read(id, DXL_PRESENT_POSITION_L, 2); pos = r[5] + (r[6] << 8); pos

# Read / write the acceleration
# r = D.read(id, 73, 1); acc = r[5]; acc
# r = D.write(id, 73, (1,)); r

# D.write(10, 73, (20,));
# D.write(11, 73, (15,));
# D.write(12, 73, (20,));
# D.write(13, 73, (20,));


str = "DynamixelIO object is in D. Use with D.<tab>"
print
print str
print "=" * len(str)
print
print

IPython.embed()