Modification in dynamixel_driver:
/opt/ros/groovy/stacks/dynamixel_motor/dynamixel_driver/src/dynamixel_driver/dynamixel_io.py:
In def __write_serial(self, data):
Add self.ser.read(len(data)) to the end

(file from package ptu: see there...)