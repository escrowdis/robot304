#!/usr/bin/env python
import rospy
from smbus2 import SMBus, i2c_msg
import time
import geometry_msgs.msg

bus = SMBus(1)
ADDR_SLAVE = 0x04
shift_data = 128

def sendI2CData(data):
  # Shift data from [-128, 127] -> [0, 255] for I2C data transport
  msg = i2c_msg.write(ADDR_SLAVE,
        [int(data.x + shift_data), int(data.y + shift_data)])

  bus.i2c_rdwr(msg)

  # TODO: format unset
  # readEncoder()

  return 0

def readEncoder():
  msg = i2c_msg.read(ADDR_SLAVE, 4)
  bus.i2c_rdwr(msg)
  print("readEncoder ---")
  for val in msg:
    print(val)
  print("---------------")

  return 0

def callback(data):
  rospy.loginfo(rospy.get_caller_id() + "received motor values: %d, %d", data.x, data.y)
  sendI2CData(data)

def listener():
  rospy.init_node('motor', anonymous = True)
  rospy.Subscriber("motor_value", geometry_msgs.msg.Point32, callback)
  rospy.spin()

if __name__ == '__main__':
  listener()
