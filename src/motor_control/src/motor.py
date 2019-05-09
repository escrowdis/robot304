#!/usr/bin/env python
import rospy
import smbus
import time
from std_msgs.msg import String

bus = smbus.SMBus(1)
address_left = 0x04
#address_right = 0x05

def sendleft(value):
  bus.write_byte(address_left,value)
  return -1

#def sendright(value):
#  bus.write_byte(address_right,value)
#  return -1

def callback(data):
  rospy.loginfo(rospy.get_caller_id() + "received", data)
  value = data.split()
  left = value[0]
  right = value[1]
  sendleft(left)
#  sendright(right)


def listener():
  rospy.init_node('motor',anonymous = True)
  rospy.Subscriber("motor_value",String, callback)
  rospy.spin()

if __name__ == '__main__':


 #constant motor output for testing
  for i in range(0,3):
    for j in range(0,200):
      print("start sending")
      print(j)
      sendleft(j)
#      sendright(i)
      time.sleep(.1)

    for k in range(200,0):
      sendleft(k)
#      sendright(i)
      time.sleep(.3)


  listener()
