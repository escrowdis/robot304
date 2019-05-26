#!/usr/bin/env python
import rospy
from smbus2 import SMBus, i2c_msg
import time
import struct
import geometry_msgs.msg
import threading

bus = SMBus(1)
ADDR_SLAVE = 0x04
shift_data = 128
LEN_DATA_I2C_IN = 16

# interval time to read I2C data. unit: Second
interval2Read = 0.5
i2cReading = False
threadRunning = False

def sendI2CData(data):
  # Shift data from [-128, 127] -> [0, 255] for I2C data transport
  msg = i2c_msg.write(ADDR_SLAVE,
        [int(data.x + shift_data), int(data.y + shift_data)])

  bus.i2c_rdwr(msg)

  return 0

def readEncoder():
  global i2cReading, threadRunning, interval2Read
  if i2cReading or not threadRunning:
    return 1
  i2cReading = True
  msg = i2c_msg.read(ADDR_SLAVE, LEN_DATA_I2C_IN)
  bus.i2c_rdwr(msg)
  if not threadRunning:
    i2cReading = False
    return 1

  print("readEncoder ---")
  data = list(msg)
  pos_l = struct.unpack('<f', ''.join(chr(i) for i in data[:4]))
  pos_r = struct.unpack('<f', ''.join(chr(i) for i in data[4:8]))
  vel_l = struct.unpack('<f', ''.join(chr(i) for i in data[8:12]))
  vel_r = struct.unpack('<f', ''.join(chr(i) for i in data[12:16]))
  print("{}, {}, {}, {}".format(pos_l, pos_r, vel_l, vel_r))
  print("---------------")
  i2cReading = False
  if threadRunning:
    threading.Timer(interval2Read, readEncoder).start()

  return 0

def stop():
  global threadRunning, threadRead
  print("readEncoder() stopping...")
  threadRunning = False
  threadRead.join()
  print("readEncoder() stopped")

def callback(data):
  rospy.loginfo(rospy.get_caller_id() + "received motor values: %d, %d", data.x, data.y)
  sendI2CData(data)

def listener():
  global threadRunning, threadRead
  rospy.init_node('motor', anonymous = True)
  rospy.Subscriber("motor_value", geometry_msgs.msg.Point32, callback)
  rospy.on_shutdown(stop)

  threadRunning = True
  threadRead = threading.Thread(target = readEncoder)
  threadRead.start()

  rospy.spin()

if __name__ == '__main__':
  listener()
