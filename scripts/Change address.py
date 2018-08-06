#!/usr/bin/env python
# licence removed for brevity
from std_msgs.msg import Float64
from periphery import I2C

import time
import smbus
import rospy

def sensors():

	bus = smbus.SMBus(0)

	DEVICE_ADDRESS = 0x31    													# Base address
	DEVICE_NEW_ADDRESS = 0x33
	NEW_ADDRESS_COMMAND = 0xa2
	

	val = [0L,0L,0L]															# 1:st and 2:nd byte is data, 3:rd is checksum

	bus.write_byte(DEVICE_ADDRESS, NEW_ADDRESS_COMMAND)							# Trigger sensor for reading
	bus.write_byte(DEVICE_ADDRESS, DEVICE_NEW_ADDRESS)
	time.sleep(1)																# Delay 1s


if __name__ == '__main__':
	try:
		sensors()
	except rospy.ROSInterruptException:
		pass
		

