#!/usr/bin/env python

# File			:	main.py
# Author		:	Sebastian Andersson
# Co Author		:	Magnus Sorensen & Jacob Norman
# Description	: 	A program that initiate three 'Teraranger Evo 60m' sensors, and then publish its data
# 					on a topic 'sensor_distance'. The data sent is stored in the sensor_values class, which
#					contain left, middle and right sensor values, as well as the error codes from each sensor

import rospy

from std_msgs.msg  import String
from periphery import I2C
from distance_sensor.msg import sensor_values

import time
import smbus

# Globals.
data = sensor_values()
pub  = rospy.Publisher('sensor_distance', data_class=sensor_values, queue_size=1000)
rospy.init_node('Teraranger', anonymous=False)
update_rate = rospy.Rate(70)

DEVICE_ADDRESS_LEFT = 0x31
DEVICE_REG_MODE_TRIGGER_LEFT = 0x62
DEVICE_REG_MODE_READ_LEFT = 0x63

DEVICE_ADDRESS_MIDDLE = 0x33
DEVICE_REG_MODE_TRIGGER_MIDDLE = 0x66
DEVICE_REG_MODE_READ_MIDDLE = 0x67 

DEVICE_ADDRESS_RIGHT = 0x35
DEVICE_REG_MODE_TRIGGER_RIGHT = 0x6a
DEVICE_REG_MODE_READ_RIGHT = 0x6b

MEAN_RANGE = 1

bus = smbus.SMBus(0)
val = [0L,0L,0L]

def main():
	rospy.loginfo(rospy.get_caller_id() + " node entered main")
	time.sleep(0.4)
	rospy.loginfo(rospy.get_caller_id() + " initialization process begun ")
	data.error = [0, 0, 0]
	try:
		bus.write_byte(DEVICE_ADDRESS_LEFT, DEVICE_REG_MODE_TRIGGER_LEFT)
	except IOError as e:
		rospy.loginfo(rospy.get_caller_id() + " ERROR [{num}] occured with left sensor 0x31".format(num=e.errno))
		return e.errno

	try:
		bus.write_byte(DEVICE_ADDRESS_MIDDLE, DEVICE_REG_MODE_TRIGGER_MIDDLE)
	except IOError as e:
		rospy.loginfo(rospy.get_caller_id() + " ERROR [{num}] occured with middle sensor 0x33".format(num=e.errno))
		return e.errno

	try:
		bus.write_byte(DEVICE_ADDRESS_RIGHT, DEVICE_REG_MODE_TRIGGER_RIGHT)
	except IOError as e:
		rospy.loginfo(rospy.get_caller_id() + " ERROR [{num}] occured with right sensor 0x35".format(num=e.errno))
		return e.errno

	time.sleep(0.8)
	rospy.loginfo(rospy.get_caller_id() + " initialization successful")
	rospy.loginfo("--------------------------------")
	rospy.loginfo(" sensors are now publishing data")
	rospy.loginfo(" Topic: sensor_distance         ")
	rospy.loginfo(" Rate : 70 Hz                   ")
	rospy.loginfo("--------------------------------")
# ----------------------------------------------------------------------------------------------------
# loop -----------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------
	while not rospy.is_shutdown():
		data.error = [0, 0, 0]

		data.left_sensor = 0
		data.middle_sensor = 0 		# Clearing the distance values
		data.right_sensor = 0

		for i in range(0,MEAN_RANGE):
# left ----------------------------------------------------------------------------------------------------
			try:
				bus.write_byte(DEVICE_ADDRESS_LEFT, DEVICE_REG_MODE_READ_LEFT)
				val = bus.read_i2c_block_data(DEVICE_ADDRESS_LEFT, 0x00)
			except IOError as e:
				#rospy.loginfo("ERROR [{num}] occured with left sensor 0x31".format(num=e.errno))
				# index 0 = sensor left
				data.error[0] = e.errno
				data.left_sensor = -1
			else:
				distance = (val[0] << 8) + val[1]
				data.left_sensor += distance
# middle ----------------------------------------------------------------------------------------------------
			try:
				bus.write_byte(DEVICE_ADDRESS_MIDDLE, DEVICE_REG_MODE_READ_MIDDLE)
				val = bus.read_i2c_block_data(DEVICE_ADDRESS_MIDDLE, 0x00)
			except IOError as e:
				#rospy.loginfo("ERROR [{num}] occured with middle sensor 0x33".format(num=e.errno))
				# index 1 = sensor middle
				data.error[1] = e.errno	
				data.middle_sensor = -1		
			else:
				distance = (val[0] << 8) + val[1]
				data.middle_sensor += distance
# right ----------------------------------------------------------------------------------------------------
			try:
				bus.write_byte(DEVICE_ADDRESS_RIGHT, DEVICE_REG_MODE_READ_RIGHT)
				val = bus.read_i2c_block_data(DEVICE_ADDRESS_RIGHT, 0x00)
			except IOError as e:
				#rospy.loginfo("ERROR [{num}] occured with right sensor 0x35".format(num=e.errno))
				# index 2 = sensor right
				data.error[2] = e.errno	
				data.right_sensor = -1
			else:			
				distance = (val[0] << 8) + val[1]
				data.right_sensor += distance
# ----------------------------------------------------------------------------------------------------
		
		data.left_sensor /= MEAN_RANGE
		data.middle_sensor /= MEAN_RANGE 	# Get mean of measurement(s)
		data.right_sensor /= MEAN_RANGE
		pub.publish(data)
		update_rate.sleep()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

