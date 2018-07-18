#!/usr/bin/env python
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
update_rate = rospy.Rate(5)

DEVICE_ADDRESS_LEFT = 0x31
DEVICE_REG_MODE_TRIGGER_LEFT = 0x62
DEVICE_REG_MODE_READ_LEFT = 0x63

DEVICE_ADDRESS_MIDDLE = 0x33
DEVICE_REG_MODE_TRIGGER_MIDDLE = 0x66
DEVICE_REG_MODE_READ_MIDDLE = 0x67 

DEVICE_ADDRESS_RIGHT = 0x35
DEVICE_REG_MODE_TRIGGER_RIGHT = 0x6a
DEVICE_REG_MODE_READ_RIGHT = 0x6b

bus = smbus.SMBus(0)
val = [0L,0L,0L]

def main():

	bus.write_byte(DEVICE_ADDRESS_LEFT, DEVICE_REG_MODE_TRIGGER_LEFT)
	bus.write_byte(DEVICE_ADDRESS_MIDDLE, DEVICE_REG_MODE_TRIGGER_MIDDLE)
	bus.write_byte(DEVICE_ADDRESS_RIGHT, DEVICE_REG_MODE_TRIGGER_RIGHT)
	time.sleep(0.2)

	while not rospy.is_shutdown():
	
		bus.write_byte(DEVICE_ADDRESS_LEFT, DEVICE_REG_MODE_READ_LEFT)
		val = bus.read_i2c_block_data(DEVICE_ADDRESS_LEFT, 0x00)
		distance = (val[0] << 8) + val[1]
		data.left_sensor = distance
		
		bus.write_byte(DEVICE_ADDRESS_MIDDLE, DEVICE_REG_MODE_READ_MIDDLE)
		val = bus.read_i2c_block_data(DEVICE_ADDRESS_MIDDLE, 0x00)
		distance = (val[0] << 8) + val[1]
		data.middle_sensor = distance

		bus.write_byte(DEVICE_ADDRESS_RIGHT, DEVICE_REG_MODE_READ_RIGHT)
		val = bus.read_i2c_block_data(DEVICE_ADDRESS_RIGHT, 0x00)
		distance = (val[0] << 8) + val[1]
		data.right_sensor = distance
		
		pub.publish(data)
		update_rate.sleep()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

