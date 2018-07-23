#!/usr/bin/env python
import rospy
from distance_sensor.msg import sensor_values

def dembitches(data):
	rospy.loginfo(rospy.get_caller_id() + " Range_left: %f", data.left_sensor)
	rospy.loginfo(rospy.get_caller_id() + " Range_middle: %f", data.middle_sensor)
	rospy.loginfo(rospy.get_caller_id() + " Range_right: %f", data.right_sensor)
	rospy.loginfo("-------------------------------")

def main():
	rospy.init_node('subscriber',anonymous=False)
	rospy.Subscriber("sensor_distance", sensor_values, callback=dembitches)
	rospy.spin()

if __name__ == '__main__':
	main()
