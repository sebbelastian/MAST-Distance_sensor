#!/usr/bin/env python
import rospy

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " Range: %f", data.data)

def main():
	rospy.init_node('subscriber',anonymous=False)
	rospy.Subscriber("sensor_distance", sensor_values, callback)
	rospy.spin()

if __name__ == '__main__':
	main()
