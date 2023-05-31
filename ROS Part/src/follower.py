#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        rospy.init_node('follower')
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.min_distance = 0.2  

    def laser_scan_callback(self, scan):
        ranges = scan.ranges
	min_range = min(ranges)
	min_range_index = ranges.index(min_range)
	twist = Twist()  

        if min_range < self.min_distance:
	    if min_range_index >300 or min_range_index < 60:
            	self.follow_object_forward()
	    	rospy.loginfo("following!! forward")
	    elif min_range_index >120 and min_range_index < 240:
		self.follow_object_backward()
		rospy.loginfo("following!! backward")
	    elif min_range_index >= 60 and min_range_index <=120:
		self.follow_object_right()
		rospy.loginfo("following!! right")
	    elif min_range_index >= 240 and min_range_index <=300:
		self.follow_object_left()
		rospy.loginfo("following!! left")
	elif min_range > self.min_distance:
	    twist.linear.x = 0.0
	    twist.angular.z = 0.0
	    self.pub.publish(twist)

    def follow_object_forward(self):
        twist = Twist()
        twist.linear.x = -0.15  
        twist.angular.z = 0.0  
        self.pub.publish(twist)

    def follow_object_backward(self):
        twist = Twist()
        twist.linear.x = 0.15  
        twist.angular.z = 0.0  
        self.pub.publish(twist)

    def follow_object_right(self):
        twist = Twist()
        twist.linear.x = -0.05  
        twist.angular.z = 0.5  
        self.pub.publish(twist)

    def follow_object_left(self):
        twist = Twist()
        twist.linear.x = -0.05  
        twist.angular.z = -0.5  
        self.pub.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    follower = Follower()
    follower.run()
