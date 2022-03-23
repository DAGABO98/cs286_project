#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    print len(msg.ranges)

def laser_scan():
    rospy.init_node('dummpy_scan')
    sub = rospy.Subscriber('/locobot/laser/scan', LaserScan, callback)

if __name__ == '__main__':
    laser_scan()
