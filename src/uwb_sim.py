#!/usr/bin/env python

###########################################################################
### TO DO                                                               ###
### - Improve capturing of ser data, sometimes channels misclassified   ###
### - Add /dev/ACM0 as argument
###########################################################################

# ROS-related dependencies
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

class UWB_meas:
    
    def __init__(self, name_space1="tb3_0/", name_space2="tb3_1/", name_space3="tb3_2/"):
        self.target_location = [-2.6, 1.0]

        self.curr_position1 = [-7.0, -1.0]
        self.curr_position2 = [6.0, -1.0]
        self.curr_position3 = [-.5, 3.0]

        rospy.Subscriber(str(name_space1)+'odom', Odometry, self.odom_cb1)
        rospy.Subscriber(str(name_space2)+'odom', Odometry, self.odom_cb2)
        rospy.Subscriber(str(name_space3)+'odom', Odometry, self.odom_cb3)

    def odom_cb1(self, msg):
        position = msg.pose.pose.position
        curr_pos_x = position.x
        curr_pos_y = position.y
        self.curr_position1 = [curr_pos_x, curr_pos_y]

    def odom_cb2(self, msg):
        position = msg.pose.pose.position
        curr_pos_x = position.x
        curr_pos_y = position.y
        self.curr_position2 = [curr_pos_x, curr_pos_y]

    def odom_cb3(self, msg):
        position = msg.pose.pose.position
        curr_pos_x = position.x
        curr_pos_y = position.y
        self.curr_position3 = [curr_pos_x, curr_pos_y]

    def get_dist(self):
        robot_pos1 = np.array(self.curr_position1)
        robot_pos2 = np.array(self.curr_position2)
        robot_pos3 = np.array(self.curr_position3)
        target_pos = np.array(self.target_location)

        dist1 = np.linalg.norm(target_pos-robot_pos1)
        dist2 = np.linalg.norm(target_pos-robot_pos2)
        dist3 = np.linalg.norm(target_pos-robot_pos3)

        self.dist1 = [0.0, dist1]
        self.dist2 = [0.0, dist2]
        self.dist3 = [0.0, dist3]

        log_string = "Distance1: "+str(dist1)+" Distance2: "+str(dist2)+" Distance3: "+str(dist3)

        rospy.loginfo(log_string)

def publish_dist(name_space1="tb3_0/", name_space2="tb3_1/", name_space3="tb3_2/"):

    pub1 = rospy.Publisher(name_space1+'distance_multi', Float64MultiArray, queue_size=10)
    pub2 = rospy.Publisher(name_space2+'distance_multi', Float64MultiArray, queue_size=10)
    pub3 = rospy.Publisher(name_space3+'distance_multi', Float64MultiArray, queue_size=10)

    rospy.init_node('uwb', anonymous=True)
    my_msg1 = Float64MultiArray()
    my_msg2 = Float64MultiArray()
    my_msg3 = Float64MultiArray()

    rate = rospy.Rate(10) # 5hz
    sensor = UWB_meas(name_space1, name_space2, name_space3)
    while not rospy.is_shutdown():
        # Get measurement from sensor
        sensor.get_dist()

        # Set up message
        my_msg1.data = sensor.dist1
        my_msg2.data = sensor.dist2
        my_msg3.data = sensor.dist3

        # publish distance measurement
        pub1.publish(my_msg1)
        pub2.publish(my_msg2)
        pub3.publish(my_msg3)
        rate.sleep()

if __name__ == '__main__':
    print("Start")
    
    # Publish sensor data
    try:
        publish_dist()
    except rospy.ROSInterruptException as e:
        print(e)
        pass        
