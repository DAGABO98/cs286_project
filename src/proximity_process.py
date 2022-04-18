import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float32
import math
from std_msgs.msg import Float64MultiArray
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import random
import sys
import numpy as np


def euclideanDistance(a, b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)



class QuadraticCost:
    '''
    Each robot will have a cost function. 
    '''
    def __init__(self):
        # initial state and solution that evolves with time
        self.x = np.array((0, 0))

    def step(self, dList, pList):
        n = 0
        gradient = 0
        for (p, d) in zip(pList, dList):
            if d > 0:
                xp = np.linalg.norm(p - self.x)
                gradient += (self.x - p) * (1 - d / xp)
                n += 1
        
        self.x -= 1 / (10 * n) * (gradient) # update x
        return


class AOA_Processor:
    def __init__(self, name_space1="tb3_1/", name_space2="tb3_4"):

        rospy.Subscriber(str(name_space1)+'odom', Odometry, self.odom_cb1)
        rospy.Subscriber(str(name_space2)+'odom', Odometry, self.odom_cb2)

        rospy.Subscriber(str(name_space1)+'distance_multi', Float64MultiArray, self.dist1_cb)
        rospy.Subscriber(str(name_space2)+'distance_multi', Float64MultiArray, self.dist2_cb)
        
        self.name_space1 = name_space1
        self.name_space2 = name_space2

        self.start_position1 = [0.0, 0.0]
        self.start_position2 = [1.0, 0.0]

        self.postions1 = [ (0.0, 0.0) for i in range(300)]
        self.positions_index1 = 0
        self.positions2 = [ (0.0, 0.0) for i in range(300)]
        self.positions_index2 = 0

        self.current_ori1 = 0.0
        self.current_ori2 = 0.0

        self.distances_to_target1 = [-10.0 for i in range(300)]
        self.distance_index1 = 0
        self.distances_to_target2 = [-10.0 for i in range(300)]
        self.distance_index2 = 0

        self.aoa_direction1 = 0.0
        self.aoa_direction2 = 0.0

        self.cost_function1 = QuadraticCost()
        self.cost_function2 = QuadraticCost()

    def odom_cb1(self,msg):
        positions1 = msg.pose.pose.position
        self.positions1[self.positions_index1] = (positions1.x + self.start_position1[0], positions1.y + self.start_position2[1])
        self.positions_index1  = (self.positions_index1 + 1) % 300
        self.current_ori1 = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]

    def odom_cb2(self,msg):
        positions2 = msg.pose.pose.position
        self.positions2[self.positions_index2] = (positions2.x, positions2.y)
        self.positions_index2  = (self.positions_index2 + 1) % 300
        self.current_ori2 = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]

    def dist1_cb(self,msg):
        distances = []
        for tx in msg.data:
            distances.append(tx)
        diatnce_to_target = distances[-1]

        self.distances_to_target1[self.distance_index1] = distance_to_target
        self.distance_index1 = (self.distance_index1 + 1) % 300

    def dist2_cb(self,msg):
        distances = []
        for tx in msg.data:
            distances.append(tx)
        diatnce_to_target = distances[-1]

        self.distances_to_target2[self.distance_index2] = distance_to_target
        self.distance_index2 = (self.distance_index2 + 1) % 300



    def publish_aoa(self):
        pub1 = rospy.Publisher(str(self.name_space1)+"aoa_topic", Float32, queue_size=10)
        pub2 = rospy.Publisher(str(self.name_space2)+"aoa_topic", Float32, queue_size=10)
        rospy.init_node("aoa_processor", anonymous=True)
        rate = rospy.Rate(0.5) # 1 hz

        while not rospy.is_shutdown():
            rate.sleep()
            target = self.target_optim(iterations=300)
            self.aoa_direction1 = np.arctan2(target[0] - self.positions1[self.positions_index1-1][0], target[1] - self.positions1[self.positions_index1-1][1])
            self.aoa_direction2 =  np.arctan2(target[0] - self.positions2[self.positions_index2-1][0], target[1] - self.positions2[self.positions_index1-1][1])
            self.aoa_direction1 -= self.current_ori1
            self.aoa_direction2 -= self.current_ori2
            rospy.loginfo("AOA move direction 1: " + str(self.aoa_direction1))
            rospy.loginfo("AOA move direction 2: " + str(self.aoa_direction2))

            pub1.publish(self.aoa_direction1)
            pub2.publish(self.aoa_direction2)

    def target_optim(self, iterations=1000):
        for _ in range(iterations):
            self.cost_function1.step()
            self.cost_function2.step()
            t1 = 0.9 * self.cost_function1.x + 0.1 * self.cost_function2.x
            t2 = 0.1 * self.cost_function1.x + 0.9 * self.cost_function2.x
            self.cost_function1.x = t1
            self.cost_function2.x = t2
        
        return self.cost_function1.x
            

if __name__=='__main__':
    obj = AOA_Processor()

    try:
        obj.publish_aoa()
    except rospy.ROSInterruptException:
        pass
