import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float32
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction , MoveBaseGoal
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float64MultiArray
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import random
import sys
import copy
import numpy as np


def euclideanDistance(a, b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

class QuadraticCost:
    '''
    Each robot will have a cost function. 
    '''
    def __init__(self):
        # initial state and solution that evolves with time
        self.x = np.array((-1.0, -1.0))

    def step(self, pList, dList):
        n = 0.0
        gradient = np.array((0.0, 0.0))
        
        for (p, d) in zip(pList, dList):
            if d > 0:
                xp = np.linalg.norm(p - self.x)
                gradient += (self.x - p) * (1 - d / xp)
                n += 1
        self.x -= (1 / (10 * n)) * (gradient) # update x
        return

class RobotSLAM_Nav:
    def __init__(self, name_space):
        rospy.init_node("move_base_tester")
        self.client = actionlib.SimpleActionClient(str(name_space)+"move_base", MoveBaseAction)
        #Create the actionlib server
        self.client.wait_for_server()

        #Initialize the variable for the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = str(name_space)+"map"
	self.timeout = 60

    def move(self, target, orientation):
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = float(target[0])
        self.goal.target_pose.pose.position.y = float(target[1])
            
        q = quaternion_from_euler(0,0, orientation)

        self.goal.target_pose.pose.orientation.x = q[0]
        self.goal.target_pose.pose.orientation.y = q[1]
        self.goal.target_pose.pose.orientation.z = q[2]
        self.goal.target_pose.pose.orientation.w = q[3]
            
        rospy.loginfo("Attempting to move to the goal")
        self.client.send_goal(self.goal)
        wait=self.client.wait_for_result(rospy.Duration(self.timeout))

        if not wait:
            rospy.loginfo("Timed-out after failing to reach the goal.")
            self.client.cancel_goal()
            rospy.loginfo("Please provide a new goal position")
        else:
            rospy.loginfo("Reached goal successfully")


class AOA_Processor:
    def __init__(self, name_space1="tb3_1/", name_space2="tb3_4/"):

        rospy.Subscriber(str(name_space1)+'odom', Odometry, self.odom_cb1)
        rospy.Subscriber(str(name_space2)+'odom', Odometry, self.odom_cb2)

        rospy.Subscriber(str(name_space1)+'distance_multi', Float64MultiArray, self.dist1_cb)
        rospy.Subscriber(str(name_space2)+'distance_multi', Float64MultiArray, self.dist2_cb)

        rospy.Subscriber(str(name_space1)+"within_threshold", Float32, self.within_threshold1_cb)
        rospy.Subscriber(str(name_space2)+"within_threshold", Float32, self.within_threshold2_cb)
        
        self.name_space1 = name_space1
        self.name_space2 = name_space2

        self.start_position1 = [0.0, 0.0]
        self.start_position2 = [1.0, 0.0]

        self.positions1 = [ (0.0, 0.0) for i in range(50)]
        self.positions_index1 = 0
        self.positions2 = [ (1.0, 0.0) for i in range(50)]
        self.positions_index2 = 0

        self.current_ori1 = 0.0
        self.current_ori2 = 0.0

        self.distances_to_target1 = [-10.0 for i in range(50)]
        self.distance_index1 = 0
        self.distances_to_target2 = [-10.0 for i in range(50)]
        self.distance_index2 = 0

        self.aoa_direction1 = 0.0
        self.aoa_direction2 = 0.0

        self.within_threshold1 = False
        self.within_threshold2 = False
        self.first_pub1 = True
        self.first_pub2 = True

        self.target = None

        self.cost_function1 = QuadraticCost()
        self.cost_function2 = QuadraticCost()

    def odom_cb1(self,msg):
        positions1 = msg.pose.pose.position
        self.positions1[self.positions_index1] = (positions1.x + self.start_position1[0], positions1.y + self.start_position1[1])
        self.positions_index1  = (self.positions_index1 + 1) % 50
        self.current_ori1 = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]

    def odom_cb2(self,msg):
        positions2 = msg.pose.pose.position
        self.positions2[self.positions_index2] = (positions2.x + self.start_position2[0], positions2.y + self.start_position2[1])
        self.positions_index2  = (self.positions_index2 + 1) % 50
        self.current_ori2 = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]

    def dist1_cb(self,msg):
        distances = []
        for tx in msg.data:
            distances.append(tx)
        distance_to_target = distances[-1]

        if distance_to_target < 0.8:
            self.within_threshold1 = True

        if distance_to_target <= 0.35:
            self.stop1 = True
        else:
            self.stop1 = False

        self.distances_to_target1[self.distance_index1] = distance_to_target
        self.distance_index1 = (self.distance_index1 + 1) % 50

    def dist2_cb(self,msg):
        distances = []
        for tx in msg.data:
            distances.append(tx)
        distance_to_target = distances[-1]

        if distance_to_target < 0.8:
            self.within_threshold2 = True

        if distance_to_target <= 0.35:
            self.stop2 = True
        else:
            self.stop2 = False

        self.distances_to_target2[self.distance_index2] = distance_to_target
        self.distance_index2 = (self.distance_index2 + 1) % 50
    
    def within_threshold1_cb(self, msg):
        if msg.data == 1000.0:
            move_robot = RobotSLAM_Nav(self.name_space1)
            while True:
                if self.stop1:
                    continue
                else:
                    target = [self.target[0]-self.init_position1[0], self.target[1]-self.init_position1[1]]
                    move_robot.move(target, self.current_ori1)

    def within_threshold2_cb(self, msg):
        if msg.data == 1000.0:
            move_robot = RobotSLAM_Nav(self.name_space2)
            while True:
                if self.stop2:
                    continue
                else:
                    target = [self.target[0]-self.init_position2[0], self.target[1]-self.init_position2[1]]
                    move_robot.move(target, self.current_ori2)

    def publish_aoa(self):
        pub1 = rospy.Publisher(str(self.name_space1)+"aoa_topic", Float32, queue_size=10)
        pub2 = rospy.Publisher(str(self.name_space2)+"aoa_topic", Float32, queue_size=10)
        rospy.init_node("aoa_processor", anonymous=True)
        rate = rospy.Rate(0.5) # 1 hz

        while not rospy.is_shutdown():
            rate.sleep()
            target = self.target_optim(iterations=100)
            self.target = target
            
            if self.within_threshold1:
                self.aoa_direction1 = 1000.0
                if self.first_pub1:
                    self.first_pub1 = False
                    pubt1 = rospy.Publisher(str(self.name_space1)+"within_threshold", Float32, queue_size=1)
                    pubt1.publish(self.aoa_direction1)
                
            else:
                self.aoa_direction1 = np.arctan2(target[1] - self.positions1[self.positions_index1-1][1], target[0] - self.positions1[self.positions_index1-1][0]) * 180/np.pi
                self.aoa_direction1 -= self.current_ori1

            if self.within_threshold2:
                self.aoa_direction2 = 1000.0
                if self.first_pub2:
                    self.first_pub2 = False
                    pubt2 = rospy.Publisher(str(self.name_space2)+"within_threshold", Float32, queue_size=1)
                    pubt2.publish(self.aoa_direction2)
            else:
                self.aoa_direction2 = np.arctan2(target[1] - self.positions2[self.positions_index2-1][1], target[0] - self.positions2[self.positions_index2-1][0]) * 180/np.pi
                self.aoa_direction2 -= self.current_ori2
            rospy.loginfo("AOA move direction 1: " + str(self.aoa_direction1))
            rospy.loginfo("AOA move direction 2: " + str(self.aoa_direction2))

            pub1.publish(self.aoa_direction1)
            pub2.publish(self.aoa_direction2)



    def target_optim(self, iterations=1000):
        positions1 = copy.deepcopy(self.positions1)
        positions2 = copy.deepcopy(self.positions2)
        distances_to_target1 = copy.deepcopy(self.distances_to_target1)
        distances_to_target2 = copy.deepcopy(self.distances_to_target2)
        for _ in range(iterations):
            self.cost_function1.step(positions1, distances_to_target1)
            self.cost_function2.step(positions2, distances_to_target2)
            t1 = 0.9 * self.cost_function1.x + 0.1 * self.cost_function2.x
            t2 = 0.1 * self.cost_function1.x + 0.9 * self.cost_function2.x
            self.cost_function1.x = t1
            self.cost_function2.x = t2
        
        return 0.5*self.cost_function1.x + 0.5*self.cost_function2.x
            

if __name__=='__main__':
    obj = AOA_Processor()

    try:
        obj.publish_aoa()
    except rospy.ROSInterruptException:
        pass
