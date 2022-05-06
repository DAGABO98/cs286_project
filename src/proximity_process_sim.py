import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float32
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction , MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
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
        self.x = np.array((2.0, 2.0))
        self.t = 0

    def step(self, pList, dList, step_size=0.9):
        n = 0.0
        gradient = np.array((0.0, 0.0))

        for (p, d) in zip(pList, dList):
            if d > 0:
                xp = np.linalg.norm(p - self.x)
                gradient += (self.x - p) * (1 - d / xp)
                n += 1
        if self.t < 5000:
            self.x -= (1 / (step_size * n)) * (gradient) # update x
        self.t += 1
        return

class RobotSLAM_Nav:
    def __init__(self, name_space):
        self.client = actionlib.SimpleActionClient(str(name_space)+"move_base", MoveBaseAction)
        #Create the actionlib server
        self.client.wait_for_server()

        # Initialize the variable for the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = str(name_space)+"map"
        self.timeout = 60
        self.step_size = 0.6

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

    def cancel_target(self):
        self.client.cancel_goal()

    def move_towards_target(self, move_direction, current_ori, current_position):
        x = current_position[0] + self.step_size*math.cos(move_direction)
        y = current_position[1] + self.step_size*math.sin(move_direction)

        print("Moving to next location x = ",x, ", y =",y)

        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0,0, current_ori)

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
    def __init__(self, name_space1="tb3_0/", name_space2="tb3_1/", name_space3="tb3_2/"):

        rospy.Subscriber(str(name_space1)+'odom', Odometry, self.odom_cb1)
        rospy.Subscriber(str(name_space2)+'odom', Odometry, self.odom_cb2)
        rospy.Subscriber(str(name_space3)+'odom', Odometry, self.odom_cb3)


        rospy.Subscriber(str(name_space1)+'distance_multi', Float64MultiArray, self.dist1_cb)
        rospy.Subscriber(str(name_space2)+'distance_multi', Float64MultiArray, self.dist2_cb)
        rospy.Subscriber(str(name_space3)+'distance_multi', Float64MultiArray, self.dist3_cb)

        rospy.Subscriber(str(name_space1)+"within_threshold", Float32, self.within_threshold1_cb)
        rospy.Subscriber(str(name_space2)+"within_threshold", Float32, self.within_threshold2_cb)
        rospy.Subscriber(str(name_space2)+"within_threshold", Float32, self.within_threshold2_cb)

        self.name_space1 = name_space1
        self.name_space2 = name_space2
        self.name_space3 = name_space3

        self.start_position1 = [-7.0, -1.0]
        self.start_position2 = [6.0, -1.0]
        self.start_position3 = [0.5, 3.0]

        self.positions1 = [ (-7.0, -1.0) for i in range(100)]
        self.positions_index1 = 0
        self.positions2 = [ (6.0, -1.0) for i in range(100)]
        self.positions_index2 = 0
        self.positions3 = [ (0.5, 3.0) for i in range(100)]
        self.positions_index3 = 0

        self.current_ori1 = 0.0
        self.current_ori2 = 0.0
        self.current_ori3 = 0.0

        self.distances_to_target1 = [-10.0 for i in range(100)]
        self.distance_index1 = 0
        self.distances_to_target2 = [-10.0 for i in range(100)]
        self.distance_index2 = 0
        self.distances_to_target3 = [-10.0 for i in range(100)]
        self.distance_index3 = 0

        self.aoa_direction1 = 0.0
        self.aoa_direction2 = 0.0
        self.aoa_direction3 = 0.0

        self.move_direction1 = 0.0
        self.move_direction2 = 0.0
        self.move_direction3 = 0.0

        self.within_threshold1 = False
        self.within_threshold2 = False
        self.within_threshold3 = False

        self.first_pub1 = True
        self.first_pub2 = True
        self.first_pub3 = True


        self.target1 = None
        self.target2 = None
        self.target3 = None

        self.stop1 = False
        self.stop2 = False
        self.stop3 = False

        self.cost_function1 = QuadraticCost()
        self.cost_function2 = QuadraticCost()
        self.cost_function3 = QuadraticCost()

    def odom_cb1(self,msg):
        positions1 = msg.pose.pose.position
        self.positions1[self.positions_index1] = (positions1.x, positions1.y)
        self.positions_index1  = (self.positions_index1 + 1) % 100
        self.current_ori1 = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]

    def odom_cb2(self,msg):
        positions2 = msg.pose.pose.position
        self.positions2[self.positions_index2] = (positions2.x, positions2.y)
        self.positions_index2  = (self.positions_index2 + 1) % 100
        self.current_ori2 = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]

    def odom_cb3(self,msg):
        positions3 = msg.pose.pose.position
        self.positions3[self.positions_index3] = (positions3.x, positions3.y)
        self.positions_index3  = (self.positions_index3 + 1) % 100
        self.current_ori3 = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]

    def dist1_cb(self,msg):
        distances = []
        for tx in msg.data:
            distances.append(tx)
        distance_to_target = distances[-1]

        if distance_to_target < 2:
            self.within_threshold1 = True

        if distance_to_target <= 0.5:
            self.stop1 = True
        # else:
        #     self.stop1 = False

        self.distances_to_target1[self.distance_index1] = distance_to_target
        self.distance_index1 = (self.distance_index1 + 1) % 100

    def dist2_cb(self,msg):
        distances = []
        for tx in msg.data:
            distances.append(tx)
        distance_to_target = distances[-1]

        if distance_to_target < 2:
            self.within_threshold2 = True

        if distance_to_target <= 0.5:
            self.stop2 = True
        # else:
        #     self.stop2 = False

        self.distances_to_target2[self.distance_index2] = distance_to_target
        self.distance_index2 = (self.distance_index2 + 1) % 100
    
    def dist3_cb(self,msg):
        distances = []
        for tx in msg.data:
            distances.append(tx)
        distance_to_target = distances[-1]

        if distance_to_target < 2:
            self.within_threshold3 = True

        if distance_to_target <= 0.5:
            self.stop3 = True
        # else:
        #     self.stop2 = False

        self.distances_to_target3[self.distance_index3] = distance_to_target
        self.distance_index3 = (self.distance_index3 + 1) % 100

    def within_threshold1_cb(self, msg):
        if msg.data == 1000.0:
            move_robot = RobotSLAM_Nav(self.name_space1)
            while True:
                if self.stop1:
                    move_robot.cancel_target()
                    break
                else:
                    target = [self.target1.pose.position.x, self.target1.pose.position.y]
                    #move_robot.move_towards_target(self.move_direction1, self.current_ori1, self.positions1[-1])
                    move_robot.move(target, 0.0)

    def within_threshold2_cb(self, msg):
        if msg.data == 1000.0:
            move_robot = RobotSLAM_Nav(self.name_space2)
            while True:
                if self.stop2:
                    move_robot.cancel_target()
                    break
                else:
                    target = [self.target2.pose.position.x, self.target2.pose.position.y]
                    #move_robot.move_towards_target(self.move_direction2, self.current_ori2, self.positions2[-1])
                    move_robot.move(target, 0.0)

    def within_threshold3_cb(self, msg):
        if msg.data == 1000.0:
            move_robot = RobotSLAM_Nav(self.name_space3)
            while True:
                if self.stop3:
                    move_robot.cancel_target()
                    break
                else:
                    target = [self.target3.pose.position.x, self.target3.pose.position.y]
                    #move_robot.move_towards_target(self.move_direction2, self.current_ori2, self.positions2[-1])
                    move_robot.move(target, 0.0)

    def publish_aoa(self):
        pub1 = rospy.Publisher(str(self.name_space1)+"aoa_topic", Float32, queue_size=10)
        pub2 = rospy.Publisher(str(self.name_space2)+"aoa_topic", Float32, queue_size=10)
        pub3 = rospy.Publisher(str(self.name_space3)+"aoa_topic", Float32, queue_size=10)

        pubt3 = rospy.Publisher(str(self.name_space3)+"within_threshold", Float32, queue_size=1)
        pubt2 = rospy.Publisher(str(self.name_space2)+"within_threshold", Float32, queue_size=1)
        pubt1 = rospy.Publisher(str(self.name_space1)+"within_threshold", Float32, queue_size=1)
        pub_target1 = rospy.Publisher(str(self.name_space1) + "optimized_target_topic", Marker, queue_size=10) #move_base_simple/goal
        pub_target2 = rospy.Publisher(str(self.name_space2)+"optimized_target_topic", Marker, queue_size=10) #optimized_target_topic
        pub_target3 = rospy.Publisher(str(self.name_space3)+"optimized_target_topic", Marker, queue_size=10)
        
        rospy.init_node("aoa_processor", anonymous=True)
        rate = rospy.Rate(0.5) # 1 hz

        while not rospy.is_shutdown():
            rate.sleep()
            target = self.target_optim(iterations=500)
            print("Optimization output = " + str(target))
            p = Point()
            p.x = target[0]
            p.y = target[1]
            p.z = 0.0
            self.target = p

            if self.within_threshold1:
                self.aoa_direction1 = 1000.0
                if self.first_pub1:
                    self.first_pub1 = False
                    self.move_direction1 = np.arctan2(target[1] - self.positions1[self.positions_index1-1][1], target[0] - self.positions1[self.positions_index1-1][0]) * 180/np.pi
                    self.move_direction1 -= self.current_ori1 * 180 / np.pi
                    pubt1.publish(self.aoa_direction1)

            else:
                self.aoa_direction1 = np.arctan2(target[1] - self.positions1[self.positions_index1-1][1], target[0] - self.positions1[self.positions_index1-1][0]) * 180/np.pi
                self.aoa_direction1 -= self.current_ori1 * 180 / np.pi

            if self.within_threshold2:
                self.aoa_direction2 = 1000.0
                if self.first_pub2:
                    self.first_pub2 = False
                    self.move_direction2 = np.arctan2(target[1] - self.positions2[self.positions_index2-1][1], target[0] - self.positions2[self.positions_index2-1][0]) * 180/np.pi
                    self.move_direction2 -= self.current_ori2 * 180 / np.pi
                    pubt2.publish(self.aoa_direction2)
            else:
                self.aoa_direction2 = np.arctan2(target[1] - self.positions2[self.positions_index2-1][1], target[0] - self.positions2[self.positions_index2-1][0]) * 180/np.pi
                self.aoa_direction2 -= self.current_ori2 * 180 / np.pi

            if self.within_threshold3:
                self.aoa_direction3 = 1000.0
                if self.first_pub3:
                    self.first_pub3 = False
                    self.move_direction3 = np.arctan2(target[1] - self.positions3[self.positions_index3-1][1], target[0] - self.positions3[self.positions_index3-1][0]) * 180/np.pi
                    self.move_direction3 -= self.current_ori3 * 180 / np.pi
                    pubt3.publish(self.aoa_direction3)
            else:
                self.aoa_direction3 = np.arctan2(target[1] - self.positions3[self.positions_index3-1][1], target[0] - self.positions3[self.positions_index3-1][0]) * 180/np.pi
                self.aoa_direction3 -= self.current_ori3 * 180 / np.pi

            rospy.loginfo("AOA move directions: " + str([self.aoa_direction1, self.aoa_direction2, self.aoa_direction3]))
            rospy.loginfo("Robot Locations: " + str([self.positions1[-1], self.positions2[-1], self.positions3[-1]]))
            rospy.loginfo("Robot Orientations: " + str([self.current_ori1 * 180 / np.pi, self.current_ori2 * 180 / np.pi, self.current_ori3 * 180/np.pi]))

            pub1.publish(self.aoa_direction1)
            pub2.publish(self.aoa_direction2)
            pub3.publish(self.aoa_direction3)

            # Marker for target 1
            p1 = Marker()
            p1.header.frame_id = str(self.name_space1)+"map"
            p1.type = p1.CUBE
            p1.action = 0
            p1.scale.x = 0.2
            p1.scale.y = 0.2
            p1.scale.z = 0.2
            p1.color.a = 255
            p1.color.r = 0
            p1.color.g = 255
            p1.color.b = 0
            p1.pose.orientation.w = 1.0
            p1.pose.position.x = target[0]
            p1.pose.position.y = target[1]
            p1.pose.position.z = 0.0

            # Marker for target 2
            p2 = Marker()
            p2.header.frame_id = str(self.name_space2)+"map"
            p2.type = p2.CUBE
            p2.action = p2.ADD
            p2.scale.x = 0.2
            p2.scale.y = 0.2
            p2.scale.z = 0.2
            p2.color.a = 255
            p2.color.r = 0
            p2.color.g = 255
            p2.color.b = 0
            p2.pose.orientation.w = 1.0
            p2.pose.position.x = target[0]
            p2.pose.position.y = target[1]
            p2.pose.position.z = 0.0
            self.target1 = p1
            self.target2 = p2
            
            # Marker for target 3
            p3 = Marker()
            p3.header.frame_id = str(self.name_space3)+"map"
            p3.type = p3.CUBE
            p3.action = p3.ADD
            p3.scale.x = 0.2
            p3.scale.y = 0.2
            p3.scale.z = 0.2
            p3.color.a = 255
            p3.color.r = 0
            p3.color.g = 255
            p3.color.b = 0
            p3.pose.orientation.w = 1.0
            p3.pose.position.x = target[0]
            p3.pose.position.y = target[1]
            p3.pose.position.z = 0.0
            
            self.target1 = p1
            self.target2 = p2
            self.target3 = p3
            pub_target1.publish(p1)
            pub_target2.publish(p2)
            pub_target3.publish(p3)


    def target_optim(self, iterations=1000):
        positions1 = copy.deepcopy(self.positions1)
        positions2 = copy.deepcopy(self.positions2)
        positions3 = copy.deepcopy(self.positions3)
        distances_to_target1 = copy.deepcopy(self.distances_to_target1)
        distances_to_target2 = copy.deepcopy(self.distances_to_target2)
        distances_to_target3 = copy.deepcopy(self.distances_to_target3)
        
        for _ in range(iterations):
            self.cost_function1.step(positions1, distances_to_target1)
            self.cost_function2.step(positions2, distances_to_target2)
            self.cost_function3.step(positions3, distances_to_target3)
            t1 = 0.6 * self.cost_function1.x + 0.2 * self.cost_function2.x + 0.2 * self.cost_function3.x
            t2 = 0.2 * self.cost_function1.x + 0.6 * self.cost_function2.x + 0.2 * self.cost_function3.x
            t3 = 0.2 * self.cost_function1.x + 0.2 * self.cost_function2.x + 0.6 * self.cost_function3.x
            self.cost_function1.x = t1
            self.cost_function2.x = t2
            self.cost_function3.x = t3

        return 0.33*self.cost_function1.x + 0.33*self.cost_function2.x + 0.33*self.cost_function3.x


if __name__=='__main__':
    obj = AOA_Processor()

    try:
        obj.publish_aoa()
    except rospy.ROSInterruptException:
        pass
