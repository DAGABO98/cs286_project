import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float32
import math
from std_msgs.msg import Float64MultiArray
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

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
        self.current_postion1 = None
        self.current_position2 = None
        self.current_ori1 = 0.0
        self.current_ori2 = 0.0
        self.distances1 = []
        self.distances2 = []

        self.aoa_direction1 = 0.0
        self.aoa_direction2 = 0.0

    def odom_cb1(self,msg):
        self.current_position1 = msg.pose.pose.position
        self.current_ori1 = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]

    def odom_cb2(self,msg):
        self.current_position2 = msg.pose.pose.position
        self.current_ori2 = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pos
e.pose.orientation.z, msg.pose.pose.orientation.w])[2]

    def dist1_cb(self,msg):
        distances = []
        for tx in msg.data:
            if tx == 0.0:
                continue
            else:
                distances.append(tx)
        self.distances1 = distances

    def dist2_cb(self,msg):
        distances = []
        for tx in msg.data:
            if tx == 0.0:
                continue
            else:
                distances.append(tx)
        self.distances2 = distances


    def publish_aoa(self):
        pub1 = rospy.Publisher(str(self.name_space1)+"aoa_topic", Float32, queue_size=10)
        pub2 = rospy.Publisher(str(self.name_space2)+"aoa_topic", Float32, queue_size=10)
        rospy.init_node("aoa_processor", anonymous=True)
        rate = rospy.Rate(0.5) # 1 hz
        while not rospy.is_shutdown():
            rospy.loginfo("AOA move direction 1: " + str(self.aoa_direction1))
            rospy.loginfo("AOA move direction 1: " + str(self.aoa_direction1))

            pub1.publish(self.aoa_direction1)
            pub2.publish(self.aoa_direction2)
            rate.sleep()

if __name__=='__main__':
    obj = AOA_Processor()

    try:
        obj.publish_aoa()
    except rospy.ROSInterruptException:
        pass
