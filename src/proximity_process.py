import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float32
import math
from wsr_toolbox_cpp.msg import wsr_aoa_array

class AOA_Processor:
    def __init__(self):

        rospy.Subscriber('wsr_aoa_topic', wsr_aoa_array, self.wsr_cb)
        
        self.aoa_direction = 0.0

    def wsr_cb(self,msg):
        print("######################### Got message ######################")
        for tx in msg.aoa_array:
            print("=========== ID: "+ tx.id +" =============")
            print("TOP N AOA azimuth peak: "+ str(tx.aoa_azimuth))
            print("TOp N AOA elevation peak: "+ str(tx.aoa_elevation))
            print("Profile variance: "+ str(tx.profile_variance))
            self.aoa_direction = tx.aoa_azimuth[0]
            print("Got new AOA")

    def publish_aoa(self):
        pub = rospy.Publisher("aoa_topic", Float32, queue_size=10)
        rospy.init_node("aoa_processor", anonymous=True)
        rate = rospy.Rate(1) # 1 hz
        while not rospy.is_shutdown():
            rospy.loginfo("AOA move direction: " + str(self.aoa_direction))
            pub.publish(self.aoa_direction)
            rate.sleep()

if __name__=='__main__':
    obj = AOA_Processor()

    try:
        obj.publish_aoa()
    except rospy.ROSInterruptException:
        pass
