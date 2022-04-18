#!/usr/bin/env python

###########################################################################
### TO DO                                                               ###
### - Improve capturing of ser data, sometimes channels misclassified   ###
### - Add /dev/ACM0 as argument
###########################################################################

# ROS-related dependencies
import rospy
from std_msgs.msg import Float64MultiArray

# Other dependencies 
import serial
from serial import SerialException
import time
import array
import struct
import sys
import re

class UWB_meas:


    def __init__(self, my_dev):

        # Define initial variables
        self.my_dev = my_dev
        self.dist = []
        self.ser = []
        self.connected = 0

        # Try to connect to sensor
        self.connect_UWB()

    def connect_UWB(self): 

        # try establishing connection
        while ( self.connected != 1 ):
            try:
                self.ser = serial.Serial(self.my_dev,
                                    baudrate=115200,
                                    bytesize=serial.EIGHTBITS,
                                    stopbits=serial.STOPBITS_ONE,
                                    parity=serial.PARITY_NONE,
                                    dsrdtr=False
                                    )
                print("*** Serial device connected *** \n\n")
                self.connected = 1
                
            except SerialException as e:
                print(e)
                time.sleep(1.0)
                print('waiting...')


    def get_clean(self):
        try:
            # clear Tx and Rx buffer
            if self.ser.in_waiting != 0:
                self.ser.reset_input_buffer()
            if self.ser.out_waiting != 0:
                self.ser.reset_output_buffer()

        except SerialException as e:
            print(e)
            self.connected=0
            self.connect_UWB()
    
    def get_dist(self):
        try:

            # request and read data, data is a single number typed double
            len_var = 8
            type_var = 'd'      # int='i', double='d', float='f'
            trig = 'T'

            if self.ser.in_waiting != 0:
                data = self.ser.read(self.ser.in_waiting)
                first_val = [0.0]
                second_val = [0.0]
                for line in data.decode("utf-8").split("\r\n"):
                    m = re.match(r"d\w\w: (\d+\.\d+) d\w\w:(\d+\.\d+)", line)
                    if m is not None:
                        first_val.append(float(m[1]))
                        second_val.append(float(m[2]))
                    else:
                        continue

                first_val_avg = sum(first_val)/len(first_val)
                second_val_avg = sum(second_val)/len(second_val)
                rec_arr = [first_val_avg, second_val_avg]

                rospy.loginfo(rec_arr)
                
                # Write distance to class attribute
                self.dist = rec_arr

            # request next
            self.ser.write(trig.encode('utf-8'))
        
        except SerialException as e:
            print(e)
            self.connected=0
            self.connect_UWB()

def publish_dist(sensor):

    pub = rospy.Publisher('distance_multi', Float64MultiArray, queue_size=10)
    rospy.init_node('uwb', anonymous=True)
    my_msg = Float64MultiArray()  

    rate = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        # Get measurement from sensor
        sensor.get_dist()

        # Set up message
        my_msg.data = sensor.dist

        # publish distance measurement
        pub.publish(my_msg)
        rate.sleep()



if __name__ == '__main__':

    # Initialize UWB sensor
    sensor=UWB_meas(my_dev='/dev/ttyACM1')
    print("Start")
    
    # Publish sensor data
    try:
        publish_dist(sensor)
    except rospy.ROSInterruptException:
        pass        
