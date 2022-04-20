#!/usr/bin/env python

# ROS-related dependencies
import rospy
from std_msgs.msg import Float64

# Other dependencies 
import serial
from serial import SerialException
import time
import array
import struct
import sys


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
                
            except SerialException:
                time.sleep(1.0)
                print('waiting...')


    def get_clean(self):
        try:
            # clear Tx and Rx buffer
            if self.ser.in_waiting != 0:
                self.ser.reset_input_buffer()
            if self.ser.out_waiting != 0:
                self.ser.reset_output_buffer()

        except SerialException:
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
                
                if type(data) == type(""):
                    val = []
                    for t in data.split():
                        try:
                            val.append(float(t))
                        except ValueError:
                            pass
                    
                    val=val[0]
                else:
                    num_elem = int(len(data)/len_var)
                    for j in range(num_elem-1):         # to account for the dummy zero at the end of trasmitted array
                        val = struct.unpack(type_var, data[j*len_var : (j+1)*len_var])
                    val = round(val[0], 3)
                
                # Print results to command window
                rospy.loginfo(val)
                
                # Write distance to class attribute
                self.dist = val
                

                
            

            # request next
            self.ser.write(trig.encode('utf-8'))
        
        except SerialException:
            self.connected=0
            self.connect_UWB()


        


def publish_dist(sensor):

    pub = rospy.Publisher('distance', Float64, queue_size=10)
    rospy.init_node('uwb', anonymous=True)

    rate = rospy.Rate(1) # 5hz
    while not rospy.is_shutdown():
        # Get measurement from sensor
        sensor.get_dist()

        # publish distance measurement
        rospy.loginfo(sensor.dist)
        pub.publish(sensor.dist)
        rate.sleep()



if __name__ == '__main__':

    # Initialize UWB sensor
    sensor=UWB_meas(my_dev='/dev/ttyACM0')
    print("Start")
    
    # Publish sensor data
    try:
        publish_dist(sensor)
    except rospy.ROSInterruptException:
        pass        
