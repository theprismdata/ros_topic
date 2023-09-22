#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Imu
import os
import signal
import json

transform_list=[]

def callback_imu(msg):
    lx = msg.linear_acceleration.x
    ly = msg.linear_acceleration.y
    lz = msg.linear_acceleration.z
    print("Imu lx {lx} {ly} {lz} z{o_z}".format(lx = lx, ly = ly, lz = lz,  o_z = msg.orientation.z))


def listener():
    rospy.init_node('turtle_msg_debug', anonymous=True)
    
    rospy.Subscriber("imu", Imu, callback_imu)
    rospy.spin()

if __name__ == '__main__':
    listener()


