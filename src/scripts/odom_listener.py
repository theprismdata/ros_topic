#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import os
import signal
import json

transform_list=[]

def callback_odom(data):
    odom_pose = data.pose
    
    #print(odom_pose.pose)
    odom_position = odom_pose.pose
    bf_pos_x = odom_position.position.x
    bf_pos_y = odom_position.position.y
    bf_pos_z = odom_position.position.z
    
    odom_orientation = odom_pose.pose.orientation
    bf_ori_x = odom_orientation.x
    bf_ori_y = odom_orientation.y
    bf_ori_z = odom_orientation.z
    bf_ori_w = odom_orientation.w
    
    
    print('odom Tr x:{bpx} y:{bpy} z:{bpz} Ori x:{bf_ori_x} y:{bf_ori_y} z:{bf_ori_z} w:{bf_ori_w}'.format(
                  bpx=bf_pos_x, bpy=bf_pos_y, bpz=bf_pos_z, 
                  bf_ori_x = bf_ori_x,bf_ori_y = bf_ori_y,bf_ori_z = bf_ori_z,bf_ori_w = bf_ori_w ))
    

def listener():
    rospy.init_node('turtle_msg_debug', anonymous=True)
    
    rospy.Subscriber("odom", Odometry, callback_odom)

    # rospy.Subscriber("scan", LaserScan, callback_scan)
    #rospy.Subscriber("tf", TFMessage, callback_tf_scan)
    
    rospy.spin()

if __name__ == '__main__':
    listener()

