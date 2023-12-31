#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import os
import signal
import json

def callback_map_scan(map_data):
    """
    map_info = data.info
    origin = map_info.origin
    print(map_info)
    """
    pose = map_data.info.origin
    position = pose.position
    pos_x = position.x
    pos_y = position.y
    pos_z = position.z

    orientation = pose.orientation
    ori_x = orientation.x
    ori_y = orientation.y
    ori_z = orientation.z
    data = map_data.data
    # map_dict = {'info':{'position':{'pos_x':pos_x,'pos_y':pos_y,'pos_z':pos_z},
    #                     'orientation': {'ori_x':ori_x,'ori_y':ori_y, 'ori_z':ori_z}},
    #             'data':data}
    map_dict = {'info':{'position':{'pos_x':pos_x,'pos_y':pos_y,'pos_z':pos_z},
                        'orientation': {'ori_x':ori_x,'ori_y':ori_y, 'ori_z':ori_z}}}
    print(map_dict)
   

def listener():
    rospy.init_node('jetson_msg', anonymous=True)
    rospy.Subscriber("map", OccupancyGrid, callback_map_scan)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
