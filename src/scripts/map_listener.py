#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import os
import signal
import json

#move moment tracking position
def callback_odom(data):
    print(data)

def callback_tf_scan(data):
    transform = data.transforms[0].transform
    
    trans_x = transform.translation.x
    trans_y = transform.translation.y
    trans_z = transform.translation.z
    print('x:{tx}, y:{ty} , z:{tz}'.format(tx=trans_x, ty=trans_y, tz=trans_z))

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
    map_dict = {'info':{'position':{'pos_x':pos_x,'pos_y':pos_y,'pos_z':pos_z},
                        'orientation': {'ori_x':ori_x,'ori_y':ori_y, 'ori_z':ori_z}},
                'data':data}
    print(map_dict)
    print('----')
#    map_dict = {'info':{'orientation':map_data.info.orientation}}
#    print(map_dict)
    map_jaon = json.dumps(map_dict)
    #map_dict = {'info':map_data.info,i 'data':map_data.data}
    #print(map_dict)
    with open('map.json','w') as f:
        json.dump(map_dict, f)
    os.kill(os.getpid(), signal.SIGKILL)
   

def listener():
    rospy.init_node('jetson_msg', anonymous=True)
    
    # rospy.Subscriber("odom", Odometry, callback_odom)
    # rospy.Subscriber("scan", LaserScan, callback_scan)
    # rospy.Subscriber("tf", TFMessage, callback_tf_scan)
    rospy.Subscriber("map", OccupancyGrid, callback_map_scan)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
