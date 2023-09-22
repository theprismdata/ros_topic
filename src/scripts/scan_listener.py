#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import pickle
import os
import signal
scan_list = []

#move moment tracking position
def callback_odom(data):
    print(data)

def callback_tf_scan(data):
    transform = data.transforms[0].transform
    
    trans_x = transform.translation.x
    trans_y = transform.translation.y
    trans_z = transform.translation.z
    print('x:{tx}, y:{ty} , z:{tz}'.format(tx=trans_x, ty=trans_y, tz=trans_z))

def callback_map_scan(data):
    map_info = data.info
    origin = map_info.origin
    print(map_info)

def callback_scan(data):
    print(list(data.ranges)[:10])
    """
    scan_list.append(data)
    if len(scan_list) == 100:
        with open('scan_dump.pkl', 'wb') as f:
            pickle.dump(scan_list, f)
            print(scan_list)
            print('scan dump finish')
            os.kill(os.getpid(), signal.SIGKILL)
    """
def listener():
    rospy.init_node('jetson_msg_debug', anonymous=True)
    
    # rospy.Subscriber("odom", Odometry, callback_odom)
    rospy.Subscriber("scan", LaserScan, callback_scan)
    # rospy.Subscriber("tf", TFMessage, callback_tf_scan)
    #rospy.Subscriber("map", OccupancyGrid, callback_map_scan)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
