#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import pickle
import os
import signal

scan_list = []


def callback_scan(data):
    print(len(data.ranges), data.ranges[100:110])
    #print(data.intensities)
    scan_m = data
    
    scan_range_list = list(scan_m.ranges)
    for si , v in enumerate(scan_range_list):
        scan_range_list[si] = v+1
    scan_m.ranges = tuple(scan_range_list)
    
    scan_m.intensities = scan_m.intensities
    pub = rospy.Publisher('scan_m', LaserScan)
    pub.publish(scan_m)
    print(scan_m.ranges[100:110])
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
    
    rospy.Subscriber("scan", LaserScan, callback_scan)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
