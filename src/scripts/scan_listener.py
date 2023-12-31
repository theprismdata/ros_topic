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
    print(data.ranges)
    print(data.intensities)
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
