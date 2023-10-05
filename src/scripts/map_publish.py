#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String

def callback_map_scan(map_data):
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
                        'orientation': {'ori_x':ori_x,'ori_y':ori_y, 'ori_z':ori_z}}}
    print(map_dict)
    map_pub = rospy.Publisher('map_edit', OccupancyGrid, queue_size=1)
    # map_pub = rospy.Publisher('map_edit', String, queue_size=1)

    map_msg = OccupancyGrid()
    map_msg.data = data
    try:
        map_pub.publish(map_msg)
    except Exception as e:
        print(str(e))

def listener():
    rospy.init_node('jetson_msg', anonymous=True)

    rospy.Subscriber("map", OccupancyGrid, callback_map_scan)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
    