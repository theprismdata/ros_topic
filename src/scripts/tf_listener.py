#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import os
import signal
import json

transform_list=[]

def callback_tf_scan(data):
    transform_data = data.transforms[0]
    
    child_frame_id = transform_data.child_frame_id
    transform_dict = {}
    if child_frame_id == "base_imu_link":
      imu_tx = transform_data.transform.translation.x
      imu_ty = transform_data.transform.translation.y
      imu_tz = transform_data.transform.translation.z
      imu_ro_x = transform_data.transform.rotation.x
      imu_ro_y = transform_data.transform.rotation.y
      imu_ro_z = transform_data.transform.rotation.z
      
      transform_dict['base_imu_link'] ={
                                  'translation':{
                                                  'tx':imu_tx,
                                                  'ty':imu_ty,
                                                  'tz':imu_tz
                                                 },
                                  'rotation': {
                                              'rotation_x':imu_ro_x,
                                              'rotation_y':imu_ro_y, 
                                              'rotation_z':imu_ro_z
                                              }
                                  } 
      
    elif child_frame_id == "/laser_frame":
      lf_tx = transform_data.transform.translation.x
      lf_ty = transform_data.transform.translation.y
      lf_tz = transform_data.transform.translation.z
      lf_ro_x = transform_data.transform.rotation.x
      lf_ro_y = transform_data.transform.rotation.y
      lf_ro_z = transform_data.transform.rotation.z
      
      transform_dict['laser_frame'] = {
                                  'translation':{
                                                   'tx':lf_tx,
                                                   'ty':lf_ty,
                                                   'tz':lf_tz
                                                 },
                                  'rotation': {
                                              'rotation_x':lf_ro_x,
                                              'rotation_y':lf_ro_y, 
                                              'rotation_z':lf_ro_z
                                              }
                                  }
 
    elif child_frame_id == "odom":
      odom_tx = transform_data.transform.translation.x
      odom_ty = transform_data.transform.translation.y
      odom_tz = transform_data.transform.translation.z
      odom_ro_x = transform_data.transform.rotation.x
      odom_ro_y = transform_data.transform.rotation.y
      odom_ro_z = transform_data.transform.rotation.z
      
      transform_dict['odom'] = {
                                  'translation':{
                                                 'tx':odom_tx,
                                                 'ty':odom_ty,
                                                 'tz':odom_tz
                                                 },
                                  'rotation': {
                                              'rotation_x':odom_ro_x,
                                              'rotation_y':odom_ro_y, 
                                              'rotation_z':odom_ro_z
                                              }
                                  }
                                  
                                 
                                  
    print(transform_dict)

def listener():
    rospy.init_node('turtle_msg_debug', anonymous=True)
    rospy.Subscriber("tf", TFMessage, callback_tf_scan)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
