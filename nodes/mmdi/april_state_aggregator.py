#!/usr/bin/env python3

""" Converts april tag poses from TF into estimates of tool pose for use in Kalman filter
 Last Updated: 6/10/2024
"""

__author__ = "Mike Hagenow"

import numpy as np
import rospy
import sys
from std_msgs.msg import Int32, String, Float32
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import serial
import signal
import time
from functools import partial

from scipy.spatial.transform import Rotation as ScipyR

import tf2_ros

# TODO: attach signal handler
def signal_handler(valve, sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

class AprilStateAggregator():
    def __init__(self):
        rospy.init_node('april_state_aggregator', anonymous=True)

        self.tags = ['tag_25','tag_20','tag_27','tag_21','tag_26','tag_22','tag_23','tag_24']

        rospy.Subscriber("/tf", TFMessage, self.checkTF)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        self.seq = 0

        self.toolposepub = rospy.Publisher("/vo", Odometry, queue_size=1)

        rospy.spin()
      
    def checkTF(self,data):
        ###############################################
        # Check for tags and if so, compute transform #
        ###############################################

        # TODO: NEED TO REVISIT THE TRANSFORMATIONS -- THEY DO NOT SEEM CORRECT

        for ii in range(len(data.transforms)):
            if data.transforms[ii].child_frame_id in self.tags:

                # look up transformed to tool location
                try:
                    trans = self.tfBuffer.lookup_transform('base', 'tool_'+data.transforms[ii].child_frame_id, rospy.Time()) # TODO: move to actual transform
                
                    ###############################################
                    # Publish as odometry (for robot_pose_ekf   ) #
                    ###############################################

                    # note: packages doesn't use velocity in odometry msg
                    od_tmp = Odometry()
                    od_tmp.header.seq = self.seq
                    od_tmp.header.stamp = rospy.Time.now()
                    od_tmp.header.frame_id = 'base'
                    self.seq+=1

                    od_tmp.child_frame_id = 'tool'

                    # covariance tuning for observations

                    # TODO: can try setting covariance based on an estimate of how well the april tag is imaged! :)
                    od_tmp.pose.covariance = (0.1*np.eye(6)).astype(float).ravel()
                    
                    od_tmp.pose.pose.position.x = trans.transform.translation.x
                    od_tmp.pose.pose.position.y = trans.transform.translation.y
                    od_tmp.pose.pose.position.z = trans.transform.translation.z
                    
                    od_tmp.pose.pose.orientation.x = trans.transform.rotation.x
                    od_tmp.pose.pose.orientation.y = trans.transform.rotation.y
                    od_tmp.pose.pose.orientation.z = trans.transform.rotation.z
                    od_tmp.pose.pose.orientation.w = trans.transform.rotation.w

                    self.toolposepub.publish(od_tmp)
                except Exception as e:
                    print(e)

                
               
   
if __name__ == "__main__":
    asa = AprilStateAggregator()