#!/usr/bin/env python3

""" Handles natural demonstrations (state filtering and camera optimization)
 Last Updated: 6/11/2024
"""

__author__ = "Mike Hagenow"

import numpy as np
import rospy
import sys
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as ScipyR
import serial
import signal
import time
from functools import partial
import subprocess32

from scipy.optimize import minimize

import tf2_ros

def signal_handler(nh,sig, frame):
    print('You pressed Ctrl+C!')
    if nh._p is not None:
        nh._p.terminate() 
    sys.exit(0)

def camera_pose_residual(x,trans_tool,curr_pos, curr_rotvec, odom_pos,odom_q,trans_tool_cam):
    ''' Calculates a loss for camera shared control objectives '''

    pose_temp = PoseStamped()
    pose_temp.header.frame_id = 'map'
    pose_temp.header.stamp = rospy.Time.now()
    pose_temp.pose.position.x = curr_pos[0]+x[0]
    pose_temp.pose.position.y = curr_pos[1]+x[1]
    pose_temp.pose.position.z = curr_pos[2]+x[2]

    rotvec_temp = curr_rotvec + x[3:6]
    q_tmp = ScipyR.from_rotvec(rotvec_temp).as_quat()

    pose_temp.pose.orientation.x = q_tmp[0]
    pose_temp.pose.orientation.y = q_tmp[1]
    pose_temp.pose.orientation.z = q_tmp[2]
    pose_temp.pose.orientation.w = q_tmp[3]

    cam_proposed_tx = tf2_geometry_msgs.do_transform_pose(pose_temp, trans_tool_cam)

    cam_pos = np.array([cam_proposed_tx.transform.translation.x, cam_proposed_tx.transform.translation.y, cam_proposed_tx.transform.translation.z])
    cam_q = np.array([cam_proposed_tx.transform.rotation.x, cam_proposed_tx.transform.rotation.y, cam_proposed_tx.transform.rotation.z, cam_proposed_tx.transform.rotation.w])
    
    desired_dist = 0.2
    cam_to_odom = odom_pos - cam_pos
    cam_to_odom_camframe = ScipyR.from_quat(cam_q).apply(cam_to_odom)
    loss0 = (abs(cam_to_odom_camframe[2])-desired_dist)**2.0 # tracking dist in z-axis of camera frame

    vec_cam_odom = (odom_pos-cam_pos) / np.linalg.norm(odom_pos-cam_pos)
    z_axis_cam = ScipyR.from_quat(q_tmp).as_matrix()[:,2]
    loss1 = np.dot(vec_cam_odom,z_axis_cam)**2.0

    # TODO: add a loss away from edges of space

    weights = np.array([1.0, 1.0])
    loss = np.array([loss0, loss1])
    return np.dot(weights,loss)


    
    

def quat_con(x):
    ''' quaternion must have magnitude 1 -- used in NL optimization '''
    return 1.0-np.linalg.norm(x[0:4])

class NaturalHandler():
    def __init__(self):
        rospy.init_node('natural_handler', anonymous=True)
       
        rospy.Subscriber("mode", Int32, self.checkMode)

        self.odom_seen_pub = rospy.Publisher('/distance_odom_seen', Bool, queue_size=1)
        self.pose_pub = rospy.Publisher('/ur5e/compliant_pose', PoseStamped, queue_size=1)


        self.natural_mode = False
        self.robot_camera_active = False
        self.robot_pose_received = False

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self._p = None

        rospy.sleep(2)
       
        signal.signal(signal.SIGINT, partial(signal_handler, self))        

        self.main()


    def main(self):
        rate = rospy.Rate(10) # hz
        while not rospy.is_shutdown():
            if not self.robot_camera_active:
                try:
                    trans = self.tfBuffer.lookup_transform('head_camera', 'odom', rospy.Time())
                    now = rospy.get_rostime()
                    
                    if abs(trans.transform.translation.z)>0.25 and (now.secs - trans.header.stamp.secs)<1:
                        self.odom_seen_pub.publish(Bool(True))
                except Exception as e:
                    pass
            else:
                if not self.robot_pose_received:
                    try:

                        self.trans_tool_cam = self.tfBuffer.lookup_transform('toolnew', 'head_camera', rospy.Time())                     

                        trans = self.tfBuffer.lookup_transform('base', 'toolnew', rospy.Time())
                        curr_pos = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
                        curr_q = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])                           
                        
                        self.got_robot_pose = True
                        self.curr_pos = curr_pos.copy()
                        self.curr_q = curr_q.copy()
                    except Exception as e:
                        print(e)
                else:
                    # camera pose optimization 
                    try:
                        trans_tool = self.tfBuffer.lookup_transform('base', 'toolnew', rospy.Time())
                        trans_odom = self.tfBuffer.lookup_transform('base', 'odom', rospy.Time())
                        odom_pos  = np.array([trans_odom.transform.translation.x, trans_odom.transform.translation.y, trans_odom.transform.translation.z])
                        odom_q = np.array([trans_odom.transform.rotation.x, trans_odom.transform.rotation.y, trans_odom.transform.rotation.z, trans_odom.transform.rotation.w])    
                        self.optimizePose(trans_tool, odom_pos, odom_q)
                    
                    except Exception as e:
                        print(e)    
                    pass
            rate.sleep()
        if self._p is not None:
            self._p.terminate() 


    def checkMode(self,data):
        if data.data == 4: # ready to start shared camera control
            if not self.natural_mode:
                if self._p is not None:
                    self._p.terminate() 
                self._p = subprocess32.Popen(["roslaunch", "mmdi", "mmdi_state.launch"])
                self.natural_mode = True
        elif data.data == 3: # already started shared camera control
            self.robot_camera_active = True
        else: # other mode
            self.natural_mode = False
            self.robot_camera_active = False
            self.robot_pose_received = False
            self.odom_seen_pub.publish(Bool(False))


    def optimizePose(self, trans_tool, odom_pos, odom_q):

        # Set up limits (in the toolnew [command] frame)
        cart_mins = np.array([0.0, 0.0, 0.1])
        cart_maxs = np.array([0.8, 0.8, 1.2])

        delta_pos_max = 0.01 # 10Hz (i.e., move 10x per second)
        delta_rot_max = 0.02 # 10Hz

        curr_pos = np.array([trans_tool.transform.translation.x, trans_tool.transform.translation.y, trans_tool.transform.translation.z])
        curr_q = np.array([trans_tool.transform.rotation.x, trans_tool.transform.rotation.y, trans_tool.transform.rotation.z, trans_tool.transform.rotation.w])

        curr_rotvec = ScipyR.from_quat(curr_q).as_rotvec()

        ang_max = 0.6

        bounds = [(-delta_pos_max,delta_pos_max),(-delta_pos_max,delta_pos_max),(-delta_pos_max,delta_pos_max),(-delta_rot_max,delta_rot_max),(-delta_rot_max,delta_rot_max),(-delta_rot_max,delta_rot_max)]

        # TODO: update bounds based on constraints?

        pose0 = np.zeros((6))
        res = minimize(camera_pose_residual, pose0, bounds=bounds,args=(trans_tool, curr_pos, curr_rotvec, odom_pos,odom_q,self.trans_tool_cam), method='SLSQP', options={'disp': False,'maxiter': 50})

        print(res)
         
        # Saturation TODO: make sure this works :)
        new_pos = curr_pos+res.x[0:3]
        new_pos = np.minimum(cart_maxs,new_pos)
        new_pos = np.maximum(cart_mins,new_pos)
        new_ang = curr_rotvec + res.x[3:6]
        if np.linalg.norm(new_ang) > ang_max:
            new_ang = ang_max * (new_ang / np.linalg.norm(new_ang))
        
        new_q = ScipyR.from_rotvec(new_ang).as_quat()
        
        # TODO: test publish desired pose
        # pose_out = PoseStamped()
        # pose_out.header.frame_id = 'map'
        # pose_out.header.stamp = rospy.Time.now()
        # pose_out.pose.position.x = new_pos[0]
        # pose_out.pose.position.y = new_pos[1]
        # pose_out.pose.position.z = new_pos[2]
        # pose_out.pose.orientation.x = new_q[0]
        # pose_out.pose.orientation.y = new_q[1]
        # pose_out.pose.orientation.z = new_q[2]
        # pose_out.pose.orientation.w = new_q[3]
        # self.posepub.publish(pose_out)



if __name__ == "__main__":
    nh = NaturalHandler()