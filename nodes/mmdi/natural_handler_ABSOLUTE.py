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
# from tf2_geometry_msgs import do_transform_pose
from scipy.spatial.transform import Rotation as ScipyR
from scipy.spatial.transform import Slerp
import serial
import signal
import time
from functools import partial
import subprocess32
import os

from scipy.optimize import minimize

import tf2_ros

def signal_handler(nh,sig, frame):
    print('You pressed Ctrl+C!')
    if nh._p is not None:
        nh._p.terminate() 
    sys.exit(0)

def camera_pose_residual(x, odom_pos,odom_q,trans_tool_cam,verbose):
    ''' Calculates a loss for camera shared control objectives '''

    pose_temp = PoseStamped()
    pose_temp.header.frame_id = 'map'
    pose_temp.header.stamp = rospy.Time.now()
    pose_temp.pose.position.x = x[0]
    pose_temp.pose.position.y = x[1]
    pose_temp.pose.position.z = x[2]

    q_tmp = ScipyR.from_rotvec(x[3:6]).as_quat()

    pose_temp.pose.orientation.x = q_tmp[0]
    pose_temp.pose.orientation.y = q_tmp[1]
    pose_temp.pose.orientation.z = q_tmp[2]
    pose_temp.pose.orientation.w = q_tmp[3]

    q_tool_cam = np.array([trans_tool_cam.transform.rotation.x, trans_tool_cam.transform.rotation.y, trans_tool_cam.transform.rotation.z, trans_tool_cam.transform.rotation.w])
    p_tool_cam = np.array([trans_tool_cam.transform.translation.x, trans_tool_cam.transform.translation.y, trans_tool_cam.transform.translation.z])

    cam_pos = ScipyR.from_quat(q_tmp).apply(p_tool_cam)+np.array([pose_temp.pose.position.x, pose_temp.pose.position.y, pose_temp.pose.position.z])
    R_tool_cam = ScipyR.from_quat(q_tool_cam)
    R_tool = ScipyR.from_quat(q_tmp)
    cam_q = (R_tool * R_tool_cam).as_quat()

    desired_dist = 0.3
    cam_to_odom = odom_pos - cam_pos
    cam_to_odom_camframe = ScipyR.from_quat(cam_q).apply(cam_to_odom)
    # if verbose:
    #     print("CAM TO ODOM CAMFRAME: ",cam_to_odom_camframe[2])
    loss0 = (cam_to_odom_camframe[2]-desired_dist)**2.0 # tracking dist in z-axis of camera frame


    vec_cam_odom = (odom_pos-cam_pos) / np.linalg.norm(odom_pos-cam_pos)
    # if verbose:
    #     print("Vec Cam Odom: ",vec_cam_odom)
    z_axis_cam = ScipyR.from_quat(cam_q).as_matrix()[:,2]
    # if verbose:
    #     print("Z-axis cam:",z_axis_cam)
    loss1 = (np.dot(vec_cam_odom,z_axis_cam)-1.0)**2.0

    # penalize moving more than rotating
    loss2 = np.linalg.norm(x[0:3])+0.1*np.linalg.norm(x[3:6]) 


    # MAYBE objectives... closer to the center of the range (maybe instead of the 30 degree)


    # 30 degree angle for viewing!
    vec_odom_cam = (cam_pos-odom_pos) / np.linalg.norm(cam_pos-odom_pos)
    z_axis_odom = ScipyR.from_quat(odom_q).as_matrix()[:,2]
    loss3 = (np.dot(vec_odom_cam,z_axis_odom)-0.7071)**2.0 # ideal value is 30 degree (cos(30)=0.866)


    desired_pos = np.array([0.0, -0.4, 0.35])
    loss4 = np.linalg.norm(np.array(x[0:3]-desired_pos))**2.0

    loss5 = np.linalg.norm(x[4])**2.0

    weights = np.array([100.0, 100.0, 0.0, 000.0, 2, 0.5])
    loss = np.array([loss0, loss1, loss2, loss3, loss4, loss5])

    if verbose:
        print(np.round(loss,4))
        # print("X:",np.round(x[0:3],3))
    
    # TODO: remove this after troubleshooting!!!!!
    # if verbose:
    #     input("WAITING FOR INPUT~~")

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
        lastbeept = time.time()
        while not rospy.is_shutdown():
            # print(self.robot_camera_active,self.robot_pose_received)
            if not self.robot_camera_active:
                try:
                    trans = self.tfBuffer.lookup_transform('head_camera', 'odom', rospy.Time())
                    now = rospy.get_rostime()
                    
                    # initial observation needs to be at least 10 cm away
                    if abs(trans.transform.translation.z)>0.10 and (now.secs - trans.header.stamp.secs)<1:
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
                        
                        self.curr_pos = curr_pos.copy()
                        self.curr_q = curr_q.copy()
                        self.starting_pos = curr_pos.copy()
                        self.starting_q = curr_q.copy()


                        self.robot_pose_received = True
                    except Exception as e:
                        print(e)
                else:
                    # camera pose optimization 
                    try:
                        trans_odom = self.tfBuffer.lookup_transform('base', 'odom', rospy.Time())
                        odom_pos  = np.array([trans_odom.transform.translation.x, trans_odom.transform.translation.y, trans_odom.transform.translation.z])
                        odom_q = np.array([trans_odom.transform.rotation.x, trans_odom.transform.rotation.y, trans_odom.transform.rotation.z, trans_odom.transform.rotation.w])    
                    except Exception as e:
                        print(e)    

                    stale = (rospy.Time.now() - trans_odom.header.stamp).to_sec()>1.0
                    print(stale)
                    if not stale:
                        new_pos, new_q = self.optimizePose(self.curr_pos, self.curr_q, odom_pos, odom_q, self.starting_pos, self.starting_q)
                        self.curr_pos = new_pos.copy()
                        self.curr_q = new_q.copy()
                    else:
                        if time.time()-lastbeept > 0.8:
                            lastbeept = time.time()
                            os.system('play -nq -t alsa synth {} sine {}'.format(0.2, 440))
                    
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


    def optimizePose(self, curr_pos, curr_q, odom_pos, odom_q, starting_pos, starting_q):

        # Set up limits (in the toolnew [command] frame)
        mins = np.array([-0.3, -0.45, 0.2, -0.45,-0.8,-1.57079])
        maxs = np.array([0.3, -0.25, 0.55, 0.0, 0.8, -1.50079])

        delta_pos_max = 0.001 # 10Hz (i.e., move 10x per second)
        delta_rot_max = 0.01 # 10Hz

        # TODO: only optimize if inside range ... otherwise, march into range...

        curr_rotvec = ScipyR.from_quat(curr_q).as_rotvec()
        pose0 = np.concatenate((curr_pos, curr_rotvec))

        # optimization can spit out just on the other side of the bound
        eps = 1e-5

        ready_to_optimize = True
        for ii in range(6):
            if pose0[ii] < (mins[ii]-eps):
                ready_to_optimize = False
                if ii<3:
                    pose0[ii] = pose0[ii]+2*delta_pos_max
                else:
                    pose0[ii] = pose0[ii]+5*delta_rot_max
                
                # possible overshoot for small ranges
                if pose0[ii] >= maxs[ii]:
                    pose0[ii] = (mins[ii]+maxs[ii])/2.0
            if pose0[ii] > (maxs[ii]+eps):
                ready_to_optimize = False
                if ii<3:
                    pose0[ii] = pose0[ii]-2*delta_pos_max
                else:
                    pose0[ii] = pose0[ii]-5*delta_rot_max
                
                # possible overshoot for small ranges
                if pose0[ii] <= mins[ii]:
                    pose0[ii] = (mins[ii]+maxs[ii])/2.0

        if ready_to_optimize:

            bounds = []

            ttmp = time.time()

            # position
            for ii in range(3):
                min_tmp = max(mins[ii],curr_pos[ii]-delta_pos_max)
                max_tmp = min(maxs[ii],curr_pos[ii]+delta_pos_max)
                bounds.append((min_tmp,max_tmp))
            
            

            # print("CURR ROTVEC: ",curr_rotvec)
            # input("HI")

            # orientation
            for ii in range(3):
                min_tmp = max(mins[ii+3],curr_rotvec[ii]-delta_rot_max)
                max_tmp = min(maxs[ii+3],curr_rotvec[ii]+delta_rot_max)
                bounds.append((min_tmp,max_tmp))

            pose0 = np.zeros((6))
            ttmp = time.time()
            res = minimize(camera_pose_residual, pose0, bounds=bounds,args=(odom_pos,odom_q,self.trans_tool_cam,False), method='SLSQP', options={'disp': False,'maxiter': 30})

            # For Troubleshooting
            ttmp = time.time()

            _ = camera_pose_residual(res.x, odom_pos, odom_q, self.trans_tool_cam, True)
            # print(res.x)
            
            new_pos = res.x[0:3].copy()
            new_rotvec = res.x[3:6]
        else:
            print("NOT READY -- ",pose0)
            new_pos = pose0[0:3]
            new_rotvec = pose0[3:6]
        
        new_q = ScipyR.from_rotvec(new_rotvec).as_quat()

        # print(new_pos,new_q)
        
        # TODO: test publish desired pose
        pose_out = PoseStamped()
        pose_out.header.frame_id = 'map'
        pose_out.header.stamp = rospy.Time.now()
        pose_out.pose.position.x = new_pos[0]
        pose_out.pose.position.y = new_pos[1]
        pose_out.pose.position.z = new_pos[2]
        pose_out.pose.orientation.x = new_q[0]
        pose_out.pose.orientation.y = new_q[1]
        pose_out.pose.orientation.z = new_q[2]
        pose_out.pose.orientation.w = new_q[3]
        self.pose_pub.publish(pose_out)

        return new_pos, new_q

if __name__ == "__main__":
    nh = NaturalHandler()