#!/usr/bin/env python3

import numpy as np
import pyspacemouse
import rospy
from std_msgs.msg import Float64, Bool, Int32
from geometry_msgs.msg import WrenchStamped, PoseStamped, Vector3
import tf2_ros
from scipy.spatial.transform import Rotation as ScipyR


class SMTeleop:
    def __init__(self):   
        self.got_robot_pose = False
        self.samprate = 10
        rospy.init_node('sm_teleop')
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.success = pyspacemouse.open()

        self.grip = False
        self.press = False

        self.teleopMode = False

        self.pose_pub = rospy.Publisher('/ur5e/compliant_pose', PoseStamped, queue_size=1)
        self.gripper_pub = rospy.Publisher('/ur5e/gripper_command', Bool, queue_size=1)
        self.smbool_pub = rospy.Publisher('/spacemouse_current_input', Bool, queue_size=1)
        rospy.Subscriber("mode", Int32, self.checkMode)
        rospy.sleep(1.0)
        self.main()

    def checkMode(self,data):
        # Mode 1 is teleoperation
        if data.data=='1':
            self.teleopMode = True
        else:
            self.teleopMode = False
            self.got_robot_pose = False

    def main(self):
        rate = rospy.Rate(self.samprate)
        while not rospy.is_shutdown():
            if self.teleopMode and not self.got_robot_pose:
                try:
                    trans = self.tfBuffer.lookup_transform('base', 'toolnew', rospy.Time())
                    curr_pos = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
                    curr_q = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])    
                    self.got_robot_pose = True
                    self.curr_pos = curr_pos.copy()
                    self.curr_q = curr_q.copy()
                except Exception as e:
                    print(e)
            
            if self.teleopMode and self.got_robot_pose:
                    for ii in range(50): # hack to force reading to be current
                        sm_state = pyspacemouse.read()
        
                    pos_scaling = 0.005
                    rot_scaling = 0.05

                    self.curr_pos[0]+=pos_scaling*sm_state.x
                    self.curr_pos[1]+=pos_scaling*sm_state.y
                    self.curr_pos[2]+=pos_scaling*sm_state.z

                    R_sm = ScipyR.from_rotvec([-0.2*rot_scaling*sm_state.pitch,0.2*rot_scaling*sm_state.roll,-rot_scaling*sm_state.yaw])
                    R_old = ScipyR.from_quat(self.curr_q)
                    R_new = (R_sm * R_old)
                    q_new = R_new.as_quat()

                    self.curr_q = q_new.copy()
                    
                    if sm_state.buttons[0]==1:
                        self.press = True
                    if self.press and sm_state.buttons[0]==0:
                        self.grip = not self.grip
                        self.press = False

                    print(self.curr_pos)
                    print(self.curr_q)
                    print(self.grip)

                    pose_out = PoseStamped()
                    pose_out.header.frame_id = 'map'
                    pose_out.header.stamp = rospy.Time.now()
                    pose_out.pose.position.x = self.curr_pos[0]
                    pose_out.pose.position.y = self.curr_pos[1]
                    pose_out.pose.position.z = self.curr_pos[2]
                    pose_out.pose.orientation.x = self.curr_q[0]
                    pose_out.pose.orientation.y = self.curr_q[1]
                    pose_out.pose.orientation.z = self.curr_q[2]
                    pose_out.pose.orientation.w = self.curr_q[3]
                    self.pose_pub.publish(pose_out)
                    self.gripper_pub.publish(Bool(self.grip))

                    rate.sleep()
            
            for ii in range(50): # hack to force reading to be current
                sm_state = pyspacemouse.read()
                curr_input = abs(sm_state.x)>0.2 or abs(sm_state.y)>0.2 or abs(sm_state.z)>0.2 or abs(sm_state.pitch)>0.2 or abs(sm_state.roll)>0.2 or abs(sm_state.yaw)>0.2 
                self.smbool_pub.publish(Bool(curr_input))

if __name__ == '__main__':
    smt = SMTeleop()
