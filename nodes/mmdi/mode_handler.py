#!/usr/bin/env python3

""" Mode (state machine) handling and LED publishing
 Last Updated: 6/11/2024
"""

__author__ = "Mike Hagenow"

import numpy as np
import rospy
import sys
from std_msgs.msg import String, Int32, Bool, Float32
from geometry_msgs.msg import WrenchStamped
import tf2_ros
from scipy.spatial.transform import Rotation as ScipyR

import os

import serial
import signal
import time
from functools import partial


def signal_handler(valve, sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

##################################
# MODE DESCRIPTION               #
# -1: initialization             #
# 0: no input (red)              #
# 1: teleop (green)              #
# 2: kinesthetic (yellow)        #
# 3: natural (blue)              #
# 4: pre_natural (strobe blue)   #
##################################


class ModeHandler():
    def __init__(self, simulated=False):
        rospy.init_node('mode_handler', anonymous=True)
       
        # Initial Assumed State
        self.toolContact = 1

        self.tool_q = None

        self.currSM = False
        self.uniforce = None
        self.uniforcetime = None
        self.odomSeen = False
        
        self.discrepancy_samps = 0

        self.led_pub = rospy.Publisher("led_state", String, queue_size=1)
        self.mode_pub = rospy.Publisher("/mode", Int32, queue_size=1)
        self.freedrive_pub = rospy.Publisher("/ur5e/free_drive", Bool, queue_size=1)
        rospy.Subscriber("tool_contact", Int32, self.storeToolContact)
        rospy.Subscriber("spacemouse_current_input", Bool, self.storeCurrSM)
        rospy.Subscriber("distance_odom_seen", Bool, self.storeOdomSeen)
        rospy.Subscriber("/uniforce/force", Float32, self.storeUniForce)
        rospy.Subscriber("/wrench_global", WrenchStamped, self.storeWrenchGlobal)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.startmodezero = time.time()
        self.lastNoContact = None

        rospy.sleep(2)

        
        self.mode = -1 # non value to start
        self.mode_colors = ['r','g','y','b','s']
       
        signal.signal(signal.SIGINT, partial(signal_handler, self))        

        self.modeProcessor()

    def storeCurrSM(self,data):
        self.currSM = data.data

    def storeUniForce(self,data):
        self.uniforce = data.data
        self.uniforcetime = rospy.Time.now()
    
    def storeWrenchGlobal(self,data):
        if self.uniforcetime is not None: # only store samples that are coordinated with the lower rate uniforce time
            if (rospy.Time.now()-self.uniforcetime).to_sec() < 0.01:
                self.wrenchglobal = data.wrench

    def storeToolContact(self,data):
        self.toolContact = data.data

    def storeOdomSeen(self,data):
        self.odomSeen = data.data

    def forceDiscrepancy(self):
        # TODO: needs to be tested
        if self.tool_q is not None and self.uniforce is not None and self.wrenchglobal is not None:
            R_tool_global = ScipyR.from_quat(self.tool_q)
            F_global = np.array([self.wrenchglobal.force.x, self.wrenchglobal.force.y, self.wrenchglobal.force.z])
            F_local = R_tool_global.inv().apply(F_global)

            # print("   UF:",self.uniforce)
            # print("   F_ft:",F_local[2])

            # use sample counting to get around filter induced debouncing
            if (F_local[2]-self.uniforce)>8: # only when pulling down
                self.discrepancy_samps += 1
            else:
                self.discrepancy_samps = 0 

            if self.discrepancy_samps >= 3: #3/5 seconds
                return True
            else:
                return False

        # if no tool_q transform, return false
        return False


    def modeProcessor(self):
        rate = rospy.Rate(5) # Hz
        while not rospy.is_shutdown():

            # Check tool location
            try:
                trans = self.tfBuffer.lookup_transform('base', 'toolnew', rospy.Time())
                self.tool_q = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])

            except Exception as e:
                print(e)

            ####################
            # Mode Transitions #
            ####################
            new_mode = self.mode

            print("MODE ",self.mode)

            if self.mode==-1: 
                # valid (and assumed) tx is 0
                new_mode = 0 # start with no input

            if self.mode==0:
                # valid tx is to 1,2, and 4

                if (time.time() - self.startmodezero)>2.0: # deboucing
                    
                    # Switch to Teleoperation (1)
                    if self.currSM:
                        new_mode = 1

                    # Switch to Kinesthetic (2)
                    if self.forceDiscrepancy():
                        new_mode = 2
                        self.freedrive_pub.publish(Bool(True))
                    
                    # 4 Switch to (pre) Natural (4)
                    if self.toolContact==0:
                        if self.lastNoContact is None:
                            self.lastNoContact = time.time()
                        if (time.time()-self.lastNoContact) > 1:
                            new_mode = 4
                            self.startmodefour = time.time()
                    else:
                        self.lastNoContact = None

            if self.mode==1: # TELEOPERATION
                # valid tx is to 2 and 4

                # change color based on force (intensity of color)
                F_global = np.array([self.wrenchglobal.force.x, self.wrenchglobal.force.y, self.wrenchglobal.force.z])
                if np.linalg.norm(F_global) > 10:
                        self.led_pub.publish(String(str(int((25-np.linalg.norm(F_global))/3))))
                else:
                    self.led_pub.publish(String(self.mode_colors[self.mode]))


                # play sound for force sensor
                if abs(self.uniforce)>0.5*9.8:
                    print("   PLAYING: ",66*abs(self.uniforce)-2)
                    os.system('play -nq -t alsa synth {} sine {}'.format(0.2, 66*(abs(self.uniforce)-0.5*9.8)))
                

                # Switch to Kinesthetic (2)
                if self.forceDiscrepancy():
                    new_mode = 2
                    self.freedrive_pub.publish(Bool(True))

                # too much applied force also switches to kinesthetic
                
                if(abs(self.uniforce>25) or np.linalg.norm(F_global)>25):
                    new_mode = 2
                    self.freedrive_pub.publish(Bool(True))

                 # 4 Switch to (pre) Natural (4)
                if self.toolContact==0:
                    if self.lastNoContact is None:
                        self.lastNoContact = time.time()
                    if (time.time()-self.lastNoContact) > 1:
                        new_mode = 4
                        self.startmodefour = time.time()
                else:
                    self.lastNoContact = None

            if self.mode==2:
                # valid tx is to 1

                # Switch to Teleoperation (1)
                if self.currSM: # TODO: maybe also some force discrepancy
                    new_mode = 1
                    self.freedrive_pub.publish(Bool(False))

                # 4 Switch to (pre) Natural (4)
                if self.lastNoContact is not None:
                    print("TOOL CONTACT:",self.toolContact," ",time.time()-self.lastNoContact)
                else:
                    print("TOOL CONTACT:",self.toolContact,"  NO TIME")
                if self.toolContact==0:
                    if self.lastNoContact is None:
                        self.lastNoContact = time.time()
                    if (time.time()-self.lastNoContact) > 1:
                        new_mode = 4
                        self.freedrive_pub.publish(Bool(False))
                        self.startmodefour = time.time()
                else:
                    self.lastNoContact = None



            if self.mode==3:
                # valid tx is to 0
                # TODO: think about how to actually do this since camera will be moving
    
                # Switch to Nothing (0)
                if self.toolContact==1:
                    new_mode = 0
                    self.startmodezero = time.time()

            if self.mode==4:
                # valid tx is to 3 or 0
                if self.odomSeen:
                    new_mode = 3

                # Switch to Nothing (0)
                if self.toolContact==1 and (time.time()-self.startmodefour)>5:
                    new_mode = 0
                    self.startmodezero = time.time()
                


            # Set mode and command LEDs
            if new_mode!=self.mode:
                self.mode = new_mode
                self.led_pub.publish(String(self.mode_colors[self.mode]))
            self.mode_pub.publish(Int32(self.mode))
            rate.sleep()
    

# TODO: based on mode, publish final recordable topics!!!


if __name__ == "__main__":
    mh = ModeHandler()