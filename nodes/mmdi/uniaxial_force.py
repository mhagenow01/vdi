#!/usr/bin/env python3

""" Process the raw force readings from the uniaxial force sensor
 Last Updated: 6/7/2024
"""

__author__ = "Mike Hagenow"

from json import tool
import numpy as np
import rospy
import sys
from std_msgs.msg import Int32, String, Float32
from scipy.spatial.transform import Rotation as ScipyR
import serial
import signal
import time
from functools import partial

import tf2_ros


def signal_handler(valve, sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

class UniaxialForceProcessor():
    def __init__(self, simulated=False):
        rospy.init_node('uniaxial_force_processor', anonymous=True)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.tool_N = 2.23 # 4.46 N / 2 (when flipping tool upside down)
        # Note: this is only for the rolling pin tool

        self.p_tool = None
        self.q_tool = None
        
        self.biased = False
        self.bias = 0.0

        rospy.Subscriber("/uniforce/raw", Float32, self.save_raw_force)
        self.uniforce_pub = rospy.Publisher("/uniforce/force", Float32, queue_size=1)

        self.forceLoop()

    def save_raw_force(self,data):
        grams_tmp = data.data
        self.F_tmp = (grams_tmp/1000.0) * 9.81 # kg * m/s^2 -> N

    def forceLoop(self):
        rate = rospy.Rate(100) # Hz

        while not rospy.is_shutdown():

            # Try to fetch current commanded EE location from TF
            try:
                trans = self.tfBuffer.lookup_transform('base', 'toolnew', rospy.Time())
                self.tool_q = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            except Exception as e:
                pass
            try:
                # TODO: is the tool weight direction correct (test by flipping)
                if not self.biased and self.q_tool is not None:
                    toolN_local = ScipyR.from_quat(self.tool_q).inv().apply(np.array([0.0, 0.0, self.tool_N]))[2]
                    self.bias = self.F_tmp-toolN_local
                    self.biased = True

                if self.biased:
                    toolN_local = ScipyR.from_quat(self.tool_q).inv().apply(np.array([0.0, 0.0, self.tool_N]))[2]
                    self.uniforce_pub.publish(Float32(self.F_tmp-self.bias-toolN_local))
            except Exception as e:
                print(e)

            rate.sleep()

   
if __name__ == "__main__":
    ufp = UniaxialForceProcessor()