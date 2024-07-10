#!/usr/bin/env python3

""" Gets and sends information to arduino (LED, contact, force)
 Last Updated: 6/7/2024
"""

__author__ = "Mike Hagenow"

import numpy as np
import rospy
import sys
from std_msgs.msg import Int32, String, Float32
import serial
import signal
import time
from functools import partial


def signal_handler(ad, sig, frame):
    print('You pressed Ctrl+C!')
    ad.ser.close()
    sys.exit(0)

class ArduinoHandler():
    def __init__(self, simulated=False):
        rospy.init_node('arduino_handler', anonymous=True)
       
        
        self.ser = serial.Serial('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0', 9600, timeout=0.5)

        time.sleep(2) # wait 2 seconds to allow port to open

        rospy.Subscriber("led_state", String, self.led_callback)
        self.uniforce_raw_pub = rospy.Publisher("/uniforce/raw", Float32, queue_size=1)
        self.contact_pub = rospy.Publisher("/tool_contact", Int32, queue_size=1)


        signal.signal(signal.SIGINT, partial(signal_handler, self))        

        self.curr_val = "e"
        self.led_callback(String(self.curr_val)) # send an initial value  
            
        self.readContactAndForce()


    def readContactAndForce(self):
        rate = rospy.Rate(100) # 100 hz as fall back though should be much slower
        while not rospy.is_shutdown():
            try:
                tmp =str(self.ser.readline())
                force = float(tmp.split("'")[1].split(",")[0])
                contact = int(tmp.split(",")[1][0])

                print(force,contact)
        
                self.uniforce_raw_pub.publish(Float32(force))
                self.contact_pub.publish(Int32(contact))
            except:
               print("Error reading force/contact from arduino")
            rate.sleep()


    def led_callback(self,data):
        # send to Arduino
        if data.data != self.curr_val:
        
            serialsent = False
            ii = 0
            while not serialsent and ii < 10:
                try:
                    print("sent",str(data.data))
                    if data.data=="r":
                        self.ser.write(bytearray('r', 'ascii'))
                    elif data.data=="g":
                        self.ser.write(bytearray('g', 'ascii'))
                    elif data.data=="b":
                        self.ser.write(bytearray('b', 'ascii'))
                    elif data.data=="s":
                        self.ser.write(bytearray('s', 'ascii'))
                    elif data.data=="y":
                        self.ser.write(bytearray('y', 'ascii'))
                    elif data.data=="y":
                        self.ser.write(bytearray('y', 'ascii'))
                    elif data.data=="5":
                        self.ser.write(bytearray('5', 'ascii'))
                    elif data.data=="4":
                        self.ser.write(bytearray('4', 'ascii'))
                    elif data.data=="3":
                        self.ser.write(bytearray('3', 'ascii'))
                    elif data.data=="2":
                        self.ser.write(bytearray('2', 'ascii'))
                    elif data.data=="1":
                        self.ser.write(bytearray('1', 'ascii'))
                    elif data.data=="0":
                        self.ser.write(bytearray('0', 'ascii'))
                    elif data.data=="e":
                        self.ser.write(bytearray('e', 'ascii'))
                    # else:
                    #     self.ser.write(bytearray('e', 'ascii'))
                    serialsent = True
                    self.curr_val = data.data
                except:
                    ii += 1
                    print("Failed Serial: ",ii)
                    rospy.sleep(0.1)
    


if __name__ == "__main__":
    ah = ArduinoHandler()