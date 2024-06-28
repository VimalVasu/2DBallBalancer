#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
import time
import math
import numpy as np
    
class MotorSpeedPublisher:
    def __init__(self):
        # Initialize global variables
        self.R = 3.5
        self.kpx = self.kpy = 1.6
        self.kix = self.kiy = 0.0
        self.kdx = self.kdy = 0.6
        self.lastTime = 0
        self.lastErrorX = 0
        self.lastErrorY = 0
        self.sumErrorX = 0
        self.sumErrorY = 0
    
        # Initialize ROS node
        rospy.init_node('ball_position_listener', anonymous=True)
     
        # Create a publisher for the 'motor_speeds' topic
        self.motor_speeds_publisher = rospy.Publisher('motor_speeds',String, queue_size=10)
    
        # Subscribe to the 'green_ball_coords' topic
        rospy.Subscriber('green_ball_coords', Point, self.callback)
    
    def millis(self):
        return int(time.time() * 1000)


    def callback(self, data):
        ballPosX = data.x
        ballPosY = data.y
    
        # Initialize local variables
        
    
        # Perform some math on x and y
        now = self.millis()
        timeChange = now - self.lastTime
    
        derivativeX = (ballPosX - self.lastErrorX)/timeChange
        derivativeY = (ballPosY - self.lastErrorY)/timeChange

        self.sumErrorX += self.lastErrorX
        self.sumErrorY += self.lastErrorY
  
        thetax = self.kpx * ballPosX + self.kdx * derivativeX + self.kix * self.sumErrorX
        thetay = self.kpy * ballPosY + self.kdy * derivativeY + self.kiy * self.sumErrorY
  
        spdA = -int((0.5*thetax - (math.sqrt(3)/2)*thetay) * self.R)
        spdB = -int((0.5*thetax + (math.sqrt(3)/2)*thetay) * self.R)
        spdC = -int(-thetax * self.R)
        
        spdA = spdA - (spdA % 10)
        spdB = spdB - (spdB % 10)
        spdC = spdC - (spdC % 10)
    	
        spdA = max(-1000, min(1000, spdA))
        spdB = max(-1000, min(1000, spdB))
        spdC = max(-1000, min(1000, spdC))
  
        spdA_msg = f"spdA = {spdA}"
        spdB_msg = f"spdB = {spdB}"
        spdC_msg = f"spdC = {spdC}"

        self.lastErrorX = ballPosX
        self.lastErrorY = ballPosY

        self.lastTime = now
        
        # Publish the results to the 'motor_speeds' topic
        self.motor_speeds_publisher.publish(spdA_msg)
        self.motor_speeds_publisher.publish(spdB_msg)
        self.motor_speeds_publisher.publish(spdC_msg)
        self.motor_speeds_publisher.publish("--------------------------------")

        
    def run(self):
        rospy.spin()
        
        
if __name__ == '__main__':
    # Create a publisher for the 'motor_speeds' topic
    
    try:
        controller = MotorSpeedPublisher()
        controller.run()
    except rospy.ROSInterruptException:
        pass

