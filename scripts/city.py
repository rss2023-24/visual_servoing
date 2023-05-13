#!/usr/bin/env python2
from __future__ import division

import numpy as np

import rospy
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDriveStamped
from city_driving.msg import ConeLocation

class CityController:
    SAFETY_DRIVE_TOPIC = "/vesc/low_level/ackermann_cmd_mux/input/safety"

    def __init__(self):
        self.stop_duration = 3
        self.stop_time = None # time of first drive(0) command
        self.stop_dist = 1 # distance at which to begin stopping (actual stop must be 0.75-1.0m away)
        self.prev_stop_dist = None # most recent distance to a stop sign

        # Initializes subscribers
        rospy.Subscriber('/relative_stop_sign', ConeLocation, self.stop_sign_callback)

        # Initializes publishers
        self.stop_pub = rospy.Publisher(self.SAFETY_DRIVE_TOPIC, AckermannDriveStamped) 

    def stop_sign_callback(self, msg):
        dist = np.sqrt(msg.x ** 2 + msg.y ** 2)
        print(f'stop sign distance: {dist}')

        # consider new stop if it's relative distance is further than the most recent stop sign
        new_stop = (self.prev_stop_dist is None) or (self.prev_stop_dist < dist)
        self.prev_stop_dist = dist
        if not new_stop:
            return
        
        if dist <= self.stop_dist:
            current_time = rospy.Time.now().to_sec()
            if self.stop_time is None:
                self.stop_time = current_time
            if current_time - self.stop_time < self.stop_duration:
                drive_cmd = AckermannDriveStamped()
                drive_header = Header()
                drive_header.stamp = rospy.Time.now() 
                drive_header.frame_id = "base_link"
                drive_cmd.header = drive_header
                drive_cmd.drive.speed = 0
                self.stop_pub.publish(drive_cmd)
            else:
                self.stop_time = None


if __name__ == "__main__":
    rospy.init_node('city')
    city_controller = CityController()
    rospy.spin()
