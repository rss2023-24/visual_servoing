#!/usr/bin/env python

import rospy
import numpy as np

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Header
import math

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    CAR_LENGTH = 0.325

    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
            self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)

        self.parking_distance = .5 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.cur_speed = 0

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd
        # angle to the cone
        # Builds command header
        drive_header = Header()
        drive_header.stamp = rospy.Time.now() 
        drive_header.frame_id = "base_link"
        drive_cmd.header = drive_header
        drive = AckermannDrive()
        angle = math.atan2(self.relative_y, self.relative_x)
        # divided by two to handle the constant adjustment 
        distance = math.sqrt(self.relative_x**2 + self.relative_y**2)/2
        # checks if we are close enough and pointed the right direction 
        if distance > self.parking_distance:
            if distance > self.parking_distance*2 and abs(angle) < 5:
                if self.cur_speed == 0.0:
                    self.cur_speed = 1.0
            else:
                distance = distance/3
            turn_angle = -math.atan(self.CAR_LENGTH*math.sin(angle)/((distance/2)+distance*math.cos(angle)))
            acceleration = 2*(self.cur_speed**2/distance)*math.sin(angle)
            drive.acceleration = acceleration
            drive.steering_angle = -turn_angle
            drive.speed = self.cur_speed
            drive_cmd.drive = drive
        else:
            drive.speed = 0;
            drive_cmd.drive = drive
        # case 1: if the robot is far away and at wrong angle (high L1)
        # case 2: if the robot is far away and at right angle (high L1)
        # case 3: if the robot is kinda close and at right angle (Low L1)
        # case 4: if the robot is kinda close and at wrong angle (low L1)
        # case 5: if the robot is super close and at wrong andle (Low L1)

        #################################

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        #################################
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
