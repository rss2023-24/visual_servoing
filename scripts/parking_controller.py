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

        self.parking_distance = .75 # meters; try playing with this number!
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
        print("is working")
        drive = AckermannDrive()
        # if abs(self.relative_x)> 0 and abs(self.relative_y) > 0:
        #     angle = math.atan2(max(min(-1, self.relative_x/self.relative_y), 1))
        #     if self.relative_x < 0:
        #         angle = angle*-1
        # else:
        #     angle = 0
        angle = math.atan2(self.relative_y, self.relative_x)
        print("angle:")
        print(angle)
        distance = math.sqrt(self.relative_x**2 + self.relative_y**2)
        if distance > self.parking_distance:
            if self.cur_speed == 0.0:
                self.cur_speed = 1.0
            turn_angle = -math.atan(self.CAR_LENGTH*math.sin(angle)/((distance/2)+distance*math.cos(angle)))
            acceleration = 2*(self.cur_speed**2/distance)*math.sin(angle)
            drive.acceleration = acceleration
            drive.steering_angle = -turn_angle
            drive.speed = self.cur_speed
            drive_cmd.drive = drive
        else:
            drive.speed = 0;
            drive_cmd.drive = drive


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
