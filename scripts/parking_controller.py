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
    REVERSE_TIME_SEC = 2

    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
            self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)

        # added car length so desired distance measures from front of car
        self.parking_distance = self.CAR_LENGTH + .5 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.reverse = False
        self.time_start_reverse = 0

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

        theta = math.atan2(self.relative_y, self.relative_x)
        # L_1 = math.sqrt(self.relative_x**2 + self.relative_y**2)
        L_1 = max( (math.sqrt(self.relative_x**2 + self.relative_y**2) / 2.0), self.parking_distance)
        L = self.CAR_LENGTH
        R = L_1 / (2 * math.sin(theta))

        turn_angle = math.atan(L / R)
        drive_speed = 1.0
        at_correct_distance = abs(L_1  - self.parking_distance) < 0.08
        correct_orientation = abs(turn_angle) < 0.08 # within about 5 degrees
        now = rospy.Time.now().to_sec()
        if self.reverse:
            if now - self.time_start_reverse >= self.REVERSE_TIME_SEC:
                self.reverse = False
            else:
                drive.speed = -drive_speed 
                drive.steering_angle = 0

        if not self.reverse:
            if correct_orientation and at_correct_distance:
                drive.speed = 0
            elif self.relative_x - L < 0 or abs(turn_angle) > 0.34:
                self.time_start_reverse = rospy.Time.now().to_sec()
                drive.speed = -drive_speed 
                drive.steering_angle = 0
            elif not at_correct_distance:
                direction = 1 if L_1 > self.parking_distance else -1
                drive.speed = drive_speed * direction
                drive.steering_angle = turn_angle * direction
            elif not correct_orientation:
                # correct distance, drive backward to give space for correcting angle
                self.time_start_reverse = rospy.Time.now().to_sec()
                drive.speed = -drive_speed 
                drive.steering_angle = 0
            
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
        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = math.sqrt(self.relative_x**2 + self.relative_y**2) - self.parking_distance

        #################################
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
