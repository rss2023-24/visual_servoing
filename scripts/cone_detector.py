#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from ackermann_msgs.msg import AckermannDriveStamped
from visual_servoing.msg import ConeLocationPixel
from std_msgs.msg import Header

import matplotlib.pyplot as plt

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation

DARK_ORANGE = (1, 120, 0) # Make final number smaller to detect darker oranges
LIGHT_ORANGE  = (110, 255, 255) # Make first number larger to detect yellows

DARK_RED = (0, 0, 0) # Make final number smaller to detect darker red
LIGHT_RED  = (10, 255, 255) # Make first number larger to detect red

class ConeDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """

    PERCENT_TO_SHOW = 0.35
    STARTING_PERCENT_FROM_TOP = 0.5

    # PERCENT_TO_SHOW = 0.35
    # STARTING_PERCENT_FROM_TOP = 0.35

    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = True
        self.no_cone_count = 0
        self.detect = True

        # Subscribe to ZED camera RGB frames
        self.cone_pub = rospy.Publisher("/relative_cone_px", ConeLocationPixel, queue_size=10)
        self.stop_sign_pub = rospy.Publisher("/relative_stop_sign_px", ConeLocationPixel, queue_size=10)
        self.debug_pub = rospy.Publisher("/cone_debug_img", Image, queue_size=10)
        DRIVE_TOPIC = "/vesc/ackermann_cmd_mux/input/navigation"
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images



    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################
        if not self.detect: 
            return
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cropped_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        if self.LineFollower == True:
            # Add strip of input image to it
            height, width, _ = image.shape

            # Create blank image
            blank_image = np.zeros([height,width,3],dtype=np.uint8)

            start_index = int(height * self.STARTING_PERCENT_FROM_TOP)
            end_index = int(start_index + height * self.PERCENT_TO_SHOW)
            blank_image[start_index: end_index, :] = image[start_index: end_index, :]

            # Output line follower image
            cropped_image = blank_image

        # Gets center pixel on ground
        bounding_box = cd_color_segmentation(cropped_image, ".", DARK_ORANGE, LIGHT_ORANGE, False)
        bottom_center = ((bounding_box[0][0] + bounding_box[1][0]) / 2, bounding_box[1][1])
        
        # Creates message
        if bounding_box == ((0,0), (0,0)):
            rospy.loginfo("Error: Cone not detected")
            self.no_cone_count += 1
            if self.no_cone_count >= 10:
                self.detect = False
                drive_cmd = AckermannDriveStamped()
                drive_header = Header()
                drive_header.stamp = rospy.Time.now() 
                drive_header.frame_id = "base_link"
                drive_cmd.header = drive_header
                drive_cmd.drive.speed = 0
                self.drive_pub.publish(drive_cmd)
        else:
            self.no_cone_count = 0
            relative_cone_px = ConeLocationPixel()
            relative_cone_px.u = bottom_center[0]
            relative_cone_px.v = bottom_center[1]

            # Publishes point
            self.cone_pub.publish(relative_cone_px)

        # Handles detecting stop signs

         # Gets center pixel on ground
        stop_bounding_box = cd_color_segmentation(image, ".", DARK_RED, LIGHT_RED, False)
        stop_bottom_center = ((stop_bounding_box[0][0] + stop_bounding_box[1][0]) / 2, stop_bounding_box[1][1])
        
        # Creates message
        if stop_bounding_box == ((0,0), (0,0)):
            rospy.loginfo("Error: Stop sign not detected")
        else:
            relative_stop_px = ConeLocationPixel()
            relative_stop_px.u = stop_bottom_center[0]
            relative_stop_px.v = stop_bottom_center[1]

            # Publishes point
            self.stop_sign_pub.publish(relative_stop_px)


        # Debug
        cv2.rectangle(cropped_image,bounding_box[0],bounding_box[1],(0,255,0),2)
        debug_msg = self.bridge.cv2_to_imgmsg(cropped_image, "bgr8")
        self.debug_pub.publish(debug_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('ConeDetector', anonymous=True)
        ConeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
