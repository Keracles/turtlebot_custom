#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge

class BoxFollower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('box_follower', anonymous=True)

        # Initialize the camera feed subscriber and the image converter
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        # Initialize the robot movement publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Define the color range for the box
        self.lower_blue = np.array([100, 50, 50])
        self.upper_blue = np.array([130, 255, 255])
        self.twist = Twist()
        self.box_center_x = None
        self.box_center_y = None
        self.box_detected = False
        self.rate = rospy.Rate(10)
        self.stop_distance = 0.515  # Distance to stop before box
        self.rotation_angle = 3.14/2  # Angle to rotate to turn around the box
        self.move_speed = 0.1  # Speed to move towards the box
        self.angular_speed = 0.3  # Speed to rotate
        self.img_width = 0
        self.img_height = 0
        self.in_range = False
        
        rospy.sleep(5)

    def image_callback(self, data):
        # Convert the image message to a CV2 image
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        self.img_width = cv_image.shape[1]
        self.img_height = cv_image.shape[0]

        # Convert the image to HSV color space
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Create a mask for the blue color range
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

        # Apply morphological transformations to the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours in the mask
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw the contours on the image
        cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 3)

        # Calculate the centroid of the largest contour
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            self.box_center_x = int(M['m10'] / M['m00']) 
            self.box_center_y = int(M['m01'] / M['m00'])
            self.box_detected = True
            
            # Draw a circle at the centroid
            cv2.circle(cv_image, (self.box_center_x, self.box_center_y), 5, (0, 0, 255), -1)
                
        # Display the image
        cv2.imshow('image', cv_image)
        cv2.waitKey(1)
        
    def scan_callback(self, msg):
        if self.in_range == True:
            # stop moving forward if we're too close to the object
            twist_msg = Twist()
            twist_msg.linear.x = 0.0  # Stop moving forward
            twist_msg.angular.z = self.rotation_angle  # Start turning
            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(1.0)  # Turn for 1 second
            
            twist_msg.angular.z = 0.0  # Stop turning
            twist_msg.linear.x = self.move_speed  # Start moving forward
            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(self.stop_distance / twist_msg.linear.x)  # Move forward for distance
            
            twist_msg.linear.x = 0.0  # Stop moving forward
            twist_msg.angular.z = -self.rotation_angle  # Start turning in the opposite side
            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(1.0)  # Turn for 1 second
            
            twist_msg.angular.z = 0.0  # Stop turning
            twist_msg.linear.x = self.move_speed  # Start moving forward
            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(self.stop_distance / twist_msg.linear.x)  # Move forward for distance
            
            twist_msg.linear.x = 0.0  # Stop moving forward
            twist_msg.angular.z = - self.rotation_angle  # Start turning in the opposite side
            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(1.0)  # Turn for 1 second
            
            rospy.loginfo('Arrived at adjacent side')
            
        else:
            # get the index of the laser scan range value that corresponds to the center of the field of view
            center_index = int(len(msg.ranges) / 2)
            
            # get the range value at the center index
            center_range = msg.ranges[center_index]
            
            print(center_range)
            
            # check if we have a valid range reading
            if center_range > 0 and center_range < self.stop_distance:        
                self.in_range = True
            else:
                if self.box_center_x:
                    # calculate the error between the current center x coordinate and the desired center x coordinate
                    error = self.box_center_x - self.img_width / 2
                    
                    # create a Twist message with the appropriate linear and angular velocities
                    twist_msg = Twist()
                    twist_msg.linear.x = self.move_speed
                    twist_msg.angular.z = -float(error) / 100
                    
                    # publish the Twist message to the cmd_vel topic
                    self.cmd_vel_pub.publish(twist_msg)
    	
   	

    def run(self):
        while not rospy.is_shutdown():
            # wait for a valid center x coordinate to be set
            while self.box_center_x is None and not rospy.is_shutdown():
                rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        tracker = BoxFollower()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
