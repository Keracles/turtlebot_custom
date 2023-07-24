#!/usr/bin/env python3
#----------------------------------------------------------------------------
# Created By  : Khéo Albarede-Barte
# Created Date: 20.05.2023
# CS470 Introduction to Artificial Intelligence: final project
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------
import rospy
import cv2
import numpy as np
import tf

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from math import atan2, pi


class BoxFollower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('box_follower', anonymous=True)

        # Sleep for 3 seconds
        rospy.sleep(3)

        # Initialize the camera feed subscriber and the image converter
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Initialize the robot movement publisher
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Define the color range for the box
        self.lower_pink = np.array([150, 50, 50])
        self.upper_pink = np.array([170, 255, 255])

        # Flag
        self.box_attached = False

        # Goal
        self.x_goal = -2
        self.y_goal = -0.5
        self.current_x = 0
        self.current_y = 0

        # Start the main loop
        self.loop()

    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # Convert the image to HSV color space
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Create a mask for the blue color range
        mask = cv2.inRange(hsv, self.lower_pink, self.upper_pink)

        # Apply morphological transformations to the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours in the mask
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw the contours on the image
        cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 3)

        err_x = 0

        # Calculate the centroid of the largest contour
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # Draw a circle at the centroid
            cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)

            # Move the robot towards the centroid
            err_x = cx - cv_image.shape[1] / 2
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = -float(err_x) / 100
            rospy.loginfo(twist)
            self.vel_pub.publish(twist)
        else:  # When the robot is close enough to the box
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = -float(err_x) / 100
            rospy.loginfo(twist)
            self.vel_pub.publish(twist)
            rospy.sleep(1.0)
            self.box_attached = True
            rospy.loginfo("Box attached")
            self.image_sub.unregister()  # Unsubscribe from the image topic
            self.move_to_target()

        # Display the image
        cv2.imshow('image', cv_image)
        cv2.waitKey(1)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_orientation = msg.pose.pose.orientation
        print(self.current_x)
        print(self.current_y)

    def move_to_target(self):
        print("move target")
        # Set the linear and angular velocities
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2  # Linear velocity in m/s
        cmd_vel.angular.z = 0.0  # Angular velocity in rad/s

        while not rospy.is_shutdown():
            # Calculate the distance to the target
            distance = ((self.x_goal - self.current_x) ** 2 + (self.y_goal - self.current_y) ** 2) ** 0.5

            if distance < 0.1:  # If robot is close to the target
                # Stop the robot
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.vel_pub.publish(cmd_vel)
                rospy.loginfo("Robot reached the target!")
                break

             # Calculate the required angle to face the target
            delta_x = self.x_goal - self.current_x
            delta_y = self.y_goal - self.current_y
            target_angle = atan2(delta_y, delta_x)

            # Convert the robot's orientation quaternion to Euler angles
            orientation_q = (self.current_orientation.x, self.current_orientation.y,
                            self.current_orientation.z, self.current_orientation.w)
            _, _, current_angle = tf.transformations.euler_from_quaternion(orientation_q)

            # Calculate the required angular rotation
            angular_rotation = target_angle - current_angle

            print("target: ")
            print(target_angle)
            print("current:" )
            print(current_angle)
            print("angular: " )
            print(angular_rotation)

            # Adjust the angular rotation to ensure the robot takes the shortest path
            if angular_rotation > pi:
                angular_rotation -= 2 * pi
            elif angular_rotation < -pi:
                angular_rotation += 2 * pi

            # Set the angular velocity based on the required rotation
            cmd_vel.angular.z = 1.5 * angular_rotation

            # Publish the velocity command
            self.vel_pub.publish(cmd_vel)

    def loop(self):
        # Keep the node running until it's stopped
        rospy.spin()


if __name__ == '__main__':
    try:
        BoxFollower()
    except rospy.ROSInterruptException:
        pass
