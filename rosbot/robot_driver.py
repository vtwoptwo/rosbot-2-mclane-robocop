#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image, LaserScan, Range
from geometry_msgs.msg import Twist
#from ultralytics import YOLO
from collections import deque



class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)

        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        #self.ir_fl_sub = rospy.Subscriber('/range/fl', Range, self.ir_callback)  # Front-left IR sensor
        #self.ir_fr_sub = rospy.Subscriber('/range/fr', Range, self.ir_callback)  # Front-right IR sensor

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #self.box_pub = rospy.Publisher('/camera/boxes', Image, queue_size=1)

        # CV Bridge
        self.bridge = cv_bridge.CvBridge()

        # Image processing
        #self.model = YOLO('/home/v/mclane-robocop/catkin_ws/src/mclane_robocop/model/weights/best.pt')

        self.twist = Twist()

        self.ir_ranges = []
        self.num_readings = 10  # Number of readings to consider for averaging
        self.ir_range_queue = deque(maxlen=10)  # Max length of 10 to store the latest 10 consequtive readings
        self.consecutive_count = 0
        # State flags
        self.turn_direction = 1
        self.obs_detected = False
        self.img_detected = False
        self.ir_detected = False

    def backup_and_reset(self):
        self.twist.linear.x = -0.1
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(2)  

      
        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)
        self.ir_detected = False

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except cv_bridge.CvBridgeError as e:
            print(e)
            return

        self.process_line_following(cv_image)


    def process_line_following(self, image):
        """"
        1. Create a mask to detect white lines in the bottom quarter of the image
        2. Calculate the centroid of the detected white line
        3. Adjust the angular velocity based on the error between the centroid and the center of the image
        
        """
        if not self.obs_detected:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 

            lower_white = np.array([0, 0, 200])
            upper_white = np.array([180, 50, 255])
            mask = cv2.inRange(hsv, lower_white, upper_white)
            #cv2.imwrite('output_image.jpg', mask)
            
            h, w, d = image.shape
            search_bot = 3 * h // 4  # Bottom quarter starts at three-fourths of the image height
            mask[0:search_bot, 0:w] = 0  # Mask out everything above the bottom quarter
            cv2.imwrite('output_image.jpg', mask)
            M = cv2.moments(mask)

            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                # Proportional controller (P-controller)
                center = w / 2
                max_ang_vel = 0.25  # Maximum angular velocity
                min_ang_vel = -0.25  # Minimum angular velocity
                Kp = 0.01  # Proportional gain
                error = cx - center
                ang_vel = np.clip(-Kp * error, min_ang_vel, max_ang_vel)
                self.twist.linear.x = 0.08
                self.twist.angular.z = ang_vel
                print(ang_vel)
                left_pixels = np.sum(mask[search_bot:, :w//2] > 0)
                right_pixels = np.sum(mask[search_bot:, w//2:] > 0)
                if left_pixels > right_pixels:
                    self.turn_direction = 1
                else:
                    self.turn_direction = -1
                self.cmd_vel_pub.publish(self.twist)
            else:
                # If the line is not detected, start turning in the last known direction
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.3 * self.turn_direction  # Turn left or right
                self.cmd_vel_pub.publish(self.twist)

    def laser_callback(self, msg):
        ranges = np.array(msg.ranges)
        threshold_distance = 0.18
        forward_range = 120 # Front range in degrees

        if np.any(ranges < threshold_distance):
            min_index = np.argmin(ranges)
            n_ranges = len(ranges)
            one_degree = n_ranges / 360

            # Calculate the start and end indices for the front range
            start_index = int((180 - forward_range / 2) * one_degree)
            end_index = int((180 + forward_range / 2) * one_degree)

            # Check if the minimum distance is within the front range
            if min_index >= start_index and min_index <= end_index:
                self.obs_detected = True
                print("OBSTACLE IN FRONT")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)

                # Turn 180 degrees
                self.twist.angular.z = 0.157  # Adjust angular velocity as needed

                if min_index < n_ranges / 2:
                    self.twist.angular.z = -self.twist.angular.z
                else:
                    self.twist.angular.z = self.twist.angular.z

                turn_start_time = rospy.Time.now()
                while rospy.Time.now() - turn_start_time < rospy.Duration(1):  # Adjust duration as needed
                    self.cmd_vel_pub.publish(self.twist)
    
            else:
                print("OBSTACLE NOT IN FRONT")
                self.obs_detected = False


    def run(self):
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    rc = RobotController()
    rc.run()
