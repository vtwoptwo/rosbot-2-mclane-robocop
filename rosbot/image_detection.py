#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ultralytics import YOLO
from sensor_msgs.msg import Range
import ast
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image, LaserScan, Range
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ultralytics import YOLO
from collections import Counter
import socket
import math
import ast



class FreshyFruits:
    def __init__(self):
        rospy.init_node('freshy_fruit', anonymous=True)
        # Fruit key ['Fresh Apple', 'Fresh Banana', 'Fresh Guava', 'Fresh Orange', 'Fresh Pomegranate', 'Rotten Apple', 'Rotten Banana', 'Rotten Guava', 'Rotten Orange', 'Rotten Pomegranate', 'Stale Apple', 'Stale Banana', 'Stale Guava', 'Stale Orange', 'Stale Pomegranate']
        self.fruit_rules = rospy.get_param('/fruit_rules', "{0:0,1:0,2:0,3:0,4:0,5:0,6:0,7:0,8:0,9:0,10:0,11:0,12:0,13:0,14:0}")
        self.fruit_rules = ast.literal_eval(self.fruit_rules)
        self.tcp_server_ip = '10.205.3.76'
        self.port = rospy.get_param('/port', 6667)
        self.running = False
        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.box_pub = rospy.Publisher('/camera/boxes', Image, queue_size=1)

        # CV Bridge
        self.bridge = cv_bridge.CvBridge()

        # Image processing
        self.model = YOLO('/home/v/mclane-robocop/catkin_ws/src/mclane_robocop/model/weights/best.pt')
        self.twist = Twist()

        # State flags
        self.turn_direction = 1
        self.obs_detected = False
        self.img_detected = False
        self.ir_detected = False


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except cv_bridge.CvBridgeError as e:
            #print(e)
            return    
        results = self.model.predict(cv_image)
        self.process_object_detection(results, cv_image)

    def process_object_detection(self, results, image):
        for result in results:
            probs = result.boxes.conf
            cl = result.boxes.cls
            probs = probs.cpu()
            cl = cl.cpu()
            probs_numpy = probs.numpy()
            cl_numpy = cl.numpy()
            #print(probs_numpy)
          
            if np.any(probs_numpy > 0.85):
                #print("All probabilities are greater than 0.9")
                self.img_detected = True
                maxim = probs_numpy.argmax()
                message = f"{probs_numpy[maxim]}bam{cl_numpy[maxim]}"
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                    for i in range(50):
                        s.sendto(message.encode(), ('10.205.3.76', self.port))
                        #print(f"Message sent: {message}")
                #print("Detected high confidence object")
                annotated_frame = result.plot()
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                    self.box_pub.publish(ros_image)
                except cv_bridge.CvBridgeError as e:
                    rospy.logerr(e)
                    #print(e)
  
                    
    def run(self):
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    fresh = FreshyFruits()
    fresh.run()

