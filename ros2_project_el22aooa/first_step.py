# Exercise 1 - Display an image of the camera feed to the screen

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal

class colourIdentifier(Node):
    def __init__(self):
        super().__init__('cI')
        
        # Initialise CvBridge and subscriber
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        
        # Define sensitivity for color detection
        self.sensitivity = 15

    def callback(self, data):
        try:
            # 1. Convert ROS image to OpenCV BGR format
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            
            # 2. Convert BGR image to HSV format
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # 3. Define the green range for masking
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
            
            # 4. Create the mask (finds all green pixels)
            green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
            
            # 5. Apply the mask to the original image
            filtered_img = cv2.bitwise_and(image, image, mask=green_mask)

            # 6. Show the original feed
            cv2.imshow('Original_Feed', image)
            
            # 7. Show the filtered (green-only) feed
            cv2.namedWindow('Filtered_Feed', cv2.WINDOW_NORMAL)
            cv2.imshow('Filtered_Feed', filtered_img)
            cv2.resizeWindow('Filtered_Feed', 320, 240)
            
            cv2.waitKey(3)

        except CvBridgeError as e:
            self.get_logger().error(f'Could not convert image: {e}')

def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()

    rclpy.init(args=None)
    cI = colourIdentifier()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(cI,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            continue
    except (KeyboardInterrupt, ROSInterruptException):
        pass

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
