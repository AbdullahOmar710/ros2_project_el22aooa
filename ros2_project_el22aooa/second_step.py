# Exercise 2 - detecting two colours, and filtering out the third colour and background.

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

        # 1. Initialise sensitivity 
        self.sensitivity = 10
        
        # 2. Set up CvBridge and Subscriber
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)

    def callback(self, data):
        try:
            # Convert the received image into an opencv image
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            
            # Convert the BGR image into an HSV image
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # COLOUR 1: GREEN 
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
            green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)

            # COLOUR 2: RED 
            hsv_red_lower = np.array([0, 100, 100])
            hsv_red_upper = np.array([10, 255, 255])
            red_mask = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)

            # 3. COMBINE MASKS
            combined_mask = cv2.bitwise_or(green_mask, red_mask)

            # 4. APPLY MASK TO ORIGINAL IMAGE
            filtered_img = cv2.bitwise_and(image, image, mask=combined_mask)

            # 5. SHOW RESULTS
            cv2.namedWindow('Combined_Filter', cv2.WINDOW_NORMAL)
            cv2.imshow('Combined_Filter', filtered_img)
            cv2.resizeWindow('Combined_Filter', 320, 240)
            cv2.waitKey(3)

        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error: {e}')

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
