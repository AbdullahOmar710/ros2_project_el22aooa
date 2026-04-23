# Exercise 3 - If green object is detected, and above a certain size, then send a message (print or use lab2)

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
        
        # 1. Initialise flags and sensitivity
        self.green_detected = False
        self.sensitivity = 15
        
        # 2. Set up CvBridge and Subscriber
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)

    def callback(self, data):
        try:
            # Convert ROS image to OpenCV image
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            
            # Reset flag every frame
            self.green_detected = False

            # Convert to HSV and create green mask
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
            mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)

            # 3. FIND CONTOURS
            # This finds the boundaries of the green blobs
            contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                # Find the largest green object in view
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)

                # 4. AREA THRESHOLD
                # A suitable area for a Gazebo object is usually between 500 and 1000
                if area > 800:
                    self.green_detected = True
                    
                    # Calculate the center (Moments)
                    M = cv2.moments(c)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])

                        # 5. DRAW TRACKING CIRCLE
                        # Find the radius to draw a perfect circle around the object
                        (x, y), radius = cv2.minEnclosingCircle(c)
                        center = (int(x), int(y))
                        
                        # Draw on the original image: (image, center, radius, color, thickness)
                        cv2.circle(image, center, int(radius), (0, 255, 0), 2)
                        # Draw a small dot at the exact center
                        cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)

            # 6. ACTION BASED ON FLAG
            if self.green_detected:
                print("TARGET ACQUIRED: Green object detected!")
            else:
                print("Searching...")

            # Show the live tracking feed
            cv2.imshow('Object_Tracking', image)
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
