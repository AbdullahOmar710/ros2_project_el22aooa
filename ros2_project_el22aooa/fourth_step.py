# Exercise 4 - following a colour (green) and stopping upon sight of another (blue).

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


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        # 1. Publisher for movement
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 2. Flags and Variables
        self.green_area = 0
        self.blue_detected = False
        self.sensitivity = 15
        
        # 3. CV Setup
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # --- GREEN DETECTION (FOLLOW TARGET) ---
            green_lower = np.array([60 - self.sensitivity, 100, 100])
            green_upper = np.array([60 + self.sensitivity, 255, 255])
            green_mask = cv2.inRange(hsv_image, green_lower, green_upper)
            g_contours, _ = cv2.findContours(green_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(g_contours) > 0:
                c = max(g_contours, key=cv2.contourArea)
                self.green_area = cv2.contourArea(c)
                # Draw green tracking
                (x, y), radius = cv2.minEnclosingCircle(c)
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            else:
                self.green_area = 0

            # --- BLUE DETECTION (STOP SIGN) ---
            # Blue hue is usually around 120
            blue_lower = np.array([120 - self.sensitivity, 100, 100])
            blue_upper = np.array([120 + self.sensitivity, 255, 255])
            blue_mask = cv2.inRange(hsv_image, blue_lower, blue_upper)
            b_contours, _ = cv2.findContours(blue_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            
            # If we see any significant blue, set the flag
            self.blue_detected = len(b_contours) > 0 and cv2.contourArea(max(b_contours, key=cv2.contourArea)) > 500

            # Show the feed
            cv2.imshow('Robot_Vision', image)
            cv2.waitKey(3)

        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error: {e}')

    # --- MOVEMENT METHODS ---
    def walk_forward(self):
        msg = Twist()
        msg.linear.x = 0.2  # Move forward at 0.2 m/s
        self.publisher.publish(msg)

    def walk_backward(self):
        msg = Twist()
        msg.linear.x = -0.2 # Move backward at 0.2 m/s
        self.publisher.publish(msg)

    def stop(self):
        msg = Twist() # All values default to 0.0
        self.publisher.publish(msg)

def main():
    rclpy.init(args=None)
    robot = Robot()

    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            # PRIORITY 1: STOP if blue is seen
            if robot.blue_detected:
                print("STOP: Blue detected!")
                robot.stop()
            
            # PRIORITY 2: FOLLOW if green is seen
            elif robot.green_area > 0:
                # If area is small, we are far away -> move forward
                if robot.green_area < 5000:
                    print(f"Target far (Area: {int(robot.green_area)}) - Moving Forward")
                    robot.walk_forward()
                # If area is very large, we are too close -> move backward
                elif robot.green_area > 15000:
                    print(f"Target too close (Area: {int(robot.green_area)}) - Moving Backward")
                    robot.walk_backward()
                # If in between, stay still
                else:
                    print("Target in range - Holding position")
                    robot.stop()
            
            # PRIORITY 3: If nothing is seen, stop
            else:
                print("Searching for targets...")
                robot.stop()
            
            time.sleep(0.1) # Loop rate

    except (KeyboardInterrupt, ROSInterruptException):
        robot.stop()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
