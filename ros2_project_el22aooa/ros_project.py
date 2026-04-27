import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class MazeExplorer(Node):
    def __init__(self):
        super().__init__('el22aooa_explorer')
        
        self.bridge = CvBridge()
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Room coordinates from RViz
        self.waypoints = [
            [2.23, 1.68],   # Starting point
            [2.84, -6.63],  # Green box room
            [-6.26, -3.6],   # Red box room
            [-1.48, -10.4]  # Blue box room
        ]
        self.current_waypoint_index = 0
        self.target_reached = False

        # Tracking sets to prevent duplicate terminal messages
        self.logged_detections = set()
        self.logged_navigation = set()
        
        # Camera subscription
        self.camera_subscriber = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.vision_callback, 
            10
        )
        
        self.get_logger().info("System initialised. Exploration starting.")
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints) or self.target_reached:
            return

        # Terminal feedback for the current target
        labels = ["Starting point", "Green room", "Red room", "Blue room"]
        target_name = labels[self.current_waypoint_index]
        
        if target_name not in self.logged_navigation:
            self.get_logger().info(f"Navigating to {target_name}.")
            self.logged_navigation.add(target_name)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        x, y = self.waypoints[self.current_waypoint_index]
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0 
        
        self.nav_client.wait_for_server()
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            return

        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        labels = ["Starting point", "Green room", "Red room", "Blue room"]
        self.get_logger().info(f"Arrived at {labels[self.current_waypoint_index]}.")
        
        self.current_waypoint_index += 1
        self.send_next_waypoint()

    def vision_callback(self, data):
        if self.target_reached:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            targets = {
                'Red Object':   ([0, 100, 100], [10, 255, 255], (0, 0, 255)),
                'Green Object': ([50, 100, 100], [70, 255, 255], (0, 255, 0)),
                'Blue Object':  ([110, 100, 100], [130, 255, 255], (255, 0, 0))
            }

            for label, (lower, upper, color_bgr) in targets.items():
                mask = cv2.inRange(hsv_frame, np.array(lower), np.array(upper))
                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                
                if contours:
                    best_match = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(best_match)
                    
                    if area > 500:
                        # Log detection once per object
                        if label not in self.logged_detections:
                            self.get_logger().info(f"{label} detected.")
                            self.logged_detections.add(label)

                        x, y, w, h = cv2.boundingRect(best_match)
                        cv2.rectangle(frame, (x, y), (x+w, y+h), color_bgr, 2)
                        cv2.putText(frame, label, (x, y-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

                        # Final approach and stop logic
                        if label == 'Blue Object' and area > 150000 and self.current_waypoint_index == 3:
                            self.get_logger().info("Final target reached. Mission complete.")
                            self.target_reached = True
                            if hasattr(self, 'goal_handle'):
                                self.goal_handle.cancel_goal_async()

            cv2.imshow("Maze Explorer: Vision Feed", frame)
            cv2.waitKey(1)
            
        except Exception as error:
            self.get_logger().error(f"Vision error: {error}")

def main(args=None):
    rclpy.init(args=args)
    explorer = MazeExplorer()
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()\

if __name__ == '__main__':
    main()
